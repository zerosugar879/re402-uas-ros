import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Bool, Float32MultiArray

from robot_hardware.utils.yaml_loader import load_node_params
from robot_hardware.utils.timeout import is_data_fresh


class ControllerNode(Node):

    # Initialize node
    def __init__(self):
        super().__init__("controller_node")

        self._load_config()
        self._load_field_boundary()

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.last_tag_seen_time = None
        self.last_robot_msg_time = None
        self.last_ball_msg_time = None
        self.estimated_heading = 0.0
        self.ball_x = -1.0
        self.ball_y = -1.0

        self.imu_yaw = 0.0

        self.last_imu_time = None
        self.drift_offset = None

        self._boost_start_time = None
        self._last_cmd_was_moving = False
        self._stop_start_time = None
        self._stop_release = False
        self.pwm_l = 0
        self.pwm_r = 0

        self.path = []

        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel_raw", 10)
        self.pub_fused_heading = self.create_publisher(Pose2D, "/robot/fused_heading", 10)
        self.pub_boost = self.create_publisher(Bool, "/boost_active", 10)

        self.create_subscription(Pose2D, "/vision/robot_pose", self.robot_callback, 10)
        self.create_subscription(Pose2D, "/vision/ball_position", self.ball_callback, 10)
        self.create_subscription(Pose2D, "/pwm_feedback", self.pwm_callback, 10)
        self.create_subscription(Pose2D, "/imu/angles", self.imu_callback, 10)
        self.create_subscription(Float32MultiArray, "/planned_path", self.path_callback, 10)

        self.create_timer(0.05, self.control_loop)
        self.start_time = self.get_clock().now()

        self.get_logger().info(
            f"Controller Node Started (straight_linear_vel={self.straight_linear_vel}, "
            f"kp_v={self.kp_v}, kp_w={self.kp_w})"
        )

    # Load config
    def _load_config(self):
        params = load_node_params(self)
        if params is not None:
            for k, v in params.items():
                setattr(self, k, v)

    # Load boundary
    def _load_field_boundary(self):
        import json
        import os
        path = os.path.expanduser("~/field_boundary.json")
        if os.path.exists(path):
            try:
                with open(path) as f:
                    data = json.load(f)
                self.field_left = data.get("field_left", self.field_left)
                self.field_right = data.get("field_right", self.field_right)
                self.field_top = data.get("field_top", self.field_top)
                self.field_bottom = data.get("field_bottom", self.field_bottom)
                self.get_logger().info(
                    f"[BOUNDARY] Loaded: L={self.field_left} R={self.field_right} "
                    f"T={self.field_top} B={self.field_bottom}"
                )
            except Exception as e:
                self.get_logger().warn(f"[BOUNDARY] Failed to load: {e}")

    # Robot callback
    def robot_callback(self, msg):
        now = self.get_clock().now()
        tag_detected = (msg.x != 0.0 or msg.y != 0.0)
        if tag_detected:
            self.robot_x = msg.x
            self.robot_y = msg.y
            self.estimated_heading = msg.theta
            self.last_tag_seen_time = now
            self.last_robot_msg_time = now

    # Ball callback
    def ball_callback(self, msg):
        self.last_ball_msg_time = self.get_clock().now()
        self.ball_x = msg.x
        self.ball_y = msg.y

    # PWM callback
    def pwm_callback(self, msg):
        self.pwm_l = int(msg.x)
        self.pwm_r = int(msg.y)

    # IMU callback
    def imu_callback(self, msg):
        self.imu_yaw = msg.x
        self.last_imu_time = self.get_clock().now()

    # Path callback
    def path_callback(self, msg):
        self.path = []
        for i in range(0, len(msg.data), 2):
            if i + 1 < len(msg.data):
                self.path.append((float(msg.data[i]), float(msg.data[i + 1])))

    # Print status
    def print_status(self, err_deg, dist, speed, mode, wz):
        has_ball = self.ball_x >= 0 and self.ball_y >= 0

        if has_ball:
            target_str = "Ball"
        else:
            target_str = "None"

        imu_yaw_deg = math.degrees(self.imu_yaw)
        output = (
            f"[CTRL] {mode}\n"
            f"       Target={target_str}\n"
            f"       Yaw={imu_yaw_deg:+.1f}°\n"
            f"       Err={err_deg:+.1f}°\n"
            f"       Dist={dist:.1f} px\n"
            f"       V={speed:.2f} m/s\n"
            f"       W={wz:.2f} rad/s\n"
            f"       PWM=({self.pwm_l},{self.pwm_r})"
        )
        print(output, flush=True)

        with open("/tmp/robot_status.txt", "w") as f:
            f.write(output + "\n")

        self.get_logger().info(
            f"[CTRL] {mode} Target={target_str} "
            f"IMU={imu_yaw_deg:+.1f}° Err={err_deg:+.1f}° Dist={dist:.0f} "
            f"V={speed:.2f} W={wz:.2f} PWM=({self.pwm_l},{self.pwm_r})",
            throttle_duration_sec=2.0
        )

    # Fuse heading
    def _compute_fused_heading(self, tag_heading, imu_fresh, imu_calibrated, tag_is_fresh):
        if imu_fresh and tag_is_fresh:
            raw_drift = self.imu_yaw - tag_heading
            while raw_drift > math.pi:
                raw_drift -= 2 * math.pi
            while raw_drift < -math.pi:
                raw_drift += 2 * math.pi

            if self.drift_offset is None:
                self.drift_offset = raw_drift

            drift_error = raw_drift - self.drift_offset
            while drift_error > math.pi:
                drift_error -= 2 * math.pi
            while drift_error < -math.pi:
                drift_error += 2 * math.pi
            self.drift_offset += self.drift_gain * drift_error

        if imu_calibrated:
            heading = self.imu_yaw - self.drift_offset
        else:
            heading = tag_heading

        while heading > math.pi:
            heading -= 2 * math.pi
        while heading < -math.pi:
            heading += 2 * math.pi
        return heading

    # Control loop
    def control_loop(self):
        cmd = Twist()

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.startup_delay:
            self.path = []
            mode = f"STARTUP ({elapsed:.1f}/{self.startup_delay:.0f}s)"
            self.print_status(0, 0, 0, mode, 0)
            self.pub_cmd_vel.publish(cmd)
            return

        now = self.get_clock().now()
        robot_fresh = is_data_fresh(now, self.last_robot_msg_time, self.vision_timeout)
        ball_fresh = is_data_fresh(now, self.last_ball_msg_time, self.vision_timeout)
        tag_is_fresh = is_data_fresh(now, self.last_tag_seen_time, self.vision_timeout)
        has_ball = self.ball_x >= 0 and self.ball_y >= 0 and ball_fresh
        has_robot = (self.robot_x != 0.0 or self.robot_y != 0.0) and (tag_is_fresh or robot_fresh)

        imu_fresh = (
            self.last_imu_time is not None
            and (now - self.last_imu_time).nanoseconds < 1.0e9
        )
        imu_calibrated = imu_fresh and self.drift_offset is not None

        if not has_robot or not has_ball:
            self.print_status(0, 0, 0, "HOLD", 0)
            self.pub_cmd_vel.publish(Twist())
            return

        use_path = self.obstacle_avoidance and (has_ball and has_robot)

        if use_path and self.path:
            wp = self.path[0]
            dx = wp[0] - self.robot_x
            dy = wp[1] - self.robot_y
            dist_wp = math.sqrt(dx * dx + dy * dy)

            if dist_wp < self.wp_threshold:
                self.path.pop(0)
                if self.path:
                    wp = self.path[0]
                    dx = wp[0] - self.robot_x
                    dy = wp[1] - self.robot_y
                    dist_wp = math.sqrt(dx * dx + dy * dy)

            if not self.path:
                dx = self.ball_x - self.robot_x
                dy = self.ball_y - self.robot_y

            mode = "PATH FOLLOW" if self.path else "TRACKING BALL"
        else:
            dx = self.ball_x - self.robot_x
            dy = self.ball_y - self.robot_y
            mode = "TRACKING BALL"

        dist = math.sqrt(dx * dx + dy * dy)
        goal_dist = math.hypot(self.ball_x - self.robot_x, self.ball_y - self.robot_y)

        if goal_dist >= self.stop_dist:
            self._stop_release = False

        if goal_dist < self.stop_dist:
            if self._stop_start_time is None:
                self._stop_start_time = self.get_clock().now()
                self._stop_release = False
                self.print_status(0, 0, goal_dist, "STOP", 0)
                self.pub_cmd_vel.publish(cmd)
                return

            elapsed = (self.get_clock().now() - self._stop_start_time).nanoseconds / 1e9
            if elapsed < self.stop_hold_duration:
                self.print_status(0, goal_dist, 0, f"STOP ({elapsed:.1f}/{self.stop_hold_duration:.0f}s)", 0)
                self.pub_cmd_vel.publish(cmd)
                return

            self._stop_start_time = None
            self._stop_release = True
            self.path = []
            self.get_logger().info("[STOP] 5s hold done")

        while self.estimated_heading > math.pi:
            self.estimated_heading -= 2 * math.pi
        while self.estimated_heading < -math.pi:
            self.estimated_heading += 2 * math.pi

        tag_heading = self.estimated_heading + self.heading_offset

        heading = self._compute_fused_heading(tag_heading, imu_fresh, imu_calibrated, tag_is_fresh)

        heading_msg = Pose2D()
        heading_msg.x = heading
        self.pub_fused_heading.publish(heading_msg)

        target = math.atan2(dy, dx)
        error = target - heading
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi

        err_deg = math.degrees(error)

        in_motion = (goal_dist >= self.stop_dist) or self._stop_release
        if in_motion and not self._last_cmd_was_moving:
            self._boost_start_time = self.get_clock().now()
            self.get_logger().info(f"[BOOST] stop->go detected, {self.boost_duration:.1f}s boost")
        self._last_cmd_was_moving = in_motion

        boost_active = False
        if self._boost_start_time is not None:
            boost_elapsed = (self.get_clock().now() - self._boost_start_time).nanoseconds / 1e9
            boost_active = boost_elapsed < self.boost_duration

        boost_msg = Bool()
        boost_msg.data = boost_active
        self.pub_boost.publish(boost_msg)

        if boost_active:
            _straight_linear_vel = self.boost_straight_linear_vel
            _max_linear_vel = self.boost_max_linear_vel
            _min_linear_vel = self.boost_min_linear_vel
            _kp_v = self.boost_kp_v
            _kp_w = self.boost_kp_w
            _max_angular_vel = self.boost_max_angular_vel
        else:
            _straight_linear_vel = self.straight_linear_vel
            _max_linear_vel = self.max_linear_vel
            _min_linear_vel = self.min_linear_vel
            _kp_v = self.kp_v
            _kp_w = self.kp_w
            _max_angular_vel = self.max_angular_vel

        if mode == "PATH FOLLOW" and self.path:
            if abs(err_deg) < self.path_deadband:
                error = 0.0
            if dist < self.path_slowdown_dist:
                speed = max(min(self.path_follow_kp * dist, _max_linear_vel * 0.45), _min_linear_vel * 0.6)
            else:
                speed = max(min(self.path_follow_kp * dist, _max_linear_vel * 0.6), _min_linear_vel * 0.7)
            wz = max(-_max_angular_vel * 0.7, min(_max_angular_vel * 0.7, self.path_turn_kp * error))
            cmd.linear.x = float(speed)
            cmd.angular.z = float(wz)
        else:
            speed = max(min(_kp_v * dist, _max_linear_vel), _min_linear_vel)
            if abs(err_deg) > 5.0:
                wz = max(-_max_angular_vel, min(_max_angular_vel, _kp_w * error))
                cmd.linear.x = float(_straight_linear_vel * 0.5)
                cmd.angular.z = wz
            else:
                wz = float(_kp_w * error)
                cmd.linear.x = float(_straight_linear_vel)
                cmd.angular.z = wz

        self.print_status(err_deg, dist, cmd.linear.x, mode, wz)
        self.pub_cmd_vel.publish(cmd)


# Main entry
def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except rclpy._rclpy_pybind11.RCLError:
            pass


if __name__ == "__main__":
    main()