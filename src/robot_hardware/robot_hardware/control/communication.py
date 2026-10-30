import math
import socket

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Bool

from robot_hardware.utils.yaml_loader import load_node_params


class CommunicationNode(Node):

    # Initialize node
    def __init__(self):
        super().__init__("communication_node")

        self._boost_active = False

        self._load_config()

        self.esp32_addr = (self.esp32_ip, self.esp32_port)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.01)

        self.create_subscription(
            Twist, "/cmd_vel_raw", self.cmd_callback, 10
        )

        self.pub_pwm = self.create_publisher(Pose2D, "/pwm_feedback", 10)
        self.pub_imu = self.create_publisher(Pose2D, "/imu/angles", 10)

        self.send_config()

        self.create_timer(5.0, self.send_config)

        self.create_subscription(
            Bool, "/boost_active", self.boost_callback, 10
        )

        self.get_logger().info(
            f"Communication Node Started\n"
            f"  TX UDP: {self.esp32_ip}:{self.esp32_port}\n"
            f"  PWM  min={self.pwm_min} max={self.pwm_max} "
            f"turn={self.pwm_turn} straight={self.pwm_straight}"
        )

    # Boost callback
    def boost_callback(self, msg):
        if msg.data == self._boost_active:
            return
        self._boost_active = msg.data
        self.get_logger().info(f"[BOOST] {'ACTIVE' if msg.data else 'OFF'}")
        self.send_config()

    # Send config
    def send_config(self):
        if self._boost_active:
            _min = self.boost_pwm_min
            _max = self.boost_pwm_max
            _turn = self.boost_pwm_turn
            _straight = self.boost_pwm_straight
        else:
            _min = self.pwm_min
            _max = self.pwm_max
            _turn = self.pwm_turn
            _straight = self.pwm_straight
        try:
            cfg = (f"C:{_min},{_max},{_turn},{_straight},"
                   f"{self.wheel_base:.4f},{self.wheel_radius:.4f},{self.max_wheel_vel:.4f}\n")
            self.sock.sendto(cfg.encode(), self.esp32_addr)
            self.get_logger().info(
                f"[CFG] sent config to ESP32: {cfg.strip()}",
                throttle_duration_sec=5.0
            )
        except Exception as e:
            self.get_logger().error(f"[CFG] send error: {e}")

    # Load config
    def _load_config(self):
        params = load_node_params(self)
        if params is not None:
            for k, v in params.items():
                setattr(self, k, v)

    # Cmd callback
    def cmd_callback(self, msg):
        vx = msg.linear.x
        wz = msg.angular.z

        if not self.motor_enable:
            self.get_logger().warn(
                "motor_enable=False -> data NOT sent to ESP32",
                throttle_duration_sec=2.0
            )
            return

        try:
            self.sock.sendto(
                f"{vx:.4f},{wz:.4f}\n".encode(),
                self.esp32_addr
            )
        except Exception as e:
            self.get_logger().error(f"UDP send error: {e}")
            return

        try:
            data, _ = self.sock.recvfrom(64)
            parts = data.decode().strip().split(',')
            if len(parts) >= 2:
                pwm_l = int(parts[0])
                pwm_r = int(parts[1])
                fb = Pose2D()
                fb.x = float(pwm_l)
                fb.y = float(pwm_r)
                self.pub_pwm.publish(fb)

                if len(parts) >= 3:
                    try:
                        yaw_deg = float(parts[2])
                        yaw_rad = math.radians(yaw_deg)
                        while yaw_rad > math.pi:
                            yaw_rad -= 2.0 * math.pi
                        while yaw_rad < -math.pi:
                            yaw_rad += 2.0 * math.pi
                        imu = Pose2D()
                        imu.x = yaw_rad
                        self.pub_imu.publish(imu)
                    except (ValueError, IndexError):
                        pass

                self.get_logger().info(
                    f"[PWM] L={pwm_l:+4d}  R={pwm_r:+4d}  (vx={vx:.3f} wz={wz:.3f})",
                    throttle_duration_sec=1.0
                )
        except socket.timeout:
            self.get_logger().info(
                f"[Twist] vx={vx:.3f}  wz={wz:.3f}  (no ESP32 response)",
                throttle_duration_sec=1.0
            )

    # Destroy node
    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


# Main entry
def main(args=None):
    rclpy.init(args=args)
    node = CommunicationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()