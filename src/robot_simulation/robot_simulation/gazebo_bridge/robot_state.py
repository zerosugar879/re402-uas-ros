import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose as RosPose, Pose2D, TransformStamped
from tf2_ros import TransformBroadcaster

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.boolean_pb2 import Boolean
from gz.transport13 import Node as GzNode

from robot_hardware.utils.yaml_loader import load_node_params
from robot_hardware.utils.pixel_world import pixel_to_world


class RobotStatePublisher(Node):

    # Initialize node
    def __init__(self):
        super().__init__("robot_state_publisher")

        params = load_node_params(self)
        self.a = params["a"]
        self.b = params["b"]
        self.c = params["c"]
        self.d = params["d"]
        self.e = params["e"]
        self.f = params["f"]
        self.invert_heading = params["invert_heading"]

        self.pixel_x = 0.0
        self.pixel_y = 0.0
        self.has_pose = False

        self.heading_fused = None
        self.heading_imu = None
        self.heading_tag = None

        self.gz_node = GzNode()
        self._gz_msg = Pose_V()
        self._gz_msg.pose.add()

        self.ros_pub = self.create_publisher(RosPose, "/robot/target_pose", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Pose2D, "/vision/robot_pose", self.robot_callback, 10)
        self.create_subscription(Pose2D, "/imu/angles", self.imu_callback, 10)
        self.create_subscription(Pose2D, "/robot/fused_heading", self.fused_callback, 10)

        self.create_timer(0.1, self.publish_pose)

        self.get_logger().info("RobotStatePublisher started (affine + set_pose)")

    # Robot callback
    def robot_callback(self, msg):
        detected = msg.x != 0.0 or msg.y != 0.0
        if detected:
            self.pixel_x = msg.x
            self.pixel_y = msg.y
            self.has_pose = True
            self.heading_tag = msg.theta

    # IMU callback
    def imu_callback(self, msg):
        self.heading_imu = msg.x

    # Fused callback
    def fused_callback(self, msg):
        self.heading_fused = msg.x

    # Get heading
    def get_heading(self):
        if self.heading_fused is not None:
            return self.heading_fused
        if self.heading_imu is not None:
            return self.heading_imu
        if self.heading_tag is not None:
            return self.heading_tag
        return 0.0

    # Set Gazebo pose
    def set_gz_pose(self, wx, wy, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        p = self._gz_msg.pose[0]
        p.name = "robot"
        p.position.x = wx
        p.position.y = wy
        p.position.z = 0.0
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = qz
        p.orientation.w = qw
        self.gz_node.request(
            "/world/default/set_pose_vector",
            self._gz_msg, Pose_V, Boolean, 100
        )

    # Publish pose
    def publish_pose(self):
        if not self.has_pose:
            return

        wx, wy = pixel_to_world(self.pixel_x, self.pixel_y, self.a, self.b, self.c, self.d, self.e, self.f)

        raw_heading = self.get_heading()
        heading = -raw_heading if self.invert_heading else raw_heading
        self.set_gz_pose(wx, wy, heading)

        ros_msg = RosPose()
        ros_msg.position.x = wx
        ros_msg.position.y = wy
        ros_msg.position.z = 0.0
        ros_msg.orientation.w = math.cos(heading / 2.0)
        ros_msg.orientation.z = math.sin(heading / 2.0)
        self.ros_pub.publish(ros_msg)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "robot"
        t.transform.translation.x = wx
        t.transform.translation.y = wy
        t.transform.translation.z = 0.0
        t.transform.rotation.w = math.cos(heading / 2.0)
        t.transform.rotation.z = math.sin(heading / 2.0)
        self.tf_broadcaster.sendTransform(t)

        head_deg = math.degrees(heading)
        self.get_logger().info(
            f"[TWIN] Robot | Pixel=({self.pixel_x:.0f},{self.pixel_y:.0f}) | "
            f"World=({wx:.3f},{wy:.3f}) | Yaw={head_deg:+.1f}°",
            throttle_duration_sec=0.5,
        )


# Main entry
def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()
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