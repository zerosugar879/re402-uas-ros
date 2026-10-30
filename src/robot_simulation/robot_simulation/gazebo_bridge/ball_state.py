import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from visualization_msgs.msg import Marker

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.boolean_pb2 import Boolean
from gz.transport13 import Node as GzNode

from robot_hardware.utils.yaml_loader import load_node_params
from robot_hardware.utils.pixel_world import pixel_to_world


class BallStatePublisher(Node):

    # Initialize node
    def __init__(self):
        super().__init__("ball_state_publisher")

        params = load_node_params(self)
        self.a = params["a"]
        self.b = params["b"]
        self.c = params["c"]
        self.d = params["d"]
        self.e = params["e"]
        self.f = params["f"]

        self.last_wx = 0.0
        self.last_wy = 0.0
        self.has_position = False

        self.gz_node = GzNode()
        self._gz_msg = Pose_V()
        self._gz_msg.pose.add()

        self.marker_pub = self.create_publisher(Marker, "/ball_marker", 10)

        self.create_subscription(Pose2D, "/vision/ball_position", self.ball_callback, 10)
        self.create_timer(0.1, self.publish_ball)

        self.get_logger().info("BallStatePublisher started (affine)")

    # Ball callback
    def ball_callback(self, msg):
        detected = msg.x >= 0.0 and msg.y >= 0.0
        if detected:
            self.last_wx, self.last_wy = pixel_to_world(msg.x, msg.y, self.a, self.b, self.c, self.d, self.e, self.f)
            self.has_position = True

    # Set Gazebo pose
    def set_gz_pose(self, wx, wy):
        p = self._gz_msg.pose[0]
        p.name = "ball"
        p.position.x = wx
        p.position.y = wy
        p.position.z = 0.11
        p.orientation.w = 1.0

        self.gz_node.request(
            "/world/default/set_pose_vector",
            self._gz_msg, Pose_V, Boolean, 100
        )

    # Publish ball
    def publish_ball(self):
        if not self.has_position:
            return

        self.set_gz_pose(self.last_wx, self.last_wy)

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "odom"
        marker.ns = "ball"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.last_wx
        marker.pose.position.y = self.last_wy
        marker.pose.position.z = 0.11
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.22
        marker.scale.y = 0.22
        marker.scale.z = 0.22
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

        self.get_logger().info(
            f"[BALL] world=({self.last_wx:.3f},{self.last_wy:.3f})",
            throttle_duration_sec=0.5,
        )


# Main entry
def main(args=None):
    rclpy.init(args=args)
    node = BallStatePublisher()
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