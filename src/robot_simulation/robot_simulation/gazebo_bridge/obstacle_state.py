import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.boolean_pb2 import Boolean
from gz.transport13 import Node as GzNode

from robot_hardware.utils.yaml_loader import load_node_params
from robot_hardware.utils.pixel_world import pixel_to_world


class ObstacleStatePublisher(Node):

    # Initialize node
    def __init__(self):
        super().__init__("obstacle_state_publisher")

        params = load_node_params(self)
        self.a = params["a"]
        self.b = params["b"]
        self.c = params["c"]
        self.d = params["d"]
        self.e = params["e"]
        self.f = params["f"]
        self.MAX_OBSTACLES = params["max_obstacles"]

        self.obstacles = []

        self.gz_node = GzNode()
        self._gz_msg = Pose_V()
        for _ in range(self.MAX_OBSTACLES):
            self._gz_msg.pose.add()

        self.marker_pub = self.create_publisher(MarkerArray, "/obstacle_markers", 10)

        self.create_subscription(Float32MultiArray, "/vision/obstacles", self.obs_callback, 10)
        self.create_timer(0.1, self.publish_obstacles)

        self.get_logger().info(f"ObstacleStatePublisher started ({self.MAX_OBSTACLES} max)")

    # Obs callback
    def obs_callback(self, msg):
        data = msg.data
        self.obstacles = []
        for i in range(0, len(data), 4):
            if i + 3 < len(data):
                px, py = data[i], data[i + 1]
                wx, wy = pixel_to_world(px, py, self.a, self.b, self.c, self.d, self.e, self.f)
                self.obstacles.append((wx, wy))

    # Publish obstacles
    def publish_obstacles(self):
        markers = MarkerArray()
        markers_present = len(self.obstacles)

        for i in range(self.MAX_OBSTACLES):
            gz_pose = self._gz_msg.pose[i]
            gz_pose.name = f"obstacle_{i}"

            if i < markers_present:
                wx, wy = self.obstacles[i]
                gz_pose.position.x = wx
                gz_pose.position.y = wy
                gz_pose.position.z = 0.05
                gz_pose.orientation.w = 1.0

                marker = Marker()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = "odom"
                marker.ns = "obstacle"
                marker.id = i
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = wx
                marker.pose.position.y = wy
                marker.pose.position.z = 0.05
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.4
                marker.scale.y = 0.2
                marker.scale.z = 0.1
                marker.color.a = 0.9
                marker.color.r = 0.2
                marker.color.g = 0.2
                marker.color.b = 1.0
                markers.markers.append(marker)
            else:
                gz_pose.position.z = -10.0
                gz_pose.orientation.w = 1.0

                marker = Marker()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = "odom"
                marker.ns = "obstacle"
                marker.id = i
                marker.action = Marker.DELETE
                markers.markers.append(marker)

        self.gz_node.request(
            "/world/default/set_pose_vector",
            self._gz_msg, Pose_V, Boolean, 100
        )
        self.marker_pub.publish(markers)

        if markers_present > 0:
            self.get_logger().info(
                f"[OBS] {markers_present} obstacles",
                throttle_duration_sec=0.5,
            )


# Main entry
def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStatePublisher()
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