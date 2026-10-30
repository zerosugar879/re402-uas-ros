import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose2D
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge

from robot_hardware.utils.yaml_loader import load_node_params
from robot_hardware.utils.pixel_world import pixel_to_world


class VisionOverlayNode(Node):

    # Initialize node
    def __init__(self):
        super().__init__("vision_overlay")

        params = load_node_params(self)
        self.a = params["a"]
        self.b = params["b"]
        self.c = params["c"]
        self.d = params["d"]
        self.e = params["e"]
        self.f = params["f"]
        self.fb_scale = params["overhead_scale"]
        self.fb_cx = params["overhead_cx"]
        self.fb_cy = params["overhead_cy"]

        self.bridge = CvBridge()
        self.last_frame = None
        self.path_pixels = []
        self.robot_px = 0.0
        self.robot_py = 0.0
        self.ball_px = 0.0
        self.ball_py = 0.0
        self.has_robot = False
        self.has_ball = False

        self.marker_pub = self.create_publisher(Marker, "/path_marker", 10)

        self.create_subscription(Float32MultiArray, "/planned_path", self.path_callback, 10)
        self.create_subscription(Pose2D, "/vision/robot_pose", self.robot_callback, 10)
        self.create_subscription(Pose2D, "/vision/ball_position", self.ball_callback, 10)
        self.create_subscription(Image, "/overhead_camera/image_raw", self.image_callback, 10)

        self.create_timer(0.1, self.tick)

        self.get_logger().info(
            f"PathOverlay started (scale={self.fb_scale:.1f} cx={self.fb_cx:.0f} cy={self.fb_cy:.0f})"
        )

    # Pixel to world
    def pixel_to_world(self, px, py):
        return pixel_to_world(px, py, self.a, self.b, self.c, self.d, self.e, self.f)

    # World to pixel
    def world_to_overhead_pixel(self, wx, wy):
        u = self.fb_cx + wx * self.fb_scale
        v = self.fb_cy - wy * self.fb_scale
        return int(round(u)), int(round(v))

    # Path callback
    def path_callback(self, msg):
        data = msg.data
        self.path_pixels = []
        for i in range(0, len(data), 2):
            if i + 1 < len(data):
                self.path_pixels.append((data[i], data[i + 1]))
        if self.path_pixels:
            wx, wy = self.pixel_to_world(self.path_pixels[0][0], self.path_pixels[0][1])
            self.get_logger().info(
                f"[OVERLAY] path {len(self.path_pixels)} wp, "
                f"first world=({wx:.3f},{wy:.3f})",
                throttle_duration_sec=2.0
            )

    # Robot callback
    def robot_callback(self, msg):
        self.robot_px = msg.x
        self.robot_py = msg.y
        self.has_robot = msg.x != 0.0 or msg.y != 0.0

    # Ball callback
    def ball_callback(self, msg):
        self.ball_px = msg.x
        self.ball_py = msg.y
        self.has_ball = msg.x >= 0.0 and msg.y >= 0.0

    # Image callback
    def image_callback(self, msg):
        self.last_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    # Timer tick
    def tick(self):
        self.publish_markers()
        self.render()

    # Publish markers
    def publish_markers(self):
        now = self.get_clock().now().to_msg()

        path_marker = Marker()
        path_marker.header.stamp = now
        path_marker.header.frame_id = "odom"
        path_marker.ns = "path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.02
        path_marker.color.a = 1.0
        path_marker.color.r = 1.0
        path_marker.color.g = 0.5
        path_marker.color.b = 0.0

        if self.path_pixels:
            for px, py in self.path_pixels:
                wx, wy = self.pixel_to_world(px, py)
                path_marker.points.append(Point(x=wx, y=wy, z=0.02))

        self.marker_pub.publish(path_marker)

        if self.has_robot:
            rwx, rwy = self.pixel_to_world(self.robot_px, self.robot_py)
            robot_marker = Marker()
            robot_marker.header.stamp = now
            robot_marker.header.frame_id = "odom"
            robot_marker.ns = "path"
            robot_marker.id = 1
            robot_marker.type = Marker.SPHERE
            robot_marker.action = Marker.ADD
            robot_marker.pose.position.x = rwx
            robot_marker.pose.position.y = rwy
            robot_marker.pose.position.z = 0.02
            robot_marker.pose.orientation.w = 1.0
            robot_marker.scale.x = 0.10
            robot_marker.scale.y = 0.10
            robot_marker.scale.z = 0.10
            robot_marker.color.a = 1.0
            robot_marker.color.r = 0.0
            robot_marker.color.g = 0.5
            robot_marker.color.b = 1.0
            self.marker_pub.publish(robot_marker)

        if self.has_ball:
            bwx, bwy = self.pixel_to_world(self.ball_px, self.ball_py)
            ball_marker = Marker()
            ball_marker.header.stamp = now
            ball_marker.header.frame_id = "odom"
            ball_marker.ns = "path"
            ball_marker.id = 2
            ball_marker.type = Marker.SPHERE
            ball_marker.action = Marker.ADD
            ball_marker.pose.position.x = bwx
            ball_marker.pose.position.y = bwy
            ball_marker.pose.position.z = 0.02
            ball_marker.pose.orientation.w = 1.0
            ball_marker.scale.x = 0.15
            ball_marker.scale.y = 0.15
            ball_marker.scale.z = 0.15
            ball_marker.color.a = 1.0
            ball_marker.color.r = 1.0
            ball_marker.color.g = 1.0
            ball_marker.color.b = 0.0
            self.marker_pub.publish(ball_marker)

    # Render view
    def render(self):
        if self.last_frame is None:
            return

        frame = self.last_frame.copy()

        if self.path_pixels:
            pts = []
            for px, py in self.path_pixels:
                wx, wy = self.pixel_to_world(px, py)
                pixel = self.world_to_overhead_pixel(wx, wy)
                pts.append(pixel)

            if len(pts) >= 2:
                for i in range(len(pts) - 1):
                    cv2.line(frame, pts[i], pts[i + 1], (0, 255, 255), 2, cv2.LINE_AA)

            for pt in pts:
                cv2.circle(frame, pt, 3, (0, 255, 255), -1)

            self.get_logger().info(
                f"[OVERLAY] drew {len(pts)} pts, "
                f"first pixel=({pts[0][0]},{pts[0][1]})",
                throttle_duration_sec=5.0
            )

        if self.has_robot:
            rwx, rwy = self.pixel_to_world(self.robot_px, self.robot_py)
            rp = self.world_to_overhead_pixel(rwx, rwy)
            cv2.circle(frame, rp, 6, (255, 0, 0), -1)

        if self.has_ball:
            bwx, bwy = self.pixel_to_world(self.ball_px, self.ball_py)
            bp = self.world_to_overhead_pixel(bwx, bwy)
            cv2.circle(frame, bp, 8, (0, 255, 255), -1)

        cv2.namedWindow("Overhead Camera", cv2.WINDOW_NORMAL)
        cv2.imshow("Overhead Camera", frame)
        cv2.waitKey(1)

    # Destroy node
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


# Main entry
def main(args=None):
    rclpy.init(args=args)
    node = VisionOverlayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()