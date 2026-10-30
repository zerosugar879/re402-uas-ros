import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class GazeboMonitor(Node):

    # Initialize node
    def __init__(self):
        super().__init__("gazebo_monitor")

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0

        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Twist, "/cmd_vel_raw", self.cmd_vel_callback, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_timer(0.1, self.log_status)

        self.get_logger().info("GazeboMonitor started (/odom + /cmd_vel_raw /cmd_vel)")

    # Odom callback
    def odom_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y

    # CmdVel callback
    def cmd_vel_callback(self, msg):
        self.cmd_vx = msg.linear.x
        self.cmd_wz = msg.angular.z
        self.pub_cmd_vel.publish(msg)

    # Log status
    def log_status(self):
        self.get_logger().info(
            f"[GZ] Pose=({self.pose_x:.3f},{self.pose_y:.3f}) "
            f"CmdVel=({self.cmd_vx:.2f},{self.cmd_wz:.2f})",
            throttle_duration_sec=0.5,
        )


# Main entry
def main(args=None):
    rclpy.init(args=args)
    node = GazeboMonitor()
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