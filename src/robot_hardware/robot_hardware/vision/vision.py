import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray

from cv_bridge import CvBridge

from robot_hardware.utils.yaml_loader import load_node_params


class VisionNode(Node):

    # Initialize node
    def __init__(self):
        super().__init__("vision_node")
        self.bridge = CvBridge()

        self._load_config()

        self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )

        self.robot_pose_pub = self.create_publisher(
            Pose2D, "/vision/robot_pose", 10
        )
        self.ball_position_pub = self.create_publisher(
            Pose2D, "/vision/ball_position", 10
        )
        self.obstacle_pub = self.create_publisher(
            Float32MultiArray, "/vision/obstacles", 10
        )
        self.debug_pub = self.create_publisher(
            Image, "/vision/debug_image", 10
        )
        self.mask_pub = self.create_publisher(
            Image, "/ball/mask", 10
        )
        self.obs_mask_pub = self.create_publisher(
            Image, "/obstacle/mask", 10
        )
        self.create_subscription(
            Float32MultiArray,
            "/planned_path",
            self.path_callback,
            10
        )

        dict_names = [
            'DICT_APRILTAG_16h5',
            'DICT_APRILTAG_25h9',
            'DICT_APRILTAG_36h10',
            'DICT_APRILTAG_36h11',
        ]
        self.dictionaries = {}
        for name in dict_names:
            dict_id = getattr(cv2.aruco, name, None)
            if dict_id is not None:
                self.dictionaries[name] = cv2.aruco.getPredefinedDictionary(dict_id)

        self.parameters = cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detectors = {
                name: cv2.aruco.ArucoDetector(d, self.parameters)
                for name, d in self.dictionaries.items()
            }
            self.get_logger().info(f"Loaded {len(self.detectors)} AprilTag families for OpenCV 5")
        else:
            self.detectors = None
            self.get_logger().warn("Using legacy detectMarkers")

        self.planned_path = []

        cv2.namedWindow("View")

        self.get_logger().info("Vision Node Started")

    # Path callback
    def path_callback(self, msg):
        self.planned_path = []
        for i in range(0, len(msg.data), 2):
            if i + 1 < len(msg.data):
                self.planned_path.append((float(msg.data[i]), float(msg.data[i + 1])))

    # Load config
    def _load_config(self):
        p = load_node_params(self)
        if p is None:
            return

        self.theta_offset = p["theta_offset"]
        self.robot_exclude_radius = p["robot_exclude_radius"]
        self.hsv = {
            "H_Low":    p["hsv_h_low"],
            "H_High":   p["hsv_h_high"],
            "S_Low":    p["hsv_s_low"],
            "V_Low":    p["hsv_v_low"],
            "Min_Area": p["hsv_min_area"],
        }
        self.obs = {
            "H_Low":    p["obs_h_low"],
            "H_High":   p["obs_h_high"],
            "S_Low":    p["obs_s_low"],
            "V_Low":    p["obs_v_low"],
            "V_High":   p["obs_v_high"],
            "Min_Area": p["obs_min_area"],
        }

    # Image callback
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        corners, ids = None, None
        for dict_name, dictionary in self.dictionaries.items():
            if self.detectors is not None:
                det = self.detectors[dict_name]
                c, i, _ = det.detectMarkers(frame)
            else:
                c, i, _ = cv2.aruco.detectMarkers(frame, dictionary)
            if i is not None and len(i) > 0:
                corners, ids = c, i
                break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        robot_pose_msg = Pose2D()
        robot_pose_msg.x = 0.0
        robot_pose_msg.y = 0.0
        robot_pose_msg.theta = 0.0

        if ids is not None:
            ids_flat = ids.flatten()
            for markerCorner, markerID in zip(corners, ids_flat):
                pts = markerCorner.reshape((4, 2)).astype(int)
                topLeft, topRight, bottomRight, bottomLeft = pts

                cv2.line(frame, tuple(topLeft), tuple(topRight), (0, 255, 0), 2)
                cv2.line(frame, tuple(topRight), tuple(bottomRight), (0, 255, 0), 2)
                cv2.line(frame, tuple(bottomRight), tuple(bottomLeft), (0, 255, 0), 2)
                cv2.line(frame, tuple(bottomLeft), tuple(topLeft), (0, 255, 0), 2)

                cX = int(np.mean(pts[:, 0]))
                cY = int(np.mean(pts[:, 1]))
                cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

                center_x = (pts[0][0] + pts[2][0]) / 2.0
                center_y = (pts[0][1] + pts[2][1]) / 2.0
                front_x = (pts[0][0] + pts[1][0]) / 2.0
                front_y = (pts[0][1] + pts[1][1]) / 2.0
                theta = math.atan2(front_y - center_y, front_x - center_x) + self.theta_offset

                self.get_logger().info(
                    f"[TAG  ] ID={markerID} | Pos=({cX},{cY})",
                    throttle_duration_sec=1.0
                )

                robot_pose_msg.x = float(cX)
                robot_pose_msg.y = float(cY)
                robot_pose_msg.theta = theta
                break

        self.robot_pose_pub.publish(robot_pose_msg)

        h_low = self.hsv["H_Low"]
        h_high = self.hsv["H_High"]
        s_low = self.hsv["S_Low"]
        v_low = self.hsv["V_Low"]
        min_area = self.hsv["Min_Area"]

        mask = cv2.inRange(hsv, (h_low, s_low, v_low), (h_high, 255, 255))
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        ball_msg = Pose2D()
        ball_msg.x = -1.0
        ball_msg.y = -1.0
        ball_msg.theta = 0.0

        ball_x = -1
        ball_y = -1

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > min_area:
                M = cv2.moments(largest)
                if M["m00"] > 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    ball_x, ball_y = cX, cY

                    self.get_logger().info(
                        f"[BALL ] Pos=({cX},{cY}) | Area={int(area)}",
                        throttle_duration_sec=1.0
                    )

                    ball_msg.x = float(cX)
                    ball_msg.y = float(cY)
                    ball_msg.theta = float(area)

                    cv2.circle(frame, (cX, cY), 5, (0, 255, 255), -1)
                    cv2.circle(frame, (cX, cY), int(np.sqrt(area / np.pi)), (0, 255, 255), 2)
                    cv2.putText(frame, "BALL", (cX - 20, cY - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        self.ball_position_pub.publish(ball_msg)

        obs_mask = cv2.inRange(hsv, (self.obs["H_Low"], self.obs["S_Low"], self.obs["V_Low"]),
                               (self.obs["H_High"], 255, self.obs["V_High"]))
        obs_mask = cv2.erode(obs_mask, None, iterations=1)
        obs_mask = cv2.dilate(obs_mask, None, iterations=1)

        obs_mask_msg = self.bridge.cv2_to_imgmsg(obs_mask, encoding="mono8")
        obs_mask_msg.header = msg.header
        self.obs_mask_pub.publish(obs_mask_msg)

        obs_contours, _ = cv2.findContours(obs_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        obstacle_data = Float32MultiArray()
        obstacles_xy = []

        robot_x = robot_pose_msg.x
        robot_y = robot_pose_msg.y
        robot_detected = (robot_x != 0.0 or robot_y != 0.0)

        for cnt in obs_contours:
            area = cv2.contourArea(cnt)
            if area < self.obs["Min_Area"]:
                continue
            M = cv2.moments(cnt)
            if M["m00"] <= 0:
                continue
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            if ball_x >= 0 and abs(cX - ball_x) < 30 and abs(cY - ball_y) < 30:
                continue

            if robot_detected and math.hypot(cX - robot_x, cY - robot_y) < self.robot_exclude_radius:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            obstacles_xy.extend([float(cX), float(cY), float(w), float(h)])
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(frame, "OBS", (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        obstacle_data.data = obstacles_xy
        self.obstacle_pub.publish(obstacle_data)

        rx = int(robot_pose_msg.x)
        ry = int(robot_pose_msg.y)

        if robot_pose_msg.x != 0.0 or robot_pose_msg.y != 0.0:
            cv2.putText(frame, "ROBOT", (rx - 25, ry - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        if self.planned_path:
            color = (255, 255, 0) if obstacles_xy else (0, 255, 255)
            for i in range(len(self.planned_path) - 1):
                p1 = self.planned_path[i]
                p2 = self.planned_path[i + 1]
                cv2.line(frame, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), color, 2, cv2.LINE_AA)
            for px, py in self.planned_path:
                cv2.circle(frame, (int(px), int(py)), 3, color, -1)

        info = f"Robot({rx},{ry}) Ball({ball_x},{ball_y})"
        cv2.putText(frame, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)

        cv2.imshow("View", frame)
        key = cv2.waitKey(1) & 0xFF

        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)


# Main entry
def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except rclpy._rclpy_pybind11.RCLError:
            pass


if __name__ == "__main__":
    main()