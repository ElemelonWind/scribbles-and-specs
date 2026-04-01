#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2
import numpy as np

WINDOW_NAME = "Specs Ink Detection"
GRID_ROWS = 10
GRID_COLS = 10


def create_board_mask(shape, corners):
    mask = np.zeros(shape, dtype=np.uint8)
    cv2.fillPoly(mask, [np.int32(corners)], 255)
    return mask


def extract_ink_mask(gray, board_mask):
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, ink_mask = cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY_INV)
    ink_mask = cv2.bitwise_and(ink_mask, board_mask)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    ink_mask = cv2.morphologyEx(ink_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    ink_mask = cv2.morphologyEx(ink_mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    return ink_mask


def find_contour_centroids(ink_mask, min_area=80):
    contours, _ = cv2.findContours(ink_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centroids = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        M = cv2.moments(contour)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        centroids.append((cx, cy))
    return centroids


def cluster_waypoints(normalized_points):
    grid_map = {}
    for x_norm, y_norm in normalized_points:
        if x_norm < 0.0 or x_norm > 1.0 or y_norm < 0.0 or y_norm > 1.0:
            continue
        col = min(GRID_COLS - 1, int(x_norm * GRID_COLS))
        row = min(GRID_ROWS - 1, int(y_norm * GRID_ROWS))
        key = (row, col)
        if key not in grid_map:
            grid_map[key] = (x_norm, y_norm)
    sorted_keys = sorted(grid_map.keys())
    return [grid_map[k] for k in sorted_keys]


class InkDetectionNode(Node):
    def __init__(self):
        super().__init__('ink_detection')
        self.bridge = CvBridge()
        self.board_homography = None
        self.board_corners = None

        self.waypoints_pub = self.create_publisher(PoseArray, '/specs/ink_waypoints', 10)
        self.board_corners_sub = self.create_subscription(
            Float32MultiArray,
            '/specs/board_corners',
            self.board_corners_callback,
            10
        )
        self.board_homography_sub = self.create_subscription(
            Float32MultiArray,
            '/specs/board_homography',
            self.board_homography_callback,
            10
        )
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

    def board_corners_callback(self, msg):
        if len(msg.data) == 8:
            self.board_corners = np.array(msg.data, dtype=np.float32).reshape(4, 2)

    def board_homography_callback(self, msg):
        if len(msg.data) == 9:
            self.board_homography = np.array(msg.data, dtype=np.float32).reshape(3, 3)

    def image_callback(self, msg):
        if self.board_homography is None or self.board_corners is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f"Failed to convert image: {exc}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        board_mask = create_board_mask(gray.shape, self.board_corners)
        ink_mask = extract_ink_mask(gray, board_mask)

        centroids = find_contour_centroids(ink_mask)
        normalized_points = []
        for cx, cy in centroids:
            pixel_point = np.array([[[float(cx), float(cy)]]], dtype=np.float32)
            projected = cv2.perspectiveTransform(pixel_point, self.board_homography)[0][0]
            if 0.0 <= projected[0] <= 1.0 and 0.0 <= projected[1] <= 1.0:
                normalized_points.append((float(projected[0]), float(projected[1])))

        waypoints = cluster_waypoints(normalized_points)
        self.publish_waypoints(waypoints)
        self.visualize(frame, centroids, waypoints)

    def publish_waypoints(self, waypoints):
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'camera'
        for x_norm, y_norm in waypoints:
            pose = Pose()
            pose.position.x = x_norm
            pose.position.y = y_norm
            pose.position.z = 0.0
            pose_array.poses.append(pose)
        self.waypoints_pub.publish(pose_array)

    def visualize(self, frame, centroids, waypoints):
        overlay = frame.copy()
        if self.board_corners is not None:
            cv2.polylines(overlay, [np.int32(self.board_corners)], True, (0, 255, 255), 2)

        for cx, cy in centroids:
            cv2.circle(overlay, (cx, cy), 5, (0, 0, 255), -1)

        for idx, (x_norm, y_norm) in enumerate(waypoints):
            label = f"W{idx}"
            cv2.putText(overlay, label, (10 + idx * 60, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        info = f"Ink marks={len(centroids)} waypoints={len(waypoints)}"
        cv2.putText(overlay, info, (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow(WINDOW_NAME, overlay)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = InkDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
