#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
import apriltag

WINDOW_NAME = "Specs Ink Detection"
GRID_ROWS = 10
GRID_COLS = 10


def create_board_mask(shape, corners):
    mask = np.zeros(shape, dtype=np.uint8)
    cv2.fillPoly(mask, [np.int32(corners)], 255)
    return mask


def create_apriltag_detector():
    if apriltag is not None:
        if hasattr(apriltag, 'Detector'):
            return apriltag.Detector()
        if hasattr(apriltag, 'apriltag'):
            return apriltag.apriltag('tag36h11')
        if callable(apriltag):
            return apriltag()
    return None


def create_apriltag_exclusion_mask(shape, detections, padding=12):
    mask = np.zeros(shape, dtype=np.uint8)
    if detections is None:
        return mask

    for d in detections:
        if 'lb-rb-rt-lt' in d:
            poly = np.int32(d['lb-rb-rt-lt']).reshape(-1, 2)
            hull = cv2.convexHull(poly)
            cv2.fillPoly(mask, [hull], 255)
            for (cx, cy) in hull.reshape(-1, 2):
                cv2.circle(mask, (int(cx), int(cy)), padding, 255, -1)

    return mask


def extract_ink_mask(gray, board_mask, apriltag_mask=None, mode='auto'):
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)

    if enhanced.dtype != np.uint8:
        enhanced = cv2.normalize(enhanced, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    illum = cv2.GaussianBlur(enhanced, (61, 61), 0)
    local = cv2.addWeighted(enhanced, 1.5, illum, -0.5, 0)

    blurred = cv2.GaussianBlur(local, (5, 5), 0)

    if mode == 'auto':
        block_size = 31
        c = 2
    elif mode == 'low':
        block_size = 51
        c = -5
    else:
        block_size = 31
        c = 5

    ink_mask = cv2.adaptiveThreshold(
        blurred,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        block_size,
        c,
    )

    ink_mask = cv2.bitwise_and(ink_mask, board_mask)
    if apriltag_mask is not None:
        ink_mask = cv2.bitwise_and(ink_mask, cv2.bitwise_not(apriltag_mask))

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    ink_mask = cv2.morphologyEx(ink_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    ink_mask = cv2.morphologyEx(ink_mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    return ink_mask


def extract_ink_mask_hsv(frame, board_mask, apriltag_mask=None):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    v = hsv[:, :, 2]
    s = hsv[:, :, 1]

    dark_ink = cv2.inRange(v, 0, 150)
    saturation_range = cv2.inRange(s, 0, 255)
    ink_mask = cv2.bitwise_and(dark_ink, saturation_range)

    ink_mask = cv2.GaussianBlur(ink_mask, (3, 3), 0)
    _, ink_mask = cv2.threshold(ink_mask, 10, 255, cv2.THRESH_BINARY)

    ink_mask = cv2.bitwise_and(ink_mask, board_mask)
    if apriltag_mask is not None:
        ink_mask = cv2.bitwise_and(ink_mask, cv2.bitwise_not(apriltag_mask))

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    ink_mask = cv2.morphologyEx(ink_mask, cv2.MORPH_OPEN, kernel, iterations=1)

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
        self.tag_detector = create_apriltag_detector()

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
            print(f"Failed to convert image: {exc}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        board_mask = create_board_mask(gray.shape, self.board_corners)

        detections = None
        apriltag_mask = np.zeros(gray.shape, dtype=np.uint8)
        if self.tag_detector is not None:
            detections = self.tag_detector.detect(gray)
            apriltag_mask = create_apriltag_exclusion_mask(gray.shape, detections, padding=16)

        print(f'Apriltag detections: {len(detections) if detections is not None else 0}')
        print(f'Board mask pixels: {np.count_nonzero(board_mask)} Apriltag mask pixels: {np.count_nonzero(apriltag_mask)}')

        board_region = gray[board_mask > 0]
        if len(board_region) > 0:
            print(f'Board grayscale range: min={board_region.min()} max={board_region.max()} mean={board_region.mean():.1f}')
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        board_hsv = hsv[board_mask > 0]
        if len(board_hsv) > 0:
            v_vals = board_hsv[:, 2]
            s_vals = board_hsv[:, 1]
            print(f'Board HSV V range: min={v_vals.min()} max={v_vals.max()} mean={v_vals.mean():.1f}')
            print(f'Board HSV S range: min={s_vals.min()} max={s_vals.max()} mean={s_vals.mean():.1f}')

        ink_mask = extract_ink_mask(gray, board_mask, apriltag_mask, mode='auto')
        centroids = find_contour_centroids(ink_mask, min_area=15)
        print(f'Auto ink contours: {len(centroids)}, ink mask pixels: {np.count_nonzero(ink_mask)}')

        if len(centroids) == 0:
            print('No ink contours found: retry low threshold')
            ink_mask = extract_ink_mask(gray, board_mask, apriltag_mask, mode='low')
            centroids = find_contour_centroids(ink_mask, min_area=10)
            print(f'Low ink contours: {len(centroids)}, ink mask pixels: {np.count_nonzero(ink_mask)}')

        if len(centroids) == 0:
            print('No ink contours found: fallback HSV ink filter')
            ink_mask = extract_ink_mask_hsv(frame, board_mask, apriltag_mask)
            centroids = find_contour_centroids(ink_mask, min_area=10)
            print(f'HSV ink contours: {len(centroids)}, ink mask pixels: {np.count_nonzero(ink_mask)}')

        print(f'Ink detections: {len(centroids)}')
        normalized_points = []
        for cx, cy in centroids:
            pixel_point = np.array([[[float(cx), float(cy)]]], dtype=np.float32)
            projected = cv2.perspectiveTransform(pixel_point, self.board_homography)[0][0]
            if 0.0 <= projected[0] <= 1.0 and 0.0 <= projected[1] <= 1.0:
                normalized_points.append((float(projected[0]), float(projected[1])))

        waypoints = cluster_waypoints(normalized_points)
        print(f'Waypoints computed: {len(waypoints)} (centroids {len(centroids)}, normalized {len(normalized_points)})')
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

        print(f"Visualize: frame shape={frame.shape}, overlay shape={overlay.shape}, centroids={len(centroids)}, waypoints={len(waypoints)}")
        if overlay is None or overlay.size == 0:
            print("Visualize overlay is empty or invalid")
        else:
            try:
                cv2.imshow(WINDOW_NAME, overlay)
            except Exception as e:
                print(f"cv2.imshow failed: {e}")

        key = cv2.waitKey(1)
        if key == -1:
            print("waitKey returned -1")
        if key & 0xFF == ord('q'):
            print("Quit key pressed")
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
