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
BOARD_SIZE = 800
GRID = 10

# Eraser sits 4.5 inches "above" the AprilTag (in the tag's up direction).
# Assuming the bot faces up the board (heading ≈ 0°) when erasing, the tag
# must be that far below the ink mark for the eraser to land on it. Shift
# each waypoint by +ERASER_OFFSET_Y in normalized board coords (y = down).
BOARD_HEIGHT_IN = 29.0
ERASER_OFFSET_IN = 4.5
ERASER_OFFSET_Y = ERASER_OFFSET_IN / BOARD_HEIGHT_IN  # ≈ 0.155


# ── Ink detection (LAB background subtraction) ──────────────────────────────

def warp_board(frame, corners, size=BOARD_SIZE):
    dst = np.array([[0, 0], [size, 0], [size, size], [0, size]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(corners, dst)
    warped = cv2.warpPerspective(frame, M, (size, size))
    return warped, M


def detect_ink(warped):
    """
    Detect ink via LAB background subtraction.
    Heavy blur estimates the local background per-channel; anything that
    deviates is ink.  Handles any marker colour and cancels smooth glare.
    """
    lab = cv2.cvtColor(warped, cv2.COLOR_BGR2LAB).astype(np.float32)
    ksize = (151, 151)

    masks = []
    for ch in range(3):
        channel = lab[:, :, ch]
        bg = cv2.GaussianBlur(channel, ksize, 0)
        diff = np.abs(channel - bg)
        thresh = 7 if ch == 0 else 4
        mask = (diff > thresh).astype(np.uint8) * 255
        masks.append(mask)

    combined = cv2.bitwise_or(masks[0], cv2.bitwise_or(masks[1], masks[2]))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel, iterations=1)
    combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel, iterations=1)
    return combined


def find_centroids(mask, min_area=60):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centroids = []
    for c in contours:
        if cv2.contourArea(c) < min_area:
            continue
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        centroids.append((int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])))
    return centroids


def unwarp_centroids(centroids, M):
    """Map warped-space centroids back to original image coordinates."""
    if not centroids:
        return []
    M_inv = np.linalg.inv(M)
    pts = np.array(centroids, dtype=np.float32).reshape(1, -1, 2)
    orig = cv2.perspectiveTransform(pts, M_inv)[0]
    return [(int(x), int(y)) for x, y in orig]


def cluster_to_waypoints(centroids, board_size=BOARD_SIZE):
    grid_map = {}
    for cx, cy in centroids:
        xn = cx / board_size
        yn = cy / board_size
        key = (min(GRID - 1, int(yn * GRID)), min(GRID - 1, int(xn * GRID)))
        if key not in grid_map:
            grid_map[key] = (xn, yn)
    return [grid_map[k] for k in sorted(grid_map)]


# ── Apriltag helpers ─────────────────────────────────────────────────────────

def create_apriltag_detector():
    if hasattr(apriltag, 'Detector'):
        return apriltag.Detector()
    if hasattr(apriltag, 'apriltag'):
        return apriltag.apriltag('tag36h11')
    if callable(apriltag):
        return apriltag()
    return None


def create_apriltag_exclusion_mask(size, detections, corners, padding=16):
    """Build a mask of apriltag regions in warped-board space."""
    if not detections:
        return np.zeros((size, size), dtype=np.uint8)

    dst = np.array([[0, 0], [size, 0], [size, size], [0, size]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(corners, dst)

    mask = np.zeros((size, size), dtype=np.uint8)
    for d in detections:
        if hasattr(d, 'corners'):
            pts = d.corners.astype(np.float32)
        elif 'lb-rb-rt-lt' in d:
            pts = np.array(d['lb-rb-rt-lt'], dtype=np.float32).reshape(-1, 2)
        else:
            continue
        # Warp tag corners into board space
        warped_pts = cv2.perspectiveTransform(pts.reshape(1, -1, 2), M)[0]
        hull = cv2.convexHull(np.int32(warped_pts))
        cv2.fillPoly(mask, [hull], 255)
        for pt in hull.reshape(-1, 2):
            cv2.circle(mask, (int(pt[0]), int(pt[1])), padding, 255, -1)
    return mask


# ── ROS node ─────────────────────────────────────────────────────────────────

class InkDetectionNode(Node):
    def __init__(self):
        super().__init__('ink_detection')
        self.bridge = CvBridge()
        self.board_corners = None
        self.tag_detector = create_apriltag_detector()

        self.waypoints_pub = self.create_publisher(PoseArray, '/specs/ink_waypoints', 10)
        self.board_corners_sub = self.create_subscription(
            Float32MultiArray, '/specs/board_corners',
            self.board_corners_callback, 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw',
            self.image_callback, 10)

    def board_corners_callback(self, msg):
        if len(msg.data) == 8:
            self.board_corners = np.array(msg.data, dtype=np.float32).reshape(4, 2)

    def image_callback(self, msg):
        if self.board_corners is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge failed: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Warp board to flat square
        warped, M = warp_board(frame, self.board_corners)

        # Build apriltag exclusion mask in warped space
        detections = self.tag_detector.detect(gray) if self.tag_detector else []
        tag_mask = create_apriltag_exclusion_mask(
            BOARD_SIZE, detections, self.board_corners)

        # Detect ink and exclude apriltag regions
        ink_mask = detect_ink(warped)
        if np.any(tag_mask):
            ink_mask = cv2.bitwise_and(ink_mask, cv2.bitwise_not(tag_mask))

        centroids = find_centroids(ink_mask)
        waypoints = cluster_to_waypoints(centroids)

        self.get_logger().info(
            f'marks={len(centroids)} waypoints={len(waypoints)}')

        self.publish_waypoints(waypoints)
        self.visualize(frame, centroids, M)

    def publish_waypoints(self, waypoints):
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'camera'
        for x_norm, y_norm in waypoints:
            # Shift the waypoint so the eraser (4.5" above the tag) lands on
            # the ink mark. Assumes the bot drives with heading ≈ 0° (facing
            # up the board) — see ERASER_OFFSET_Y comment.
            y_shifted = max(0.0, min(1.0, y_norm + ERASER_OFFSET_Y))
            pose = Pose()
            pose.position.x = x_norm
            pose.position.y = y_shifted
            pose.position.z = 0.0
            pose_array.poses.append(pose)
        self.waypoints_pub.publish(pose_array)

    def visualize(self, frame, centroids, M):
        overlay = frame.copy()
        orig_pts = unwarp_centroids(centroids, M)
        for cx, cy in orig_pts:
            cv2.circle(overlay, (cx, cy), 5, (0, 0, 255), -1)
        if self.board_corners is not None:
            cv2.polylines(overlay, [np.int32(self.board_corners)], True, (0, 255, 255), 2)
        info = f"marks={len(centroids)}"
        cv2.putText(overlay, info, (10, frame.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow(WINDOW_NAME, overlay)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
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
