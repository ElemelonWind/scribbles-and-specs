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

# Physical board dimensions (matches Scribbles main.cpp).
BOARD_WIDTH_IN = 45.0
BOARD_HEIGHT_IN = 29.0

# Eraser sits 4.5 inches "above" the AprilTag (in the tag's up direction).
# Assuming the bot faces up the board (heading ≈ 0°) when erasing, the tag
# must be that far below the ink mark for the eraser to land on it. Shift
# each waypoint by +ERASER_OFFSET_Y in normalized board coords (y = down).
ERASER_OFFSET_IN = 4.5
ERASER_OFFSET_Y = ERASER_OFFSET_IN / BOARD_HEIGHT_IN  # ≈ 0.155

# Ignore any ink detected within this many inches of a board corner (the
# corner AprilTags themselves and the surrounding mounting hardware aren't
# real ink and shouldn't be erased).
CORNER_EXCLUSION_IN = 15

# Ignore any ink detected within this many inches of the bot's AprilTag
# (the bot's chassis, the eraser arm above the tag, and any shadows it
# casts often look like ink to the LAB-deviation detector).
BOT_EXCLUSION_IN = 8.0


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
    """Snap each occupied grid cell to its center point.

    Using cell centers (instead of the first centroid in a cell) makes the
    waypoint list deterministic across frames: as long as the same set of
    cells contains ink, the published list is identical and downstream
    consumers (specs_comms) won't reset their progress on every frame.
    """
    cells_hit = set()
    for cx, cy in centroids:
        col = min(GRID - 1, int((cx / board_size) * GRID))
        row = min(GRID - 1, int((cy / board_size) * GRID))
        cells_hit.add((row, col))
    waypoints = []
    for row, col in sorted(cells_hit):
        xn = (col + 0.5) / GRID
        yn = (row + 0.5) / GRID
        waypoints.append((xn, yn))
    return waypoints


def create_corner_exclusion_mask(size=BOARD_SIZE,
                                 exclusion_in=CORNER_EXCLUSION_IN,
                                 board_w_in=BOARD_WIDTH_IN,
                                 board_h_in=BOARD_HEIGHT_IN):
    """Mask out a `exclusion_in`-inch radius circle around each board corner.

    The warp stretches the board into a square, so a physical circle becomes
    an ellipse in warped space. The semi-axes are scaled accordingly.
    """
    mask = np.zeros((size, size), dtype=np.uint8)
    rx = max(1, int(round(exclusion_in * size / board_w_in)))
    ry = max(1, int(round(exclusion_in * size / board_h_in)))
    corners = [(0, 0), (size - 1, 0), (size - 1, size - 1), (0, size - 1)]
    for cx, cy in corners:
        cv2.ellipse(mask, (cx, cy), (rx, ry), 0, 0, 360, 255, -1)
    return mask


def generate_sweep_waypoints(num_rows=6, x_margin_in=2.0, y_margin_in=1.0):
    """Build a serpentine sweep over the eraser-reachable area.

    The eraser sits 4.5" above the AprilTag, so the eraser can only reach
    y ∈ [0, BOARD_HEIGHT - 4.5]. Tag waypoints are emitted directly (no
    eraser offset added downstream — these are TAG targets).
    """
    er_y_min = y_margin_in
    er_y_max = BOARD_HEIGHT_IN - ERASER_OFFSET_IN - y_margin_in
    x_min = x_margin_in
    x_max = BOARD_WIDTH_IN - x_margin_in

    waypoints = []
    for i in range(num_rows):
        if num_rows == 1:
            er_y = (er_y_min + er_y_max) / 2.0
        else:
            er_y = er_y_min + i * (er_y_max - er_y_min) / (num_rows - 1)
        tag_y = er_y + ERASER_OFFSET_IN
        xs = [x_min, x_max] if i % 2 == 0 else [x_max, x_min]
        for x in xs:
            waypoints.append((x / BOARD_WIDTH_IN, tag_y / BOARD_HEIGHT_IN))
    return waypoints


def add_bot_exclusion(mask, bot_pose, size=BOARD_SIZE,
                       exclusion_in=BOT_EXCLUSION_IN,
                       board_w_in=BOARD_WIDTH_IN,
                       board_h_in=BOARD_HEIGHT_IN):
    """Burn an `exclusion_in`-inch ellipse around the bot's tag into `mask`.

    `bot_pose` is `(x_norm, y_norm)` in the user-facing convention
    (x = across the board, y = down the board). localization swaps the
    homography components when publishing — `x_norm = dst[1]`,
    `y_norm = dst[0]` — so to map back into the warp's pixel space:
        warped_col = y_norm * size
        warped_row = x_norm * size
    """
    if bot_pose is None:
        return mask
    bx, by = bot_pose
    bot_col = int(round(by * size))
    bot_row = int(round(bx * size))
    if not (0 <= bot_col < size and 0 <= bot_row < size):
        return mask
    rx = max(1, int(round(exclusion_in * size / board_w_in)))
    ry = max(1, int(round(exclusion_in * size / board_h_in)))
    cv2.ellipse(mask, (bot_col, bot_row), (rx, ry), 0, 0, 360, 255, -1)
    return mask


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
    STATE_SWEEP_1 = 'sweep_1'
    STATE_DETECT  = 'detect'
    STATE_SWEEP_2 = 'sweep_2'
    STATE_DONE    = 'done'

    def __init__(self):
        super().__init__('ink_detection')
        self.bridge = CvBridge()
        self.board_corners = None
        self.bot_pose = None      # (x_norm, y_norm) in user-facing convention
        self.tag_detector = create_apriltag_detector()
        # Static corner-exclusion mask in warped board space (cached once).
        self.corner_mask = create_corner_exclusion_mask()

        # ── Tunables ──────────────────────────────────────────────────────
        self.declare_parameter('sweep_duration_s', 60.0)
        self.declare_parameter('detect_duration_s', 30.0)
        self.declare_parameter('sweep_rows', 6)
        self.sweep_duration_s = self.get_parameter('sweep_duration_s') \
            .get_parameter_value().double_value
        self.detect_duration_s = self.get_parameter('detect_duration_s') \
            .get_parameter_value().double_value
        sweep_rows = self.get_parameter('sweep_rows') \
            .get_parameter_value().integer_value
        self.sweep_waypoints = generate_sweep_waypoints(num_rows=sweep_rows)

        # ── ROS comms ─────────────────────────────────────────────────────
        self.waypoints_pub = self.create_publisher(PoseArray, '/specs/ink_waypoints', 10)
        self.board_corners_sub = self.create_subscription(
            Float32MultiArray, '/specs/board_corners',
            self.board_corners_callback, 10)
        self.bot_pose_sub = self.create_subscription(
            Float32MultiArray, '/specs/bot_pose',
            self.bot_pose_callback, 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw',
            self.image_callback, 10)

        # ── State machine ─────────────────────────────────────────────────
        self.state = self.STATE_SWEEP_1
        self.state_start = self.get_clock().now()
        self._publish_waypoints(self.sweep_waypoints, apply_eraser_offset=False)
        self.get_logger().info(
            f"State → {self.state}: {len(self.sweep_waypoints)} sweep waypoints "
            f"(duration={self.sweep_duration_s}s, then {self.detect_duration_s}s detect)"
        )
        # Tick at 1 Hz to advance state; re-broadcasts sweep on the same tick
        # for robustness in case the first publish was missed.
        self.state_timer = self.create_timer(1.0, self._state_tick)

    def board_corners_callback(self, msg):
        if len(msg.data) == 8:
            self.board_corners = np.array(msg.data, dtype=np.float32).reshape(4, 2)

    def bot_pose_callback(self, msg):
        if len(msg.data) >= 2:
            self.bot_pose = (float(msg.data[0]), float(msg.data[1]))

    # ── State machine helpers ────────────────────────────────────────────

    def _elapsed_in_state(self):
        return (self.get_clock().now() - self.state_start).nanoseconds / 1e9

    def _enter_state(self, new_state):
        self.get_logger().info(f"State {self.state} → {new_state}")
        self.state = new_state
        self.state_start = self.get_clock().now()
        if new_state == self.STATE_SWEEP_2:
            self._publish_waypoints(self.sweep_waypoints, apply_eraser_offset=False)
        elif new_state == self.STATE_DONE:
            self._publish_waypoints([], apply_eraser_offset=False)

    def _state_tick(self):
        elapsed = self._elapsed_in_state()
        if self.state == self.STATE_SWEEP_1 and elapsed >= self.sweep_duration_s:
            self._enter_state(self.STATE_DETECT)
        elif self.state == self.STATE_DETECT and elapsed >= self.detect_duration_s:
            self._enter_state(self.STATE_SWEEP_2)
        elif self.state == self.STATE_SWEEP_2 and elapsed >= self.sweep_duration_s:
            self._enter_state(self.STATE_DONE)

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

        # Detect ink and exclude: detected apriltag hulls, corner zones,
        # and the bot's chassis around its current tag pose.
        ink_mask = detect_ink(warped)
        if np.any(tag_mask):
            ink_mask = cv2.bitwise_and(ink_mask, cv2.bitwise_not(tag_mask))
        ink_mask = cv2.bitwise_and(ink_mask, cv2.bitwise_not(self.corner_mask))
        if self.bot_pose is not None:
            bot_mask = np.zeros((BOARD_SIZE, BOARD_SIZE), dtype=np.uint8)
            add_bot_exclusion(bot_mask, self.bot_pose)
            ink_mask = cv2.bitwise_and(ink_mask, cv2.bitwise_not(bot_mask))

        centroids = find_centroids(ink_mask)
        waypoints = cluster_to_waypoints(centroids)

        self.get_logger().info(
            f'state={self.state} marks={len(centroids)} '
            f'waypoints={len(waypoints)}'
        )

        # Only the DETECT stage drives the bot from the ink-detector output.
        # SWEEP stages publish their hardcoded waypoints from the state
        # machine; ink detection still runs there for visualization only.
        if self.state == self.STATE_DETECT:
            self._publish_waypoints(waypoints, apply_eraser_offset=True)

        self.visualize(frame, centroids, M)

    def _publish_waypoints(self, waypoints, apply_eraser_offset):
        """Publish a list of (x_norm, y_norm) waypoints.

        If `apply_eraser_offset` is True, each waypoint's y is shifted by
        +ERASER_OFFSET_Y so the eraser (mounted 4.5" above the tag) lands
        on the original point. Use False when the waypoints are already
        TAG-frame targets (e.g. the hardcoded sweep pattern).
        """
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'camera'
        for x_norm, y_norm in waypoints:
            y_out = y_norm + ERASER_OFFSET_Y if apply_eraser_offset else y_norm
            y_out = max(0.0, min(1.0, y_out))
            pose = Pose()
            pose.position.x = float(x_norm)
            pose.position.y = float(y_out)
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

        elapsed = self._elapsed_in_state()
        if self.state in (self.STATE_SWEEP_1, self.STATE_SWEEP_2):
            total = self.sweep_duration_s
        elif self.state == self.STATE_DETECT:
            total = self.detect_duration_s
        else:
            total = 0.0
        state_line = f"state={self.state}  t={elapsed:.1f}/{total:.0f}s"
        cv2.putText(overlay, state_line, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

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
