#!/usr/bin/env python3
"""
Convert an input image into a single-stroke waypoint path and publish it on
`/specs/ink_waypoints` (drop-in replacement for `ink_detection.py` /
`draw_waypoints.py`).

Because the marker is always touching the board (no pen-lift), only ONE
continuous contour is traced. The largest external contour is selected,
simplified with approxPolyDP, and mapped into normalized board coordinates
[0,1] x [0,1] while preserving the image's aspect ratio against the physical
board size (default 45in x 29in — wider than tall, so a square input is not
stretched).

Parameters:
  image         path to the input image (required)
  max_frac      fraction of the board to fill (default 0.8)
  simplify_eps  approxPolyDP epsilon, in input-image pixels (default 2.0)
  invert        one of {auto, yes, no} — whether to invert before threshold
  preview       optional path to write an annotated preview PNG
  republish_s   how often to re-broadcast the waypoint list (default 5.0)
"""
import math
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose


BOARD_W_IN = 45.0
BOARD_H_IN = 29.0


# ── Image → ordered pixel path (skeleton-walked from an endpoint) ──────────

def _skeletonize(binary):
    """Reduce binary (uint8 0/255) to a 1-pixel-wide centerline."""
    if hasattr(cv2, 'ximgproc') and hasattr(cv2.ximgproc, 'thinning'):
        return cv2.ximgproc.thinning(binary, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
    # Fallback: Lantuéjoul morphological skeleton (no opencv-contrib needed).
    skel = np.zeros_like(binary)
    img = binary.copy()
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    while cv2.countNonZero(img) > 0:
        eroded = cv2.erode(img, kernel)
        opened = cv2.dilate(eroded, kernel)
        skel = cv2.bitwise_or(skel, cv2.subtract(img, opened))
        img = eroded
    return skel


def _largest_component(skel):
    n, labels = cv2.connectedComponents(skel)
    if n <= 2:
        return skel, max(0, n - 1)
    sizes = [int((labels == i).sum()) for i in range(1, n)]
    keep = 1 + int(np.argmax(sizes))
    out = ((labels == keep).astype(np.uint8)) * 255
    return out, n - 1


def _find_endpoints(skel):
    """Endpoints: skeleton pixels with exactly 1 of 8 neighbors set."""
    skel_bin = (skel > 0).astype(np.uint8)
    kernel = np.array([[1, 1, 1],
                        [1, 0, 1],
                        [1, 1, 1]], dtype=np.uint8)
    nbrs = cv2.filter2D(skel_bin, -1, kernel)
    return np.argwhere((skel_bin > 0) & (nbrs == 1))


def _walk_skeleton(skel, start_rc):
    """Walk the skeleton from start_rc (row, col); returns ordered (x, y)."""
    work = skel.copy()
    h, w = work.shape
    r, c = int(start_rc[0]), int(start_rc[1])
    path = [(c, r)]
    work[r, c] = 0
    # 4-connected neighbors first so straight-line segments stay smooth.
    deltas = ((-1, 0), (1, 0), (0, -1), (0, 1),
              (-1, -1), (-1, 1), (1, -1), (1, 1))
    while True:
        nxt = None
        for dr, dc in deltas:
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w and work[nr, nc] > 0:
                nxt = (nr, nc)
                break
        if nxt is None:
            break
        r, c = nxt
        path.append((c, r))
        work[r, c] = 0
    return path


def extract_path(image_path, simplify_eps=2.0, invert='auto', preview_path=''):
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Could not read image: {image_path}")

    if invert == 'auto':
        do_invert = np.mean(img) > 127  # mostly light → drawing is dark, invert
    else:
        do_invert = (invert == 'yes')
    work = (255 - img) if do_invert else img

    _, binary = cv2.threshold(work, 50, 255, cv2.THRESH_BINARY)

    skel = _skeletonize(binary)
    if cv2.countNonZero(skel) == 0:
        raise ValueError("Empty skeleton — check threshold / invert")

    # Drop any spurious smaller strokes; assume a single line per the user's
    # contract, but pick the largest connected component just in case.
    skel, n_strokes = _largest_component(skel)

    endpoints = _find_endpoints(skel)
    if len(endpoints) >= 1:
        # Topmost-leftmost endpoint as the deterministic start.
        endpoints = endpoints[np.lexsort((endpoints[:, 1], endpoints[:, 0]))]
        start_rc = endpoints[0]
    else:
        # Closed loop with no real endpoint: just pick any pixel.
        start_rc = np.argwhere(skel > 0)[0]

    path = _walk_skeleton(skel, start_rc)
    pts = np.array(path, dtype=np.float32)

    # Simplify the OPEN polyline (do not close the loop — it's a line).
    if simplify_eps > 0 and len(pts) > 2:
        simplified = cv2.approxPolyDP(pts.reshape(-1, 1, 2),
                                       simplify_eps, closed=False)
        pts = simplified.reshape(-1, 2).astype(np.float32)

    if preview_path:
        preview = cv2.cvtColor(work, cv2.COLOR_GRAY2BGR)
        for i in range(1, len(pts)):
            p0 = tuple(pts[i - 1].astype(int))
            p1 = tuple(pts[i].astype(int))
            cv2.line(preview, p0, p1, (0, 0, 255), 2)
        for px, py in pts.astype(int):
            cv2.circle(preview, (int(px), int(py)), 3, (0, 255, 0), -1)
        if len(pts):
            sp = pts[0].astype(int)
            cv2.circle(preview, (int(sp[0]), int(sp[1])), 9, (255, 0, 0), 2)
            cv2.putText(preview, "START", (int(sp[0]) + 11, int(sp[1]) - 11),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        cv2.imwrite(preview_path, preview)

    h, w = img.shape
    return pts, w, h, n_strokes


# ── Resample a polyline to uniform arc-length spacing ──────────────────────

def resample_uniform(pts, spacing_norm):
    """Resample a closed polyline (in normalized [0,1] coords) so that
    consecutive points are roughly `spacing_norm` apart in normalized units.
    Returns at least 2 points; preserves the start point and closing edge.
    """
    if spacing_norm <= 0 or len(pts) < 2:
        return pts
    diffs = np.diff(pts, axis=0)
    seg_lens = np.linalg.norm(diffs, axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg_lens)])
    total = cum[-1]
    if total == 0:
        return pts
    n = max(2, int(round(total / spacing_norm)))
    new_dists = np.linspace(0.0, total, n)
    new_pts = np.empty((n, 2), dtype=pts.dtype)
    for i in range(2):
        new_pts[:, i] = np.interp(new_dists, cum, pts[:, i])
    return new_pts


# ── Pixel coords → normalized board coords (aspect-preserving) ──────────────

def pixels_to_normalized(pts_px, img_w, img_h, max_frac=0.8):
    img_aspect = img_w / img_h
    board_aspect = BOARD_W_IN / BOARD_H_IN  # ≈ 1.55

    # Fit the image into the largest box (inside max_frac of the board) that
    # has the image's own aspect ratio when measured in real-world inches.
    if img_aspect >= board_aspect:
        physical_w_in = max_frac * BOARD_W_IN
        physical_h_in = physical_w_in * (img_h / img_w)
    else:
        physical_h_in = max_frac * BOARD_H_IN
        physical_w_in = physical_h_in * (img_w / img_h)

    norm_w = physical_w_in / BOARD_W_IN
    norm_h = physical_h_in / BOARD_H_IN
    offset_x = (1.0 - norm_w) / 2.0
    offset_y = (1.0 - norm_h) / 2.0

    out = np.zeros_like(pts_px)
    out[:, 0] = offset_x + (pts_px[:, 0] / img_w) * norm_w
    out[:, 1] = offset_y + (pts_px[:, 1] / img_h) * norm_h
    return out, (norm_w, norm_h, physical_w_in, physical_h_in)


# ── ROS node ────────────────────────────────────────────────────────────────

class ImageWaypointsNode(Node):
    def __init__(self):
        super().__init__('image_waypoints')

        self.declare_parameter('image', '')
        self.declare_parameter('max_frac', 0.8)
        self.declare_parameter('simplify_eps', 2.0)
        self.declare_parameter('resample_spacing', 0.05)   # normalized; 0 disables
        self.declare_parameter('invert', 'auto')
        self.declare_parameter('preview', '')
        self.declare_parameter('republish_s', 5.0)

        image = self.get_parameter('image').get_parameter_value().string_value
        max_frac = self.get_parameter('max_frac').get_parameter_value().double_value
        simplify_eps = self.get_parameter('simplify_eps').get_parameter_value().double_value
        resample_spacing = self.get_parameter('resample_spacing').get_parameter_value().double_value
        invert = self.get_parameter('invert').get_parameter_value().string_value
        preview = self.get_parameter('preview').get_parameter_value().string_value
        republish_s = self.get_parameter('republish_s').get_parameter_value().double_value

        if not image:
            raise ValueError(
                "Required parameter 'image' is empty. "
                "Pass it with: --ros-args -p image:=/path/to/picture.png"
            )

        pts_px, img_w, img_h, n_contours = extract_path(
            image, simplify_eps=simplify_eps, invert=invert, preview_path=preview,
        )
        norm_pts, (nw, nh, pw, ph) = pixels_to_normalized(
            pts_px, img_w, img_h, max_frac=max_frac
        )
        # Resample to (roughly) uniform spacing along the path so the bot
        # gets evenly-spaced targets instead of clumps from approxPolyDP.
        if resample_spacing > 0:
            norm_pts = resample_uniform(norm_pts, resample_spacing)
        # Clamp into [0,1] just in case max_frac > 1.
        norm_pts = np.clip(norm_pts, 0.0, 1.0)
        self.waypoints = [(float(x), float(y)) for x, y in norm_pts]

        if n_contours > 1:
            self.get_logger().warning(
                f"Image has {n_contours} contours — only the largest is traced "
                f"(others are ignored because the marker can't lift)."
            )

        self.get_logger().info(
            f"image_waypoints: image={image} "
            f"({img_w}x{img_h}px → {pw:.1f}\"x{ph:.1f}\" = "
            f"{nw:.3f}x{nh:.3f} normalized) "
            f"→ {len(self.waypoints)} waypoints, republishing every {republish_s}s"
        )
        if preview:
            self.get_logger().info(f"Preview written to {preview}")

        self.pub = self.create_publisher(PoseArray, '/specs/ink_waypoints', 10)
        self.timer = self.create_timer(republish_s, self.publish_waypoints)
        self.publish_waypoints()

    def publish_waypoints(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'board'
        for x, y in self.waypoints:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = 0.0
            msg.poses.append(p)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImageWaypointsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
