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


# ── Image → ordered pixel path ──────────────────────────────────────────────

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

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        raise ValueError("No contours found in image — check threshold / invert")

    # Largest by perimeter is the most useful "outline" for line drawings.
    contour = max(contours, key=lambda c: cv2.arcLength(c, closed=True))
    if simplify_eps > 0:
        contour = cv2.approxPolyDP(contour, simplify_eps, closed=True)

    pts = contour.reshape(-1, 2).astype(np.float32)
    # Close the loop so the bot returns to the starting point.
    pts = np.vstack([pts, pts[0:1]])

    if preview_path:
        preview = cv2.cvtColor(work, cv2.COLOR_GRAY2BGR)
        for i in range(1, len(pts)):
            p0 = tuple(pts[i - 1].astype(int))
            p1 = tuple(pts[i].astype(int))
            cv2.line(preview, p0, p1, (0, 0, 255), 2)
        for px, py in pts.astype(int):
            cv2.circle(preview, (int(px), int(py)), 3, (0, 255, 0), -1)
        cv2.imwrite(preview_path, preview)

    h, w = img.shape
    return pts, w, h, len(contours)


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
        self.declare_parameter('invert', 'auto')
        self.declare_parameter('preview', '')
        self.declare_parameter('republish_s', 5.0)

        image = self.get_parameter('image').get_parameter_value().string_value
        max_frac = self.get_parameter('max_frac').get_parameter_value().double_value
        simplify_eps = self.get_parameter('simplify_eps').get_parameter_value().double_value
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
