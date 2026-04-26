#!/usr/bin/env python3
"""
Standalone (no-ROS) preview for `image_waypoints.py`.

Loads an image, extracts a single-stroke path the same way the ROS node would,
and displays the resulting waypoints on a board-shaped grid (in the actual
45 x 29 in aspect ratio) so you can sanity-check before sending the bot
anywhere.

Usage:
    python image_waypoints_test.py <image_path> [-o preview.png]
                                   [--max-frac 0.8] [--simplify-eps 2.0]
                                   [--invert auto|yes|no]
"""
import argparse
import sys

import cv2
import numpy as np


BOARD_W_IN = 45.0
BOARD_H_IN = 29.0
GRID_DIVISIONS = 10           # major grid lines on the preview canvas
GRID_RESOLUTION = 255         # what the ESP32 actually sees


# ── Same path extraction the ROS node uses ───────────────────────────────────

def extract_path(image_path, simplify_eps=2.0, invert='auto'):
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Could not read image: {image_path}")

    if invert == 'auto':
        do_invert = np.mean(img) > 127
    else:
        do_invert = (invert == 'yes')
    work = (255 - img) if do_invert else img

    _, binary = cv2.threshold(work, 50, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        raise ValueError("No contours found in image — check threshold / invert")

    contour = max(contours, key=lambda c: cv2.arcLength(c, closed=True))
    if simplify_eps > 0:
        contour = cv2.approxPolyDP(contour, simplify_eps, closed=True)
    pts = contour.reshape(-1, 2).astype(np.float32)
    pts = np.vstack([pts, pts[0:1]])     # close loop

    h, w = img.shape
    return pts, work, w, h, len(contours)


def pixels_to_normalized(pts_px, img_w, img_h, max_frac=0.8):
    img_aspect = img_w / img_h
    board_aspect = BOARD_W_IN / BOARD_H_IN

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


# ── Rendering ───────────────────────────────────────────────────────────────

def render_board(norm_pts, canvas_w=900):
    """Draw the board (correct aspect), grid, and waypoint path."""
    canvas_h = int(round(canvas_w * BOARD_H_IN / BOARD_W_IN))
    canvas = np.full((canvas_h, canvas_w, 3), 255, dtype=np.uint8)

    # Major grid lines
    for i in range(1, GRID_DIVISIONS):
        x = int(i * canvas_w / GRID_DIVISIONS)
        cv2.line(canvas, (x, 0), (x, canvas_h - 1), (220, 220, 220), 1)
        y = int(i * canvas_h / GRID_DIVISIONS)
        cv2.line(canvas, (0, y), (canvas_w - 1, y), (220, 220, 220), 1)

    # Board border
    cv2.rectangle(canvas, (0, 0), (canvas_w - 1, canvas_h - 1), (0, 0, 0), 2)

    pts = [(int(x * canvas_w), int(y * canvas_h)) for x, y in norm_pts]

    # Path
    for i in range(1, len(pts)):
        cv2.line(canvas, pts[i - 1], pts[i], (0, 0, 220), 2)

    # Waypoints
    for p in pts:
        cv2.circle(canvas, p, 4, (0, 180, 0), -1)

    # Direction arrows at a handful of points
    n = len(pts)
    if n >= 4:
        for i in (n // 4, n // 2, 3 * n // 4):
            if i + 1 < n:
                cv2.arrowedLine(canvas, pts[i], pts[i + 1], (200, 100, 0), 2, tipLength=0.5)

    # Start marker
    if pts:
        cv2.circle(canvas, pts[0], 9, (0, 180, 0), 2)
        cv2.putText(canvas, "START", (pts[0][0] + 11, pts[0][1] - 11),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 180, 0), 2)

    # Axis / dimension labels
    cv2.putText(canvas, f'x = across, board {BOARD_W_IN:.0f}"',
                (10, canvas_h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 90, 90), 1)
    cv2.putText(canvas, f'y = down, {BOARD_H_IN:.0f}"',
                (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 90, 90), 1)

    info = f"{len(pts)} waypoints"
    cv2.putText(canvas, info, (canvas_w - 200, canvas_h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (90, 90, 90), 1)

    return canvas


def render_source_overlay(work_gray, pts_px):
    overlay = cv2.cvtColor(work_gray, cv2.COLOR_GRAY2BGR)
    pts_int = pts_px.astype(int)
    for i in range(1, len(pts_int)):
        cv2.line(overlay, tuple(pts_int[i - 1]), tuple(pts_int[i]), (0, 0, 220), 2)
    for px, py in pts_int:
        cv2.circle(overlay, (int(px), int(py)), 3, (0, 180, 0), -1)
    return overlay


# ── Main ────────────────────────────────────────────────────────────────────

def main(argv=None):
    ap = argparse.ArgumentParser(description="Preview image waypoints on a board grid (no ROS).")
    ap.add_argument("image", help="Path to input image")
    ap.add_argument("--max-frac", type=float, default=0.8, help="Fraction of board to fill (default 0.8)")
    ap.add_argument("--simplify-eps", type=float, default=2.0, help="approxPolyDP epsilon in input pixels (default 2.0)")
    ap.add_argument("--invert", choices=['auto', 'yes', 'no'], default='auto')
    ap.add_argument("-o", "--output", default=None, help="Save board-view preview here")
    args = ap.parse_args(argv)

    pts_px, work, w, h, n_contours = extract_path(
        args.image, simplify_eps=args.simplify_eps, invert=args.invert,
    )
    norm_pts, (nw, nh, pw, ph) = pixels_to_normalized(
        pts_px, w, h, max_frac=args.max_frac,
    )
    norm_pts = np.clip(norm_pts, 0.0, 1.0)

    print(f"Image:        {w}x{h} px   (contours found: {n_contours})")
    if n_contours > 1:
        print(f"  WARNING: only the largest contour is traced; {n_contours - 1} others ignored")
    print(f"Drawn size:   {pw:.2f}\" x {ph:.2f}\"   ({nw:.3f} x {nh:.3f} normalized)")
    print(f"Waypoints:    {len(norm_pts)}")
    for i, (x, y) in enumerate(norm_pts):
        gx = int(round(x * GRID_RESOLUTION))
        gy = int(round(y * GRID_RESOLUTION))
        print(f"  W{i:3d}: norm=({x:.3f}, {y:.3f})   grid=({gx:3d}, {gy:3d})")

    board = render_board(norm_pts)
    overlay = render_source_overlay(work, pts_px)

    if args.output:
        cv2.imwrite(args.output, board)
        print(f"Saved board preview → {args.output}")

    cv2.namedWindow("Board View (45x29 in)", cv2.WINDOW_NORMAL)
    cv2.imshow("Board View (45x29 in)", board)
    cv2.namedWindow("Source + Path", cv2.WINDOW_NORMAL)
    cv2.imshow("Source + Path", overlay)
    print("Press any key in a window to close.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
