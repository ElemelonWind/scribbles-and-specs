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

def _skeletonize(binary):
    if hasattr(cv2, 'ximgproc') and hasattr(cv2.ximgproc, 'thinning'):
        return cv2.ximgproc.thinning(binary, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
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
    skel_bin = (skel > 0).astype(np.uint8)
    kernel = np.array([[1, 1, 1],
                        [1, 0, 1],
                        [1, 1, 1]], dtype=np.uint8)
    nbrs = cv2.filter2D(skel_bin, -1, kernel)
    return np.argwhere((skel_bin > 0) & (nbrs == 1))


def _walk_skeleton(skel, start_rc):
    work = skel.copy()
    h, w = work.shape
    r, c = int(start_rc[0]), int(start_rc[1])
    path = [(c, r)]
    work[r, c] = 0
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

    skel = _skeletonize(binary)
    if cv2.countNonZero(skel) == 0:
        raise ValueError("Empty skeleton — check threshold / invert")
    skel, n_strokes = _largest_component(skel)

    endpoints = _find_endpoints(skel)
    if len(endpoints) >= 1:
        endpoints = endpoints[np.lexsort((endpoints[:, 1], endpoints[:, 0]))]
        start_rc = endpoints[0]
    else:
        start_rc = np.argwhere(skel > 0)[0]

    path = _walk_skeleton(skel, start_rc)
    pts = np.array(path, dtype=np.float32)

    if simplify_eps > 0 and len(pts) > 2:
        pts = cv2.approxPolyDP(pts.reshape(-1, 1, 2),
                                 simplify_eps, closed=False).reshape(-1, 2).astype(np.float32)

    h, w = img.shape
    return pts, work, w, h, n_strokes


def resample_uniform(pts, spacing_norm):
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
    if len(pts_int):
        sp = pts_int[0]
        cv2.circle(overlay, (int(sp[0]), int(sp[1])), 9, (255, 0, 0), 2)
        cv2.putText(overlay, "START", (int(sp[0]) + 11, int(sp[1]) - 11),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    return overlay


# ── Main ────────────────────────────────────────────────────────────────────

def main(argv=None):
    ap = argparse.ArgumentParser(description="Preview image waypoints on a board grid (no ROS).")
    ap.add_argument("image", help="Path to input image")
    ap.add_argument("--max-frac", type=float, default=0.8, help="Fraction of board to fill (default 0.8)")
    ap.add_argument("--simplify-eps", type=float, default=2.0, help="approxPolyDP epsilon in input pixels (default 2.0)")
    ap.add_argument("--resample-spacing", type=float, default=0.05,
                     help="Target spacing between waypoints in normalized board units (0 to disable, default 0.05)")
    ap.add_argument("--invert", choices=['auto', 'yes', 'no'], default='auto')
    ap.add_argument("-o", "--output", default=None, help="Save board-view preview here")
    args = ap.parse_args(argv)

    pts_px, work, w, h, n_contours = extract_path(
        args.image, simplify_eps=args.simplify_eps, invert=args.invert,
    )
    norm_pts, (nw, nh, pw, ph) = pixels_to_normalized(
        pts_px, w, h, max_frac=args.max_frac,
    )
    if args.resample_spacing > 0:
        norm_pts = resample_uniform(norm_pts, args.resample_spacing)
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
