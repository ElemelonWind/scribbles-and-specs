#!/usr/bin/env python3
"""
Standalone test script for ink detection (no ROS required).
Usage:
    python ink_detection_test.py <image_path> [--output <output_path>]

If apriltags are detected, the board boundary is inferred automatically.
Otherwise, a window opens for you to click the 4 board corners (TL, TR, BR, BL).
"""
import argparse
import sys
import cv2
import numpy as np

try:
    import apriltag
except ImportError:
    apriltag = None

from ink_detection import (
    WINDOW_NAME,
    create_board_mask,
    create_apriltag_detector,
    create_apriltag_exclusion_mask,
    extract_ink_mask,
    extract_ink_mask_hsv,
    find_contour_centroids,
    cluster_waypoints,
)

# ── Manual corner selection ──────────────────────────────────────────────────

clicked_points = []


def _mouse_callback(event, x, y, _flags, _param):
    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_points) < 4:
        clicked_points.append((x, y))
        print(f"  Corner {len(clicked_points)}: ({x}, {y})")


def select_corners_manually(frame):
    """Open a window and let the user click 4 corners (TL, TR, BR, BL)."""
    print("Click 4 board corners in order: TL → TR → BR → BL")
    win = "Select Board Corners"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, _mouse_callback)

    while len(clicked_points) < 4:
        display = frame.copy()
        for i, pt in enumerate(clicked_points):
            cv2.circle(display, pt, 6, (0, 255, 0), -1)
            if i > 0:
                cv2.line(display, clicked_points[i - 1], pt, (0, 255, 0), 2)
        cv2.imshow(win, display)
        if cv2.waitKey(30) & 0xFF == ord('q'):
            print("Aborted.")
            sys.exit(0)

    cv2.destroyWindow(win)
    return np.array(clicked_points, dtype=np.float32)


# ── Board detection via apriltags ────────────────────────────────────────────

def detect_board_from_tags(gray):
    """Try to derive board corners from apriltag detections."""
    detector = create_apriltag_detector()
    if detector is None:
        return None, None, None

    detections = detector.detect(gray)
    if not detections:
        return None, detections, None

    # Collect all tag corner points to estimate the board boundary
    all_corners = []
    for d in detections:
        if hasattr(d, 'corners'):
            all_corners.extend(d.corners.tolist())
        elif 'lb-rb-rt-lt' in d:
            all_corners.extend(np.array(d['lb-rb-rt-lt']).reshape(-1, 2).tolist())

    if len(all_corners) < 4:
        return None, detections, None

    pts = np.array(all_corners, dtype=np.float32)
    hull = cv2.convexHull(pts)
    rect = cv2.minAreaRect(hull)
    box = cv2.boxPoints(rect)
    # Order: TL, TR, BR, BL
    box = order_points(box)
    return box, detections, detector


def order_points(pts):
    """Order points as TL, TR, BR, BL."""
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).ravel()
    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]
    tr = pts[np.argmin(d)]
    bl = pts[np.argmax(d)]
    return np.array([tl, tr, br, bl], dtype=np.float32)


# ── Homography helper ────────────────────────────────────────────────────────

def compute_homography(corners):
    """Map pixel corners → normalised [0,1]×[0,1] space."""
    dst = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float32)
    H, _ = cv2.findHomography(corners, dst)
    return H


# ── Visualisation (mirrors the ROS node but saves/shows the result) ──────────

def visualize(frame, board_corners, centroids, waypoints):
    overlay = frame.copy()
    cv2.polylines(overlay, [np.int32(board_corners)], True, (0, 255, 255), 2)

    for cx, cy in centroids:
        cv2.circle(overlay, (cx, cy), 5, (0, 0, 255), -1)

    for idx, (x_norm, y_norm) in enumerate(waypoints):
        label = f"W{idx}"
        cv2.putText(overlay, label, (10 + idx * 60, 30),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    info = f"Ink marks={len(centroids)}  waypoints={len(waypoints)}"
    cv2.putText(overlay, info, (10, frame.shape[0] - 20),
                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return overlay


# ── Main pipeline ────────────────────────────────────────────────────────────

def run(image_path, output_path):
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Error: could not read image '{image_path}'")
        sys.exit(1)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 1. Determine board corners
    board_corners, detections, detector = detect_board_from_tags(gray)
    if board_corners is None:
        print("No apriltags found – falling back to manual corner selection.")
        board_corners = select_corners_manually(frame)
        # Re-run tag detection so we still have exclusion masks
        detector = create_apriltag_detector()
        detections = detector.detect(gray) if detector else None

    print(f"Board corners:\n{board_corners}")

    # 2. Build masks
    board_mask = create_board_mask(gray.shape, board_corners)
    apriltag_mask = np.zeros(gray.shape, dtype=np.uint8)
    if detections:
        apriltag_mask = create_apriltag_exclusion_mask(gray.shape, detections, padding=16)

    print(f"Apriltag detections: {len(detections) if detections else 0}")
    print(f"Board mask pixels: {np.count_nonzero(board_mask)}  "
          f"Apriltag mask pixels: {np.count_nonzero(apriltag_mask)}")

    # 3. Ink detection (same cascade as the ROS node)
    ink_mask = extract_ink_mask(gray, board_mask, apriltag_mask, mode='auto')
    centroids = find_contour_centroids(ink_mask, min_area=15)
    print(f"Auto ink contours: {len(centroids)}, ink mask pixels: {np.count_nonzero(ink_mask)}")

    if len(centroids) == 0:
        print("Retry with low threshold...")
        ink_mask = extract_ink_mask(gray, board_mask, apriltag_mask, mode='low')
        centroids = find_contour_centroids(ink_mask, min_area=10)
        print(f"Low ink contours: {len(centroids)}, ink mask pixels: {np.count_nonzero(ink_mask)}")

    if len(centroids) == 0:
        print("Fallback to HSV ink filter...")
        ink_mask = extract_ink_mask_hsv(frame, board_mask, apriltag_mask)
        centroids = find_contour_centroids(ink_mask, min_area=10)
        print(f"HSV ink contours: {len(centroids)}, ink mask pixels: {np.count_nonzero(ink_mask)}")

    # 4. Normalise centroids → waypoints
    H = compute_homography(board_corners)
    normalized_points = []
    for cx, cy in centroids:
        pixel_point = np.array([[[float(cx), float(cy)]]], dtype=np.float32)
        projected = cv2.perspectiveTransform(pixel_point, H)[0][0]
        if 0.0 <= projected[0] <= 1.0 and 0.0 <= projected[1] <= 1.0:
            normalized_points.append((float(projected[0]), float(projected[1])))

    waypoints = cluster_waypoints(normalized_points)
    print(f"Waypoints: {len(waypoints)}  (centroids {len(centroids)}, "
          f"normalized {len(normalized_points)})")

    # 5. Visualise & save
    overlay = visualize(frame, board_corners, centroids, waypoints)

    if output_path:
        cv2.imwrite(output_path, overlay)
        print(f"Saved annotated image → {output_path}")

    # Also show the ink mask and overlay in windows
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.imshow(WINDOW_NAME, overlay)
    cv2.namedWindow("Ink Mask", cv2.WINDOW_NORMAL)
    cv2.imshow("Ink Mask", ink_mask)
    print("Press any key to close.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Test ink detection on a single image.")
    parser.add_argument("image", help="Path to the input image")
    parser.add_argument("-o", "--output", default=None,
                        help="Path to save the annotated output image")
    args = parser.parse_args()
    run(args.image, args.output)
