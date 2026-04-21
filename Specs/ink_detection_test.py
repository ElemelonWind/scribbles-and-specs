#!/usr/bin/env python3
"""
Standalone ink detection test — no ROS required.
Usage: python ink_detection_test.py <image_path> [-o output.png]

Click 4 board corners (TL → TR → BR → BL), then ink is detected automatically.
Works with any marker color and handles glare.
"""
import argparse
import sys
import cv2
import numpy as np

# ── Corner selection ─────────────────────────────────────────────────────────

clicked_points = []


def _mouse_cb(event, x, y, _flags, _param):
    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_points) < 4:
        clicked_points.append((x, y))
        print(f"  Corner {len(clicked_points)}: ({x}, {y})")


def select_corners(frame):
    """Let the user click 4 corners: TL → TR → BR → BL."""
    print("Click 4 board corners: TL → TR → BR → BL  (q to quit)")
    win = "Select Corners"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, _mouse_cb)

    while len(clicked_points) < 4:
        disp = frame.copy()
        for i, pt in enumerate(clicked_points):
            cv2.circle(disp, pt, 6, (0, 255, 0), -1)
            if i > 0:
                cv2.line(disp, clicked_points[i - 1], pt, (0, 255, 0), 2)
        cv2.imshow(win, disp)
        if cv2.waitKey(30) & 0xFF == ord("q"):
            sys.exit(0)

    cv2.destroyWindow(win)
    return np.array(clicked_points, dtype=np.float32)


# ── Ink detection ────────────────────────────────────────────────────────────

def warp_board(frame, corners, size=800):
    """Perspective-warp the board region to a square image."""
    dst = np.array([[0, 0], [size, 0], [size, size], [0, size]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(corners, dst)
    warped = cv2.warpPerspective(frame, M, (size, size))
    return warped, M


def detect_ink(warped):
    """
    Detect ink marks on a warped board image.

    Strategy: convert to LAB, subtract a heavy-blurred background from each
    channel, then threshold the absolute deviation.  This cancels smooth glare
    and catches any marker colour.
    """
    lab = cv2.cvtColor(warped, cv2.COLOR_BGR2LAB).astype(np.float32)
    # Large kernel so ink marks don't bleed into the background estimate
    ksize = (151, 151)

    # Per-channel local background subtraction
    masks = []
    for ch in range(3):
        channel = lab[:, :, ch]
        bg = cv2.GaussianBlur(channel, ksize, 0)
        diff = np.abs(channel - bg)
        # L channel (0) captures dark/light marks; A/B (1,2) capture colour
        thresh = 7 if ch == 0 else 4
        mask = (diff > thresh).astype(np.uint8) * 255
        masks.append(mask)

    # Combine: ink shows up in L *or* A *or* B
    combined = cv2.bitwise_or(masks[0], cv2.bitwise_or(masks[1], masks[2]))

    # Clean up noise
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel, iterations=1)
    combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel, iterations=1)

    return combined


def unwarp_centroids(centroids, M):
    """Map warped-space centroids back to original image coordinates."""
    if not centroids:
        return []
    M_inv = np.linalg.inv(M)
    pts = np.array(centroids, dtype=np.float32).reshape(1, -1, 2)
    orig = cv2.perspectiveTransform(pts, M_inv)[0]
    return [(int(x), int(y)) for x, y in orig]


def find_centroids(mask, min_area=60):
    """Find contour centroids in the mask."""
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


# ── Grid clustering (normalise to [0,1]² and snap to grid) ──────────────────

GRID = 10


def cluster_to_waypoints(centroids, board_size):
    """Map pixel centroids to normalised waypoints, one per grid cell."""
    grid_map = {}
    for cx, cy in centroids:
        xn = cx / board_size
        yn = cy / board_size
        key = (min(GRID - 1, int(yn * GRID)), min(GRID - 1, int(xn * GRID)))
        if key not in grid_map:
            grid_map[key] = (xn, yn)
    return [grid_map[k] for k in sorted(grid_map)]


# ── Main ─────────────────────────────────────────────────────────────────────

def run(image_path, output_path):
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Error: cannot read '{image_path}'")
        sys.exit(1)

    corners = select_corners(frame)
    print(f"Corners:\n{corners}")

    # Warp board to a flat square
    board_size = 800
    warped, M = warp_board(frame, corners, board_size)

    # Detect ink on the warped board
    ink_mask = detect_ink(warped)
    centroids = find_centroids(ink_mask)
    waypoints = cluster_to_waypoints(centroids, board_size)

    print(f"Ink marks: {len(centroids)}  Waypoints: {len(waypoints)}")
    for i, (x, y) in enumerate(waypoints):
        print(f"  W{i}: ({x:.3f}, {y:.3f})")

    # Draw results on the original (unwarped) frame
    overlay = frame.copy()
    orig_pts = unwarp_centroids(centroids, M)
    for cx, cy in orig_pts:
        cv2.circle(overlay, (cx, cy), 5, (0, 0, 255), -1)
    cv2.polylines(overlay, [np.int32(corners)], True, (0, 255, 255), 2)
    info = f"marks={len(centroids)}  waypoints={len(waypoints)}"
    cv2.putText(overlay, info, (10, frame.shape[0] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    if output_path:
        cv2.imwrite(output_path, overlay)
        print(f"Saved → {output_path}")

    cv2.namedWindow("Detection", cv2.WINDOW_NORMAL)
    cv2.imshow("Detection", overlay)
    cv2.namedWindow("Ink Mask", cv2.WINDOW_NORMAL)
    cv2.imshow("Ink Mask", ink_mask)
    print("Press any key to close.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Test ink detection on a single image.")
    ap.add_argument("image", help="Path to input image")
    ap.add_argument("-o", "--output", default=None, help="Save annotated output")
    args = ap.parse_args()
    run(args.image, args.output)
