import cv2
import numpy as np

try:
    import apriltag
except ImportError:
    apriltag = None

CAMERA_INDEX = 0
WINDOW_NAME = "Scribbles Localization"

# Configuration for board and bot AprilTags
BOARD_CORNER_TAG_IDS = [1, 2, 3, 4]  # TL, TR, BR, BL
BOT_TAG_ID = 10
CORNER_TAG_SIZE_M = 0.100  # meters (adjust to real tag size)
BOT_TAG_SIZE_M = 0.050

# Grid resolution on whiteboard
GRID_ROWS = 10
GRID_COLS = 10


def create_detector():
    if apriltag is not None:
        return apriltag.Detector()

    if hasattr(cv2, "apriltag"):
        return cv2.apriltag_AprilTagDetector(cv2.apriltag_TagDetector_Params())

    raise RuntimeError("No AprilTag detector found. Install apriltag or opencv-contrib-python")


def draw_tag(frame, detection, color=(0, 255, 0)):
    pts = np.int32(detection.corners).reshape(-1, 2)
    cv2.polylines(frame, [pts], True, color, 2)
    cx, cy = int(detection.center[0]), int(detection.center[1])
    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
    cv2.putText(frame, f"ID {detection.tag_id}", (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def get_corner_positions(detections):
    corners = {}
    for d in detections:
        if d.tag_id in BOARD_CORNER_TAG_IDS:
            corners[d.tag_id] = np.array(d.center, dtype=np.float32)
    if len(corners) != 4:
        return None

    # order corners as TL, TR, BR, BL
    ordered = [corners.get(i) for i in BOARD_CORNER_TAG_IDS]
    if any(x is None for x in ordered):
        return None

    return np.array(ordered, dtype=np.float32)


def localize_bot(tag_center, board_corners):
    # Map board corners to normalized board coordinates in unit square
    src_pts = board_corners
    dst_pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], dtype=np.float32)

    H, status = cv2.findHomography(src_pts, dst_pts)
    if H is None:
        return None

    pt = np.array([[tag_center]], dtype=np.float32)
    dst = cv2.perspectiveTransform(pt, H)[0][0]

    x_norm = np.clip(dst[0], 0.0, 1.0)
    y_norm = np.clip(dst[1], 0.0, 1.0)

    col = int(x_norm * (GRID_COLS - 1))
    row = int(y_norm * (GRID_ROWS - 1))

    return {
        "x_norm": float(x_norm),
        "y_norm": float(y_norm),
        "row": int(row),
        "col": int(col),
        "grid": (int(row), int(col)),
    }


def process_frame(frame, detector):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if apriltag is not None:
        detections = detector.detect(gray)
    else:
        detections, _ = detector.detect(gray)

    board_corners = get_corner_positions(detections)
    bot_detection = next((d for d in detections if d.tag_id == BOT_TAG_ID), None)

    for d in detections:
        color = (0, 255, 0) if d.tag_id in BOARD_CORNER_TAG_IDS else (255, 0, 0) if d.tag_id == BOT_TAG_ID else (0, 255, 255)
        draw_tag(frame, d, color)

    output = {}
    if board_corners is not None:
        cv2.polylines(frame, [np.int32(board_corners)], True, (0, 255, 255), 3)
        output["board_detected"] = True
    else:
        output["board_detected"] = False

    if bot_detection is not None and board_corners is not None:
        loc = localize_bot(bot_detection.center, board_corners)
        if loc is not None:
            output["bot_location"] = loc
            cv2.putText(frame, f"Grid {loc['row']},{loc['col']}",
                        (int(bot_detection.center[0]) + 20, int(bot_detection.center[1]) - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Norm ({loc['x_norm']:.2f},{loc['y_norm']:.2f})",
                        (int(bot_detection.center[0]) + 20, int(bot_detection.center[1]) + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    else:
        output["bot_location"] = None

    output["detections"] = detections
    return frame, output


def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("ERROR: Could not open camera")
        return

    detector = create_detector()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("ERROR: Could not read frame")
            break

        frame, output = process_frame(frame, detector)

        if output["board_detected"]:
            if output["bot_location"]:
                loc = output["bot_location"]
                print(f"Scribble coordinates: grid={loc['grid']} norm=({loc['x_norm']:.3f},{loc['y_norm']:.3f})")
            else:
                print("Scribble bot tag not detected")
        else:
            print("Board corner tags not fully detected")

        cv2.imshow(WINDOW_NAME, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
