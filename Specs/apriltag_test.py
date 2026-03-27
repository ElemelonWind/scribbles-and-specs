import cv2
import numpy as np

# The script will try to use the apriltag PyPI library.
try:
    import apriltag
except ImportError:
    apriltag = None

CAMERA_INDEX = 0
WINDOW_NAME = "Apriltag Single-Tag Test"
SINGLE_TAG_ID = 0


def create_detector():
    if apriltag is not None:
        return apriltag.Detector()

    if hasattr(cv2, "apriltag"):
        # OpenCV 4.7+ built-in AprilTag
        return cv2.apriltag_AprilTagDetector(cv2.apriltag_TagDetector_Params())

    raise RuntimeError("No AprilTag detector found (install apriltag package or use opencv-contrib)")


def draw_detection(frame, detection):
    pts = np.int32(detection.corners).reshape(-1, 2)
    cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

    c_x = int(detection.center[0])
    c_y = int(detection.center[1])
    cv2.circle(frame, (c_x, c_y), 5, (0, 0, 255), -1)
    label = f"ID {detection.tag_id}"
    cv2.putText(frame, label, (c_x + 10, c_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)


def detect_single_tag(frame, detector):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if apriltag is not None:
        detections = detector.detect(gray)
    else:
        # OpenCV wrapper returns tuple: (detection list, _)
        detections, _ = detector.detect(gray)

    # Focus on one tag if there are multiple.
    for detection in detections:
        if detection.tag_id == SINGLE_TAG_ID:
            draw_detection(frame, detection)
            x, y = int(detection.center[0]), int(detection.center[1])
            print(f"Tag {SINGLE_TAG_ID} center: ({x}, {y})")
            return

    print(f"Tag {SINGLE_TAG_ID} not found")


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

        detect_single_tag(frame, detector)
        cv2.imshow(WINDOW_NAME, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
