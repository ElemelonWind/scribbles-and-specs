import cv2

CAMERA_INDEX = 0
WINDOW_NAME = "PiCam Video Display"


def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("ERROR: Could not open camera")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("ERROR: Could not read frame")
            break

        cv2.imshow(WINDOW_NAME, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
