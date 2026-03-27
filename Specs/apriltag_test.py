#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

import apriltag

WINDOW_NAME = "Apriltag Single-Tag Test"
SINGLE_TAG_ID = 1


def create_detector():
    if apriltag is not None:
        print("Using apriltag Python library for detection.")
        if hasattr(apriltag, 'Detector'):
            return apriltag.Detector()
        if hasattr(apriltag, 'apriltag'):
            return apriltag.apriltag('tag36h11')
        if callable(apriltag):
            return apriltag()

def draw_detection(frame, detection):
    pts = np.int32(detection['lb-rb-rt-lt']).reshape(-1, 2)
    cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

    c_x = int(detection['center'][0])
    c_y = int(detection['center'][1])
    cv2.circle(frame, (c_x, c_y), 5, (0, 0, 255), -1)
    label = f"ID {detection['id']}"
    cv2.putText(frame, label, (c_x + 10, c_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)


def detect_single_tag(frame, detector):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    detections = detector.detect(gray)

    # Focus on one tag if there are multiple.
    for detection in detections:
        if detection['id'] == SINGLE_TAG_ID:
            draw_detection(frame, detection)
            x, y = int(detection['center'][0]), int(detection['center'][1])
            print(f"Tag {SINGLE_TAG_ID} center: ({x}, {y})")
            return

    print(f"Tag {SINGLE_TAG_ID} not found")


class AprilTagTestNode(Node):
    def __init__(self):
        super().__init__('apriltag_test')
        self.bridge = CvBridge()
        self.detector = create_detector()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.subscription

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f"Failed to convert image: {exc}")
            return

        detect_single_tag(frame, self.detector)
        cv2.imshow(WINDOW_NAME, frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
