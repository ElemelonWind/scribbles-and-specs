#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

import apriltag

WINDOW_NAME = "Scribbles Localization"

# Configuration for board and bot AprilTags
BOARD_CORNER_TAG_IDS = [1, 2, 4, 3]  # TL, TR, BR, BL
BOT_TAG_ID = 10
CORNER_TAG_SIZE_M = 0.100  # meters (adjust to real tag size)
BOT_TAG_SIZE_M = 0.050

# Grid resolution on whiteboard
GRID_ROWS = 255
GRID_COLS = 255


def create_detector():
    if apriltag is not None:
        if hasattr(apriltag, 'Detector'):
            return apriltag.Detector()
        if hasattr(apriltag, 'apriltag'):
            return apriltag.apriltag('tag36h11')
        if callable(apriltag):
            return apriltag()

def draw_tag(frame, detection, color=(0, 255, 0)):
    pts = np.int32(detection['lb-rb-rt-lt']).reshape(-1, 2)
    cv2.polylines(frame, [pts], True, color, 2)
    cx, cy = int(detection['center'][0]), int(detection['center'][1])
    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
    cv2.putText(frame, f"ID {detection['id']}", (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def get_corner_positions(detections):
    corners = {}
    for d in detections:
        if d['id'] in BOARD_CORNER_TAG_IDS:
            corners[d['id']] = np.array(d['center'], dtype=np.float32)
    if len(corners) != 4:
        return None

    ordered = [corners.get(i) for i in BOARD_CORNER_TAG_IDS]
    if any(x is None for x in ordered):
        return None

    return np.array(ordered, dtype=np.float32)


def localize_bot(bot_detection, board_corners):
    src_pts = board_corners
    dst_pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], dtype=np.float32)

    H, status = cv2.findHomography(src_pts, dst_pts)
    if H is None:
        return None

    tag_center = bot_detection['center']
    pt = np.array([[tag_center]], dtype=np.float32)
    dst = cv2.perspectiveTransform(pt, H)[0][0]

    # Swap so the published x = "across the board" and y = "down the board"
    # matches the user's expected orientation.
    x_norm = np.clip(dst[0], 0.0, 1.0)
    y_norm = np.clip(dst[1], 0.0, 1.0)

    col = int(x_norm * (GRID_COLS - 1))
    row = int(y_norm * (GRID_ROWS - 1))

    # Heading in the board frame: angle of the tag's "up" direction.
    # Tag corners are ordered [lb, rb, rt, lt] in 'lb-rb-rt-lt'.
    # Up vector = midpoint(lt, rt) - midpoint(lb, rb), projected through H.
    corners = np.array(bot_detection['lb-rb-rt-lt'], dtype=np.float32).reshape(-1, 2)
    bottom_mid = (corners[0] + corners[1]) / 2.0
    top_mid = (corners[2] + corners[3]) / 2.0
    pts_img = np.array([[bottom_mid], [top_mid]], dtype=np.float32)
    pts_board = cv2.perspectiveTransform(pts_img, H).reshape(-1, 2)
    # pts_board components are (down, across); swap into (x=across, y=down)
    # before computing heading.
    up_vec_x = pts_board[1][1] - pts_board[0][1]   # across delta
    up_vec_y = pts_board[1][0] - pts_board[0][0]   # down delta
    # Heading: 0° = -y direction (board "up"), increasing clockwise.
    heading_deg = (np.degrees(np.arctan2(up_vec_x, -up_vec_y))) % 360.0

    return {
        "x_norm": float(x_norm),
        "y_norm": float(y_norm),
        "heading_deg": float(heading_deg),
        "row": int(row),
        "col": int(col),
        "grid": (int(row), int(col)),
    }


def get_board_homography(board_corners):
    dst_pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], dtype=np.float32)
    H, status = cv2.findHomography(board_corners, dst_pts)
    return H


def process_frame(frame, detector):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray)

    board_corners = get_corner_positions(detections)
    bot_detection = next((d for d in detections if d['id'] == BOT_TAG_ID), None)

    for d in detections:
        color = (0, 255, 0) if d['id'] in BOARD_CORNER_TAG_IDS else (255, 0, 0) if d['id'] == BOT_TAG_ID else (0, 255, 255)
        draw_tag(frame, d, color)

    output = {}
    if board_corners is not None:
        cv2.polylines(frame, [np.int32(board_corners)], True, (0, 255, 255), 3)
        output["board_detected"] = True
        output["board_homography"] = get_board_homography(board_corners)
        output["board_corners"] = board_corners
    else:
        output["board_detected"] = False
        output["board_homography"] = None
        output["board_corners"] = None

    if bot_detection is not None and board_corners is not None:
        loc = localize_bot(bot_detection, board_corners)
        if loc is not None:
            output["bot_location"] = loc
            cv2.putText(frame, f"Grid {loc['row']},{loc['col']}",
                        (int(bot_detection['center'][0]) + 20, int(bot_detection['center'][1]) - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Norm ({loc['x_norm']:.2f},{loc['y_norm']:.2f})",
                        (int(bot_detection['center'][0]) + 20, int(bot_detection['center'][1]) + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    else:
        output["bot_location"] = None

    output["detections"] = detections
    return frame, output


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization')
        self.bridge = CvBridge()
        self.detector = create_detector()

        self.board_detected_pub = self.create_publisher(Bool, '/specs/board_detected', 10)
        self.board_homography_pub = self.create_publisher(Float32MultiArray, '/specs/board_homography', 10)
        self.board_corners_pub = self.create_publisher(Float32MultiArray, '/specs/board_corners', 10)
        # Bot pose: [x_norm, y_norm, heading_deg]
        self.bot_pose_pub = self.create_publisher(Float32MultiArray, '/specs/bot_pose', 10)

        # Parameter to control whether to display the localization image
        self.show_localization = self.declare_parameter('show_localization', True).value

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f"Failed to convert image: {exc}")
            return

        frame, output = process_frame(frame, self.detector)

        self.board_detected_pub.publish(Bool(data=output["board_detected"]))
        if output.get("board_homography") is not None:
            self.board_homography_pub.publish(
                Float32MultiArray(data=output["board_homography"].reshape(-1).tolist())
            )
        if output.get("board_corners") is not None:
            self.board_corners_pub.publish(
                Float32MultiArray(data=output["board_corners"].reshape(-1).tolist())
            )

        if output["board_detected"]:
            if output["bot_location"]:
                loc = output["bot_location"]
                self.bot_pose_pub.publish(
                    Float32MultiArray(data=[loc['x_norm'], loc['y_norm'], loc['heading_deg']])
                )
                self.get_logger().info(
                    f"Scribble coordinates: grid={loc['grid']} norm=({loc['x_norm']:.3f},{loc['y_norm']:.3f}) heading={loc['heading_deg']:.1f}°"
                )
            else:
                self.get_logger().info("Scribble bot tag not detected")
        else:
            self.get_logger().info("Board corner tags not fully detected")

        if self.show_localization:
            cv2.imshow(WINDOW_NAME, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
