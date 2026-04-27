#!/usr/bin/env python3
"""
Ink detection — eraser sweep mode.

We replaced the camera-based detector with a hardcoded serpentine sweep.
The detector kept jittering and re-driving to the same spot; running the
bot in a fixed pass over the whole board cleans the whiteboard without any
perception. Mirrors draw_waypoints.py's structure since that's known to
move the bot reliably.

Publishes:
  /specs/ink_waypoints  geometry_msgs/PoseArray  (board-normalized x/y in [0,1])

Parameters:
  rows         number of horizontal serpentine rows (default 6)
  margin_x     normalized inset on the left/right edges (default 0.13 ≈ 6 in)
  margin_y     normalized inset on the top/bottom edges (default 0.21 ≈ 6 in)
  republish_s  period to re-publish the same waypoint list (default 5.0)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose


def sweep(margin_x, margin_y, rows):
    """Alternating-row serpentine over the rectangle
    [margin_x, 1-margin_x] × [margin_y, 1-margin_y].
    """
    x_min, x_max = margin_x, 1.0 - margin_x
    y_min, y_max = margin_y, 1.0 - margin_y
    pts = []
    for i in range(rows):
        if rows == 1:
            y = (y_min + y_max) / 2.0
        else:
            y = y_min + i * (y_max - y_min) / (rows - 1)
        if i % 2 == 0:
            pts.append((x_min, y))
            pts.append((x_max, y))
        else:
            pts.append((x_max, y))
            pts.append((x_min, y))
    return pts


class InkDetectionNode(Node):
    def __init__(self):
        super().__init__('ink_detection')

        # Defaults pick a 6-inch inset on a 45×29 in board:
        #   6 / 45 ≈ 0.133  on x
        #   6 / 29 ≈ 0.207  on y
        self.declare_parameter('rows', 6)
        self.declare_parameter('margin_x', 0.133)
        self.declare_parameter('margin_y', 0.207)
        self.declare_parameter('republish_s', 5.0)

        rows = self.get_parameter('rows').get_parameter_value().integer_value
        mx = self.get_parameter('margin_x').get_parameter_value().double_value
        my = self.get_parameter('margin_y').get_parameter_value().double_value
        republish_s = self.get_parameter('republish_s').get_parameter_value().double_value

        raw = sweep(mx, my, rows)
        # Clamp to the board (paranoid).
        self.waypoints = [(max(0.0, min(1.0, x)), max(0.0, min(1.0, y))) for x, y in raw]

        self.pub = self.create_publisher(PoseArray, '/specs/ink_waypoints', 10)
        self.timer = self.create_timer(republish_s, self.publish_waypoints)
        # Publish immediately too.
        self.publish_waypoints()

        self.get_logger().info(
            f"ink_detection: rows={rows} margin=({mx:.3f},{my:.3f}) "
            f"→ {len(self.waypoints)} waypoints, republishing every {republish_s}s"
        )
        for i, (x, y) in enumerate(self.waypoints):
            self.get_logger().info(f"  W{i}: ({x:.3f}, {y:.3f})")

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
    node = InkDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
