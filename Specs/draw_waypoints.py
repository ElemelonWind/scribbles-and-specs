#!/usr/bin/env python3
"""
Drop-in replacement for `ink_detection.py` for the "always-drawing marker" mode.

Instead of detecting existing ink and computing erase waypoints, this node
publishes a fixed sequence of waypoints that traces a chosen shape. Because
the marker is always touching the board, every segment between waypoints
draws a line — so the path is one continuous stroke.

Publishes:
  /specs/ink_waypoints  geometry_msgs/PoseArray  (board-normalized x/y in [0,1])

Parameters:
  shape          one of: square, triangle, circle, star, line, zigzag (default: square)
  center_x       x of shape center, normalized (default: 0.5)
  center_y       y of shape center, normalized (default: 0.5)
  size           half-side / radius, normalized (default: 0.3)
  num_points     samples for circle (default: 24)
  republish_s    period to re-publish the same waypoint list (default: 5.0)
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose


def square(cx, cy, s):
    return [(cx - s, cy - s), (cx + s, cy - s), (cx + s, cy + s), (cx - s, cy + s), (cx - s, cy - s)]


def triangle(cx, cy, s):
    pts = []
    for i in range(4):  # close the loop
        a = -math.pi / 2 + i * 2 * math.pi / 3
        pts.append((cx + s * math.cos(a), cy + s * math.sin(a)))
    return pts


def circle(cx, cy, r, n):
    pts = []
    for i in range(n + 1):  # close the loop
        a = 2 * math.pi * i / n
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def star(cx, cy, s):
    # 5-point star drawn in one stroke: visit every other vertex of a pentagon.
    pts = []
    for i in range(6):  # 5 + close
        a = -math.pi / 2 + i * 2 * 2 * math.pi / 5
        pts.append((cx + s * math.cos(a), cy + s * math.sin(a)))
    return pts


def line(cx, cy, s):
    return [(cx, cy - s), (cx, cy + s)]


def zigzag(cx, cy, s, rows=4):
    pts = [(0.3, 0.3), (0.3, 0.7), (0.35, 0.7), (0.35, 0.3), (0.4, 0.3), (0.4, 0.7), (0.45, 0.7), (0.45, 0.3), (0.5, 0.3), (0.5, 0.7), (0.55, 0.7), (0.55, 0.3), (0.6, 0.3), (0.6, 0.7), (0.65, 0.7), (0.65, 0.3), (0.7, 0.3), (0.7, 0.7)]
  #  step = 2 * s / rows
  #  for i in range(rows + 1):
  #      y = cy - s + i * step
  #      x = cx - s if i % 2 == 0 else cx + s
  #      pts.append((x, y))
    return pts+pts[::-1]


SHAPES = {
    'square': lambda cx, cy, s, n: square(cx, cy, s),
    'triangle': lambda cx, cy, s, n: triangle(cx, cy, s),
    'circle': lambda cx, cy, s, n: circle(cx, cy, s, n),
    'star': lambda cx, cy, s, n: star(cx, cy, s),
    'line': lambda cx, cy, s, n: line(cx, cy, s),
    'zigzag': lambda cx, cy, s, n: zigzag(cx, cy, s),
}


class DrawWaypointsNode(Node):
    def __init__(self):
        super().__init__('draw_waypoints')

        self.declare_parameter('shape', 'square')
        self.declare_parameter('center_x', 0.5)
        self.declare_parameter('center_y', 0.5)
        self.declare_parameter('size', 0.3)
        self.declare_parameter('num_points', 24)
        self.declare_parameter('republish_s', 5.0)

        shape = self.get_parameter('shape').get_parameter_value().string_value
        cx = self.get_parameter('center_x').get_parameter_value().double_value
        cy = self.get_parameter('center_y').get_parameter_value().double_value
        s = self.get_parameter('size').get_parameter_value().double_value
        n = self.get_parameter('num_points').get_parameter_value().integer_value
        republish_s = self.get_parameter('republish_s').get_parameter_value().double_value

        if shape not in SHAPES:
            raise ValueError(f"Unknown shape '{shape}'. Choices: {list(SHAPES)}")

        raw = SHAPES[shape](cx, cy, s, n)
        # Clamp to the board.
        self.waypoints = [(max(0.0, min(1.0, x)), max(0.0, min(1.0, y))) for x, y in raw]

        self.pub = self.create_publisher(PoseArray, '/specs/ink_waypoints', 10)
        self.timer = self.create_timer(republish_s, self.publish_waypoints)
        # Publish immediately too.
        self.publish_waypoints()

        self.get_logger().info(
            f"draw_waypoints: shape={shape} center=({cx:.2f},{cy:.2f}) size={s:.2f} "
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
        stop_p = Pose()
        stop_p.position.x = 1.0
        stop_p.position.y = 1.0
        stop_p.position.z = 1.0
        msg.poses.append(stop_p)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DrawWaypointsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
