#!/usr/bin/env python3
"""
Drop-in replacement for the camera-based ink detector. Instead of trying to
find ink and revisit specific spots — which kept changing its mind and
re-driving to the same place — this node just publishes a serpentine sweep
that drags the eraser across the entire inset board area. Reliable, no
perception required.

Publishes:
  /specs/ink_waypoints  geometry_msgs/PoseArray  (board-normalized x/y in [0,1])

ROS parameters:
  sweep_rows    number of horizontal rows in the serpentine (default 6)
  inset_in      inches the bot's tag stays away from each edge (default 6)
  republish_s   how often to re-broadcast the same waypoint list (default 5)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose


# Physical board dimensions (matches Scribbles main.cpp).
BOARD_WIDTH_IN = 45.0
BOARD_HEIGHT_IN = 29.0


def generate_sweep_waypoints(num_rows, inset_in):
    """Serpentine sweep across the inset rectangle.

    Returns tag-frame waypoints in (x_norm, y_norm); x = across (0 → 1 left
    to right), y = down (0 → 1 top to bottom). The tag stays >= inset_in
    inches from every edge so the bot never approaches the corner AprilTags.
    """
    tag_x_min = inset_in
    tag_x_max = BOARD_WIDTH_IN - inset_in
    tag_y_min = inset_in
    tag_y_max = BOARD_HEIGHT_IN - inset_in

    waypoints = []
    for i in range(num_rows):
        if num_rows == 1:
            tag_y = (tag_y_min + tag_y_max) / 2.0
        else:
            tag_y = tag_y_min + i * (tag_y_max - tag_y_min) / (num_rows - 1)
        # Alternate direction on each row so consecutive waypoints stay
        # close together → continuous serpentine path, not zig-zag jumps.
        xs = [tag_x_min, tag_x_max] if i % 2 == 0 else [tag_x_max, tag_x_min]
        for x in xs:
            waypoints.append((x / BOARD_WIDTH_IN, tag_y / BOARD_HEIGHT_IN))
    return waypoints


class InkSweepNode(Node):
    def __init__(self):
        super().__init__('ink_detection')

        self.declare_parameter('sweep_rows', 6)
        self.declare_parameter('inset_in', 6.0)
        self.declare_parameter('republish_s', 5.0)

        rows = self.get_parameter('sweep_rows').get_parameter_value().integer_value
        inset = self.get_parameter('inset_in').get_parameter_value().double_value
        republish_s = self.get_parameter('republish_s').get_parameter_value().double_value

        self.waypoints = generate_sweep_waypoints(rows, inset)

        self.pub = self.create_publisher(PoseArray, '/specs/ink_waypoints', 10)
        self.timer = self.create_timer(republish_s, self.publish_waypoints)
        self.publish_waypoints()

        self.get_logger().info(
            f"ink_detection sweep: rows={rows} inset={inset}\" "
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
    node = InkSweepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
