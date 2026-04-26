#!/usr/bin/env python3
"""
Specs → Scribbles (ESP32) comms bridge.

Subscribes to:
  /specs/bot_pose      Float32MultiArray [x_norm, y_norm, heading_deg]
  /specs/ink_waypoints PoseArray         (board-normalized waypoints)

Sends 3-byte packets over TCP to the ESP32:
  Byte 0: [bit7: type (0=location, 1=waypoint)] [bits6..0: heading_7bit]
  Byte 1: x (0-255, grid)
  Byte 2: y (0-255, grid)

  heading_7bit = round(heading_deg * 128 / 360) & 0x7F   (~2.8° resolution)

Location packets are sent every time a new bot pose arrives (as fast as possible).
Waypoint packets are sent every 2.0s. When the bot reaches the current waypoint
(within `error_bound_m` in real units), we advance to the next waypoint.
Heading on waypoint packets is always 0.
"""
import math
import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseArray


LOCATION_TYPE = 0
WAYPOINT_TYPE = 1


def encode_message(msg_type: int, heading_deg: float, x_grid: int, y_grid: int) -> bytes:
    heading_7bit = int(round((heading_deg % 360.0) * 128.0 / 360.0)) & 0x7F
    byte0 = ((msg_type & 0x01) << 7) | heading_7bit
    x_grid = max(0, min(255, int(x_grid)))
    y_grid = max(0, min(255, int(y_grid)))
    return bytes([byte0, x_grid, y_grid])


def norm_to_grid(n: float) -> int:
    return max(0, min(255, int(round(max(0.0, min(1.0, n)) * 255.0))))


class SpecsCommsNode(Node):
    def __init__(self):
        super().__init__('specs_comms')

        self.declare_parameter('host', '192.168.4.1')
        self.declare_parameter('port', 3333)
        self.declare_parameter('board_size_m', 1.0)
        self.declare_parameter('error_bound_m', 0.05)
        self.declare_parameter('waypoint_period_s', 2.0)

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.board_size_m = self.get_parameter('board_size_m').get_parameter_value().double_value
        self.error_bound_m = self.get_parameter('error_bound_m').get_parameter_value().double_value
        waypoint_period = self.get_parameter('waypoint_period_s').get_parameter_value().double_value

        self.error_bound_norm = self.error_bound_m / max(self.board_size_m, 1e-6)

        self.sock = None
        self.sock_lock = threading.Lock()
        self.connect_to_esp32()

        self.bot_pose = None             # (x_norm, y_norm, heading_deg)
        self.waypoints = []              # list of (x_norm, y_norm)
        self.current_wp_idx = 0
        self.loc_count = 0
        self.loc_log_every = 30          # info-log every Nth LOC packet

        self.bot_pose_sub = self.create_subscription(
            Float32MultiArray, '/specs/bot_pose', self.bot_pose_callback, 10
        )
        self.waypoints_sub = self.create_subscription(
            PoseArray, '/specs/ink_waypoints', self.waypoints_callback, 10
        )
        self.waypoint_timer = self.create_timer(waypoint_period, self.send_current_waypoint)

        self.get_logger().info(
            f"specs_comms: host={self.host}:{self.port} board={self.board_size_m}m "
            f"error_bound={self.error_bound_m}m (norm={self.error_bound_norm:.4f}) "
            f"wp_period={waypoint_period}s"
        )

    # ── socket ────────────────────────────────────────────────────────────

    def connect_to_esp32(self):
        try:
            s = socket.create_connection((self.host, self.port), timeout=5)
            s.settimeout(None)
            with self.sock_lock:
                self.sock = s
            self.get_logger().info(f"Connected to ESP32 at {self.host}:{self.port}")
        except Exception as exc:
            self.get_logger().error(f"Could not connect to ESP32: {exc}")
            self.sock = None

    def send_packet(self, packet: bytes):
        with self.sock_lock:
            if self.sock is None:
                self.connect_to_esp32()
            if self.sock is None:
                return False
            try:
                self.sock.sendall(packet)
                return True
            except Exception as exc:
                self.get_logger().warning(f"Socket send failed, reconnecting: {exc}")
                try:
                    self.sock.close()
                except Exception:
                    pass
                self.sock = None
                return False

    # ── callbacks ─────────────────────────────────────────────────────────

    def bot_pose_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            self.get_logger().warning(f"bot_pose msg has {len(msg.data)} fields, expected 3")
            return
        x_norm, y_norm, heading_deg = float(msg.data[0]), float(msg.data[1]), float(msg.data[2])
        self.bot_pose = (x_norm, y_norm, heading_deg)

        packet = encode_message(
            LOCATION_TYPE, heading_deg, norm_to_grid(x_norm), norm_to_grid(y_norm)
        )
        sent = self.send_packet(packet)
        self.loc_count += 1
        if sent and (self.loc_count == 1 or self.loc_count % self.loc_log_every == 0):
            self.get_logger().info(
                f"LOC #{self.loc_count} → x={norm_to_grid(x_norm)} "
                f"y={norm_to_grid(y_norm)} h={heading_deg:.1f}°"
            )

        self.maybe_advance_waypoint()

    def waypoints_callback(self, msg: PoseArray):
        new_waypoints = [(p.position.x, p.position.y) for p in msg.poses]
        if new_waypoints != self.waypoints:
            self.waypoints = new_waypoints
            self.current_wp_idx = 0
            self.get_logger().info(f"Received {len(self.waypoints)} waypoints; resetting index to 0")

    # ── waypoint logic ────────────────────────────────────────────────────

    def maybe_advance_waypoint(self):
        if self.bot_pose is None or self.current_wp_idx >= len(self.waypoints):
            return
        bx, by, _ = self.bot_pose
        wx, wy = self.waypoints[self.current_wp_idx]
        dist_norm = math.hypot(bx - wx, by - wy)
        if dist_norm <= self.error_bound_norm:
            self.get_logger().info(
                f"Reached waypoint {self.current_wp_idx} "
                f"(dist={dist_norm * self.board_size_m * 100:.1f}cm); advancing"
            )
            self.current_wp_idx += 1
            # Send the next waypoint immediately so the ESP32's lookahead
            # controller gets a fresh path right away — otherwise the bot
            # idles until the next timer tick (and may scrub many waypoints
            # forward in the meantime, causing a big jump).
            self.send_current_waypoint()

    def send_current_waypoint(self):
        if not self.waypoints:
            return
        if self.current_wp_idx >= len(self.waypoints):
            self.get_logger().info("All waypoints reached.")
            return
        wx, wy = self.waypoints[self.current_wp_idx]
        packet = encode_message(WAYPOINT_TYPE, 0.0, norm_to_grid(wx), norm_to_grid(wy))
        if self.send_packet(packet):
            self.get_logger().info(
                f"WP[{self.current_wp_idx}] → x={norm_to_grid(wx)} y={norm_to_grid(wy)}"
            )

    def destroy_node(self):
        with self.sock_lock:
            if self.sock is not None:
                try:
                    self.sock.close()
                except Exception:
                    pass
                self.sock = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpecsCommsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
