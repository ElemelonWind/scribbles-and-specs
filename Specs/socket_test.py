#!/usr/bin/env python3
import socket
import rclpy
from rclpy.node import Node


def send_ping(host: str, port: int) -> str:
    with socket.create_connection((host, port), timeout=5) as sock:
        payload = 'ping'.encode('utf-8')
        sock.sendall(payload + b'\n')

        # wait for complete reply from server (timeout part handled by underlying socket)
        data = sock.recv(256)
        if not data:
            raise RuntimeError('Empty response from server')

        text = data.decode('utf-8', errors='replace').strip()
        if text.lower() == 'pong':
            return text

        # Some servers may return full text or include ping in reply.
        if 'pong' in text.lower():
            return 'pong'

        return text


class SocketTestNode(Node):
    def __init__(self):
        super().__init__('socket_test')
        self.declare_parameter('host', '192.168.4.1')
        self.declare_parameter('port', 3333)
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        host = self.get_parameter('host').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        self.get_logger().info(f'Attempting socket connect to {host}:{port}')
        try:
            response = send_ping(host, port)
            self.get_logger().info(f'Ping response: "{response}"')
        except Exception as exc:
            self.get_logger().error(f'Ping failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = SocketTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
