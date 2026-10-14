#!/usr/bin/env python3
"""Loopback-only communication fixture; never connects to a physical mainboard."""

import ipaddress
import os
import random
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from robot_port.msg import RobotOperation


class MockMainboardFixture(Node):
    """Exercise new command/feedback sockets and neutral Operation publication."""

    def __init__(self):
        """Require explicit test mode, localhost DDS and a reserved test domain."""
        super().__init__('stm32_test_mainboard')
        if not self.declare_parameter('test_mode', False).value:
            raise ValueError('Explicit test_mode is required')
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise ValueError('Fixture requires ROS_LOCALHOST_ONLY=1')
        domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
        if domain < 100 or domain > 232:
            raise ValueError('Fixture requires a dedicated test domain in [100,232]')
        local = self.declare_parameter('local_address', '127.0.0.2').value
        peer = self.declare_parameter('peer_address', '127.0.0.1').value
        if not ipaddress.IPv4Address(local).is_loopback or not ipaddress.IPv4Address(peer).is_loopback:
            raise ValueError('Fixture endpoints must both be loopback')
        command_port = self.declare_parameter('command_port', 26574).value
        operation_port = self.declare_parameter('operation_port', 39244).value
        if not 1 <= command_port <= 65535 or not 1 <= operation_port <= 65535 or command_port == operation_port:
            raise ValueError('Fixture requires distinct nonzero UDP ports')
        self.command_destination = (peer, command_port)
        self.operation_destination = (peer, operation_port)
        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.operation_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_socket.bind((local, command_port))
        self.operation_socket.bind((local, operation_port))
        self.operation_socket.setblocking(False)
        self.publisher = self.create_publisher(RobotOperation, '/robot/operation', 1)
        self.started = time.monotonic()
        self.producer_session = random.getrandbits(64)
        if self.producer_session == 0:
            self.producer_session = 1
        self.sequence = 0
        self.update_sequence = 0
        self.timer = self.create_timer(0.02, self.tick)

    def tick(self):
        """Send exact neutral packets after discovery and reject mixed production graphs."""
        if time.monotonic() - self.started < 1.0:
            return
        names = self.get_node_names()
        if any(not name.startswith(('stm32_test_', '_ros2cli')) for name in names):
            raise RuntimeError('Refusing fixture in a graph containing non-test nodes')
        if self.count_publishers('/robot/operation') != 1:
            raise RuntimeError('Refusing another Operation producer in the fixture graph')
        self.sequence = (self.sequence + 1) % 256
        # sequence remains opaque; this fixture does not define real request semantics.
        command = bytes([0x38, self.sequence, 0, 1, 0, 0, 0, 0])
        feedback = struct.pack('<4B10f', 0x55, self.sequence, 0, 0, *([0.0] * 10))
        self.command_socket.sendto(command, self.command_destination)
        self.operation_socket.sendto(feedback, self.operation_destination)
        while True:
            try:
                packet, source = self.operation_socket.recvfrom(65536)
            except BlockingIOError:
                break
            if source != self.operation_destination or len(packet) != 32 or packet[0] != 0xAB:
                raise RuntimeError('Unexpected operation source/header/size')
            if any(value != 0 for value in struct.unpack_from('<5f', packet, 4)):
                raise RuntimeError('Communication-only fixture must not receive motion/mechanism commands')
            if packet[2] != 0 or packet[3] not in (0, 5) or any(packet[24:]):
                raise RuntimeError('Communication-only fixture requires neutral target, status and mechanisms')
        if self.publisher.get_subscription_count() == 1:
            self.update_sequence += 1
            operation = RobotOperation()
            operation.stamp = self.get_clock().now().to_msg()
            operation.producer_session = self.producer_session
            operation.update_sequence = self.update_sequence
            operation.navigation_status = RobotOperation.STANDBY
            self.publisher.publish(operation)

    def close(self):
        """Release exclusively bound test ports."""
        self.command_socket.close()
        self.operation_socket.close()


def main():
    """Run only an explicitly isolated test fixture."""
    rclpy.init()
    node = None
    try:
        node = MockMainboardFixture()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
