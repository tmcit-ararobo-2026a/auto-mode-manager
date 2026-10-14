"""Verify the standalone foundation stays inactive and does not publish operations."""

import os
import subprocess
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from auto_mode_manager.msg import AutoModeStatus


def test_manager_standby_and_no_output():
    """Observe advancing status while rejecting any implied active control or safety."""
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = Node('manager_foundation_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    statuses = []
    node.create_subscription(AutoModeStatus, '/auto_mode/status', statuses.append, 10)
    process = subprocess.Popen([os.environ['MANAGER_EXECUTABLE']],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        end = time.monotonic() + 5
        while len(statuses) < 5 and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
        assert len(statuses) >= 5
        assert process.poll() is None
        for previous, current in zip(statuses, statuses[1:]):
            assert current.navigation_status == AutoModeStatus.STANDBY
            assert current.producer_session == previous.producer_session != 0
            assert current.update_sequence > previous.update_sequence
            assert not current.automatic_output_enabled and not current.contracts_confirmed
        assert node.count_publishers('/robot/operation') == 0
        assert node.count_publishers('/auto_mode/safety_state') == 0
    finally:
        process.terminate()
        try:
            process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate(timeout=5)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
