"""Verify the eligible communication launch and fixture isolation rules."""

import os
import signal
import socket
import subprocess
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from robot_port.msg import BridgeHealth, RobotCommand, RobotFeedback


def test_communication_stage():
    """Observe exact communication via loopback and reject unimplemented stages."""
    domain = 150 + os.getpid() % 60
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    ports = []
    for _ in range(2):
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(('127.0.0.2', 0))
        ports.append(probe.getsockname()[1])
        probe.close()
    assert ports[0] != ports[1]
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = Node('stm32_test_observer', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    healths, commands, feedbacks = [], [], []
    node.create_subscription(BridgeHealth, '/robot/communication_status', healths.append, 10)
    node.create_subscription(RobotCommand, '/robot/command', commands.append, 10)
    node.create_subscription(RobotFeedback, '/robot/feedback', feedbacks.append, 10)
    base = ['ros2', 'launch', os.environ['TEST_LAUNCH_FILE'], f'domain_id:={domain}',
            f'command_port:={ports[0]}', f'operation_port:={ports[1]}']
    process = subprocess.Popen(base, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               start_new_session=True)
    try:
        end = time.monotonic() + 10
        while time.monotonic() < end:
            assert process.poll() is None
            executor.spin_once(timeout_sec=0.02)
            if (healths and commands and feedbacks and healths[-1].operation_fresh
                    and healths[-1].command_fresh and healths[-1].feedback_fresh):
                break
        assert healths and commands and feedbacks
        assert healths[-1].operation_fresh and healths[-1].last_send_success
        assert not healths[-1].watchdog_active
        assert commands[-1].navigation_command == RobotCommand.SLEEP
        assert healths[-1].send_errors == 0 and healths[-1].receive_errors == 0
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            output = process.communicate(timeout=5)[0]
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output = process.communicate(timeout=5)[0]
        if not healths:
            print(output.decode(errors='replace'))
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
    invalid = subprocess.run(base + ['stage:=hand'], capture_output=True, timeout=5)
    assert invalid.returncode != 0
    assert b'Eligible stages' in invalid.stdout + invalid.stderr
    # Fixture cannot be invoked in a production/default domain or without explicit test mode.
    environment = dict(os.environ, ROS_DOMAIN_ID='0')
    invalid = subprocess.run([os.environ['FIXTURE_EXECUTABLE']], env=environment,
                             capture_output=True, timeout=5)
    assert invalid.returncode != 0
