"""Verify independent watchdog progress when ROS time freezes or changes epoch."""
import os
import signal
import socket
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rosgraph_msgs.msg import Clock
from robot_port.msg import RobotCommand, RobotOperation
from auto_mode_manager.msg import AutoModeStatus
from std_srvs.srv import Trigger


def test_frozen_clock_and_rollback(tmp_path):
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    domain = 150 + os.getpid() % 60
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    ports = []
    for _ in range(2):
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(('127.0.0.2', 0))
        ports.append(probe.getsockname()[1])
        probe.close()
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = Node('stm32_test_clock_observer', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    clocks = node.create_publisher(Clock, '/clock', 10)
    commands = node.create_publisher(RobotCommand, '/stm32_test/command', 10)
    reset = node.create_client(Trigger, '/auto_mode/reset_fault')
    statuses, operations = [], []
    node.create_subscription(AutoModeStatus, '/auto_mode/status', statuses.append, 100)
    node.create_subscription(RobotOperation, '/robot/operation', operations.append, 100)
    log_path = tmp_path / 'clock.log'
    log = log_path.open('wb')
    process = subprocess.Popen(['ros2', 'launch', os.environ['TEST_LAUNCH_FILE'], 'stage:=tracking_shot',
                                'use_sim_time:=true', f'domain_id:={domain}',
                                f'command_port:={ports[0]}', f'operation_port:={ports[1]}'],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    started_s = time.monotonic()
    clock_offset_s = 100.
    advancing = True
    command = RobotCommand()
    command.navigation_command, command.target_id = RobotCommand.STANDBY, 7

    def spin():
        assert process.poll() is None, log_path.read_text(errors='replace')[-5000:]
        if advancing:
            clock = Clock()
            nanoseconds = int((clock_offset_s + time.monotonic() - started_s) * 1e9)
            clock.clock.sec, clock.clock.nanosec = divmod(nanoseconds, 1000000000)
            clocks.publish(clock)
        commands.publish(command)
        executor.spin_once(timeout_sec=.01)

    def wait(predicate, timeout=8):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            spin()
            if predicate():
                return
        raise AssertionError('Clock state timeout: ' + repr(statuses[-1] if statuses else None) + '\n' + log_path.read_text(errors='replace')[-4000:])

    def duration(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            spin()

    def recover():
        command.navigation_command = RobotCommand.STANDBY
        duration(1.4)
        assert statuses[-1].navigation_status == AutoModeStatus.FAIL
        response = reset.call_async(Trigger.Request())
        wait(response.done)
        assert response.result().success, response.result().message
        wait(lambda: statuses[-1].navigation_status == AutoModeStatus.READY)

    try:
        wait(lambda: statuses and statuses[-1].navigation_status == AutoModeStatus.READY)
        command.navigation_command = RobotCommand.MOVE
        wait(lambda: statuses[-1].navigation_status == AutoModeStatus.TRACKING)
        wait(lambda: operations and operations[-1].velocity_yaw_rad_s > 0)
        advancing = False
        wait(lambda: statuses[-1].navigation_status == AutoModeStatus.FAIL, timeout=2)
        assert operations[-1].velocity_yaw_rad_s == 0
        advancing = True
        recover()
        command.navigation_command = RobotCommand.MOVE
        wait(lambda: statuses[-1].navigation_status == AutoModeStatus.TRACKING)
        previous_session = statuses[-1].producer_session
        clock_offset_s -= 1.
        wait(lambda: statuses[-1].navigation_status == AutoModeStatus.FAIL and
             statuses[-1].producer_session != previous_session)
        assert not statuses[-1].arm_enabled
        recover()
        assert statuses[-1].generation == 1
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
        log.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
