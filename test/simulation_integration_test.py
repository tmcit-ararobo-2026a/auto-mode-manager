"""Real loopback wire, quality monitor, safety process, planner, action and shot integration."""
import os
import re
import signal
import socket
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from robot_port.msg import RobotCommand, RobotOperation, BridgeHealth
from auto_mode_manager.msg import AutoModeStatus, SafetyState
from gn10_pointcloud_localization.msg import LocalizationStatus
from shooter_calculator.msg import ShotSolution
from std_msgs.msg import String
from std_srvs.srv import Trigger


def test_fixed_navigation_tracking_and_faults(tmp_path):
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
    node = Node('stm32_test_observer', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    statuses, operations, healths, qualities, safeties, shots = [], [], [], [], [], []
    for message_type, topic, values in (
            (AutoModeStatus, '/auto_mode/status', statuses), (RobotOperation, '/robot/operation', operations),
            (BridgeHealth, '/robot/communication_status', healths), (SafetyState, '/auto_mode/safety', safeties),
            (LocalizationStatus, '/localization/status', qualities), (ShotSolution, '/shooter/solution', shots)):
        node.create_subscription(message_type, topic, values.append, 100)
    commands = node.create_publisher(RobotCommand, '/stm32_test/command', 10)
    faults = node.create_publisher(String, '/stm32_test/fault', 10)
    reset = node.create_client(Trigger, '/auto_mode/reset_fault')
    log_path = tmp_path / 'simulation.log'
    log = log_path.open('wb')
    process = subprocess.Popen(['ros2', 'launch', os.environ['TEST_LAUNCH_FILE'], 'stage:=tracking_shot',
                                f'domain_id:={domain}', f'command_port:={ports[0]}',
                                f'operation_port:={ports[1]}'], stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
    desired = RobotCommand()
    desired.navigation_command, desired.target_id = RobotCommand.MOVE, 1
    fault = String()
    stopped_pids = []

    def spin():
        assert process.poll() is None, log_path.read_text(errors='replace')[-6000:]
        commands.publish(desired)
        faults.publish(fault)
        executor.spin_once(timeout_sec=.01)

    def wait(predicate, timeout=8):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            spin()
            if predicate():
                return
        raise AssertionError('Integration outcome timeout; state=' + repr(statuses[-1] if statuses else None) + '\n' + log_path.read_text(errors='replace')[-6000:])

    def duration(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            spin()

    def release():
        desired.navigation_command = RobotCommand.STANDBY
        duration(.2)
        wait(lambda: statuses and statuses[-1].navigation_status == AutoModeStatus.READY)

    def clear_fault():
        fault.data = ''
        desired.navigation_command = RobotCommand.STANDBY
        duration(.4)
        assert statuses[-1].navigation_status == AutoModeStatus.FAIL
        future = reset.call_async(Trigger.Request())
        wait(future.done)
        assert future.result().success, future.result().message
        wait(lambda: statuses[-1].navigation_status == AutoModeStatus.READY)

    def zero():
        return operations and all(v == 0 for v in (operations[-1].velocity_x_m_s,
                operations[-1].velocity_y_m_s, operations[-1].velocity_yaw_rad_s))

    try:
        wait(lambda: qualities and qualities[-1].ready_quality_valid)
        duration(.3)
        assert statuses[-1].navigation_status == AutoModeStatus.STANDBY
        assert zero() and not statuses[-1].arm_enabled
        release()
        generation = statuses[-1].generation
        desired.navigation_command = RobotCommand.MOVE
        wait(lambda: statuses[-1].navigation_status == AutoModeStatus.MOVING)
        wait(lambda: operations[-1].velocity_x_m_s > 0)
        wait(lambda: statuses[-1].navigation_status == AutoModeStatus.GOAL)
        completed_generation = statuses[-1].generation
        assert completed_generation > generation
        wait(lambda: shots and shots[-1].valid and operations[-1].belt_launcher_speed_m_s > 1)
        duration(.4)
        assert statuses[-1].navigation_status == AutoModeStatus.GOAL
        assert statuses[-1].generation == completed_generation
        assert not statuses[-1].arm_enabled
        assert statuses[-1].board_applied == AutoModeStatus.UNKNOWN
        assert statuses[-1].physical_stop_confirmed == AutoModeStatus.UNKNOWN
        # Loss is latched; quality/communication recovery cannot restart the held Move.
        fault.data = 'stop_feedback'
        wait(lambda: statuses[-1].navigation_status == AutoModeStatus.FAIL and zero())
        clear_fault()
        desired.navigation_command = RobotCommand.MOVE
        wait(lambda: statuses[-1].navigation_status == AutoModeStatus.GOAL)
        assert statuses[-1].generation > completed_generation
        # Goal departure invalidates generation without starting a hand or old path.
        departed_generation = statuses[-1].generation
        fault.data = 'goal_departure'
        duration(.03)
        fault.data = ''
        wait(lambda: statuses[-1].generation > departed_generation and statuses[-1].navigation_status == AutoModeStatus.READY)
        assert zero() and not statuses[-1].arm_enabled
        release()
        desired.target_id, desired.navigation_command = 7, RobotCommand.MOVE
        wait(lambda: statuses[-1].navigation_status == AutoModeStatus.TRACKING)
        wait(lambda: operations[-1].velocity_yaw_rad_s > 0)
        assert operations[-1].velocity_x_m_s == 0 and operations[-1].velocity_y_m_s == 0
        assert operations[-1].belt_launcher_speed_m_s == 0  # Moving compensation unavailable.
        desired.navigation_command = RobotCommand.STANDBY
        duration(.08)
        desired.navigation_command = RobotCommand.MOVE
        duration(.2)
        assert statuses[-1].navigation_status != AutoModeStatus.TRACKING and zero()
        release()
        desired.navigation_command = RobotCommand.MOVE
        wait(lambda: statuses[-1].navigation_status == AutoModeStatus.TRACKING)
        fault.data = 'stop_track'
        wait(lambda: statuses[-1].navigation_status == AutoModeStatus.FAIL and zero())
        clear_fault()
        # Independent monitor processing stalls cannot keep permitting output.
        desired.navigation_command = RobotCommand.MOVE
        wait(lambda: statuses[-1].navigation_status == AutoModeStatus.TRACKING)
        text = log_path.read_text(errors='replace')
        match = re.search(r'\[navigation_safety_monitor.py-\d+\]: process started with pid \[(\d+)\]', text)
        assert match, text[-4000:]
        safety_pid = int(match.group(1))
        os.kill(safety_pid, signal.SIGSTOP)
        stopped_pids.append(safety_pid)
        wait(lambda: statuses[-1].navigation_status == AutoModeStatus.FAIL and zero(), timeout=2)
        os.kill(safety_pid, signal.SIGCONT)
        stopped_pids.remove(safety_pid)
        clear_fault()
        # A stopped follower cannot acknowledge cancellation; the 200ms limit latches Fail.
        desired.target_id, desired.navigation_command = 1, RobotCommand.MOVE
        wait(lambda: statuses[-1].execution_phase == 'following')
        text = log_path.read_text(errors='replace')
        match = re.search(r'\[path_follower_node-\d+\]: process started with pid \[(\d+)\]', text)
        assert match
        follower_pid = int(match.group(1))
        os.kill(follower_pid, signal.SIGSTOP)
        stopped_pids.append(follower_pid)
        started = time.monotonic()
        desired.navigation_command = RobotCommand.STANDBY
        wait(lambda: statuses[-1].navigation_status == AutoModeStatus.FAIL and
             '200ms' in statuses[-1].reason, timeout=2)
        assert time.monotonic() - started >= .18
        assert zero() and statuses[-1].cancellation_pending
        os.kill(follower_pid, signal.SIGCONT)
        stopped_pids.remove(follower_pid)
        wait(lambda: not statuses[-1].cancellation_pending)
        clear_fault()
        # Costmap loss also revokes the independent assessment.
        fault.data = 'stop_map'
        wait(lambda: statuses[-1].navigation_status == AutoModeStatus.FAIL and zero())
        clear_fault()
        for failure in ('stop_obstacles', 'obstacle_point', 'profile_changed', 'invalid_mechanism'):
            fault.data = failure
            wait(lambda: statuses[-1].navigation_status == AutoModeStatus.FAIL and zero())
            clear_fault()
        assert all(not status.arm_enabled for status in statuses)
        assert healths[-1].test_mode and healths[-1].loopback_only
    finally:
        for pid in stopped_pids:
            os.kill(pid, signal.SIGCONT)
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
