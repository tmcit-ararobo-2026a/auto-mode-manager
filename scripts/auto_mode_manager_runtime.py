#!/usr/bin/env python3
"""Contract-v1 coordinator; physical profiles remain unavailable pending E1-E5."""
import copy
import math
import time
from collections import deque
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from gn10_navigation.srv import ComputePath
from gn10_navigation.action import FollowPath
from gn10_navigation.msg import ControlLease, PathVelocity
from gn10_pointcloud_localization.msg import LocalizationStatus
from lidar_perception_system.msg import MovingBucketTrack
from field_target_pub.msg import TargetSet
from shooter_calculator.msg import AimTarget, ShotSolution
from robot_port.msg import RobotCommand, RobotFeedback, RobotOperation, BridgeHealth
from auto_mode_manager.msg import AutoModeStatus, SafetyState, ExecutionPath, SimulationState
from contract_core import RequestState, Freshness, STANDBY, READY, MOVING, TRACKING, GOAL, FAIL, wrap, wire_angle
from tracking_controller import yaw_velocity
from runtime_common import load_profile, session_id, stamp_ns, fresh_stamp, pose_values


@dataclass
class Execution:
    """Keep unresolved goal acceptance until its handle has been canceled and terminated."""
    generation: int
    target_id: int
    started_s: float
    plan: object = None
    goal_handle: object = None
    goal_pending: bool = False
    terminal: bool = True
    cleared: bool = True
    cancelling: bool = False
    cancel_requested: bool = False
    phase: str = 'planning'
    permit_started_s: float = 0.0
    received_velocity_s: float = 0.0


class AutoModeManager(Node):
    """Own the sole Operation and execution state; callbacks can revoke immediately."""

    def __init__(self):
        super().__init__('auto_mode_manager')
        self.session, self.core = session_id(), RequestState()
        self.sequence = self.lease_sequence = self.status_sequence = 0
        self.output_enabled = False
        self.mode = self.declare_parameter('mode', 'disabled').value
        self.profile = None
        if self.mode == 'simulation':
            self.profile = load_profile(self.declare_parameter('profile_file', '').value)
        elif self.mode != 'disabled':
            raise ValueError('navigation_hil/full_robot unavailable: E1-E5 evidence missing')
        self.status_pub = self.create_publisher(AutoModeStatus, '/auto_mode/status', 10)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02, self.tick, clock=self.steady_clock)
        self.bridge = self.localization = self.safety = self.targets = self.track = self.shot = self.simulation = None
        self.command = self.feedback = self.velocity = self.execution = None
        self.events = deque()
        self.command_stamp = self.feedback_stamp = 0
        self.command_deadline = self.feedback_deadline = 0.0
        self.last_ros = 0
        self.ever_communicated = False
        self.return_ready_after_cancel = False
        self.restart_session_after_cancel = False
        self.streams = {name: Freshness(limit) for name, limit in
                        (('bridge', .1), ('localization', .1), ('safety', .1), ('simulation', .1),
                         ('velocity', .1), ('shot', .15), ('targets', .15), ('track', .1))}
        self.last_twist = Twist()
        self.yaw_rate = 0.0
        self.track_id = None
        self.angle_valid = [False] * 6
        if self.profile is None:
            return
        self.operation_pub = self.create_publisher(RobotOperation, '/robot/operation', 1)
        self.lease_pub = self.create_publisher(ControlLease, '/navigation/control_lease', 10)
        self.path_pub = self.create_publisher(ExecutionPath, '/auto_mode/execution_path', 10)
        self.aim_pub = self.create_publisher(AimTarget, '/auto_mode/aim_target', 1)
        self.planner = self.create_client(ComputePath, '/navigation/compute_path')
        self.follower = ActionClient(self, FollowPath, '/navigation/follow_path')
        self.create_service(Trigger, '/auto_mode/reset_fault', self.reset_fault)
        self.create_subscription(RobotCommand, '/robot/command', self.observe_command, 100)
        self.create_subscription(RobotFeedback, '/robot/feedback', self.observe_feedback, 10)
        for message_type, topic, callback in (
                (BridgeHealth, '/robot/communication_status', self.observe_bridge),
                (LocalizationStatus, '/localization/status', self.observe_localization),
                (SafetyState, '/auto_mode/safety', self.observe_safety),
                (SimulationState, '/auto_mode/simulation_state', self.observe_simulation),
                (TargetSet, '/field/targets', self.observe_targets),
                (MovingBucketTrack, '/moving_bucket_detector/target', self.observe_track),
                (PathVelocity, '/navigation/path_velocity', self.observe_velocity),
                (ShotSolution, '/shooter/solution', self.observe_shot)):
            self.create_subscription(message_type, topic, callback, 10)

    def clocks(self):
        return self.get_clock().now().nanoseconds, time.monotonic()

    def accept(self, key, session, sequence, stamp):
        ros_ns, steady_s = self.clocks()
        accepted, changed = self.streams[key].accept(session, sequence, stamp_ns(stamp), ros_ns, steady_s)
        if changed and key in ('bridge', 'localization', 'safety', 'simulation'):
            self.inhibit(key + ' process restarted', fault=True)
        return accepted

    def observe_bridge(self, message):
        if self.accept('bridge', message.bridge_session, message.update_sequence, message.stamp):
            self.bridge = message
            if not message.test_mode or not message.loopback_only:
                self.inhibit('Simulation refuses a physical bridge', fault=True)

    def observe_localization(self, message):
        if self.accept('localization', message.monitor_session, message.report_sequence, message.header.stamp):
            self.localization = message
            if not self.quality_ok() and self.core.state in (READY, MOVING, TRACKING, GOAL):
                self.inhibit('Localization quality lost', fault=True)

    def observe_safety(self, message):
        if self.accept('safety', message.monitor_session, message.evaluation_sequence, message.evaluated_stamp):
            self.safety = message
            if not message.safe and self.core.state in (READY, MOVING, TRACKING, GOAL):
                self.inhibit(message.reason, fault=True)

    def observe_simulation(self, message):
        if self.accept('simulation', message.fixture_session, message.update_sequence, message.stamp):
            self.simulation = message
            if self.core.state in (READY, MOVING, TRACKING, GOAL) and not self.mechanisms_ok():
                self.inhibit('Synthetic drive/disabled mechanisms invalid', fault=True)

    def observe_targets(self, message):
        if self.accept('targets', 1, stamp_ns(message.header.stamp), message.header.stamp):
            self.targets = message
            if not self.targets_ok() and self.core.state in (READY, MOVING, TRACKING, GOAL):
                self.inhibit('Target profile changed or invalid', fault=True)

    def observe_track(self, message):
        # Only a genuinely advancing target state can renew its state lease.
        if self.accept('track', 1, stamp_ns(message.header.stamp), message.header.stamp):
            self.track = message
            if self.core.state == TRACKING and not self.track_ok():
                self.inhibit('Track invalid, changed or lost', fault=True)

    def observe_shot(self, message):
        if message.producer_session != self.session or message.generation != self.core.generation or message.target_id != self.core.target_id:
            return
        if self.accept('shot', message.producer_session, stamp_ns(message.header.stamp), message.header.stamp):
            self.shot = message
            if not message.valid:
                self.shot = None  # Do not retain a previously valid belt speed.

    def observe_velocity(self, message):
        if self.accept('velocity', message.producer_session, message.update_sequence, message.stamp):
            self.velocity = message
            execution = self.execution
            if (execution and message.producer_session == self.session and message.generation == execution.generation):
                execution.received_velocity_s = time.monotonic()
                if (message.control_history_cleared and not message.valid and
                        all(value == 0 for value in (message.velocity.linear.x, message.velocity.linear.y,
                                                     message.velocity.linear.z, message.velocity.angular.x,
                                                     message.velocity.angular.y, message.velocity.angular.z))):
                    execution.cleared = True

    def observe_feedback(self, message):
        ros_ns, steady_s = self.clocks()
        stamp = stamp_ns(message.received_stamp)
        if stamp <= self.feedback_stamp or not fresh_stamp(message.received_stamp, ros_ns, .15):
            return
        self.feedback_stamp = stamp
        self.feedback_deadline = steady_s + .15 - (ros_ns - stamp) * 1e-9
        self.feedback = message
        if message.emergency_stop_enabled or message.over_current:
            self.inhibit('Emergency stop or overcurrent', fault=True)

    def observe_command(self, message):
        ros_ns, steady_s = self.clocks()
        stamp = stamp_ns(message.received_stamp)
        if not fresh_stamp(message.received_stamp, ros_ns, .15):
            return
        if stamp <= self.command_stamp:
            if message.navigation_command in (RobotCommand.SLEEP, RobotCommand.STANDBY):
                self.inhibit('Reordered ROS cancellation', fault=True)
            return
        self.command_stamp = stamp
        self.command_deadline = steady_s + .15 - (ros_ns - stamp) * 1e-9
        self.command = message
        was_cancelling = self.core.cancelling
        accepted = self.core.observe(message.navigation_command, message.target_id, steady_s,
                                     self.capable(message.target_id))
        if self.core.cancelling and not was_cancelling:
            self.revoke_outputs()
        if self.core.overflow:
            self.revoke_outputs()
            if self.execution is None:
                self.session, self.core = session_id(), RequestState()
                self.command = self.feedback = None
                self.ever_communicated = False
            return
        if accepted:
            self.execution = Execution(self.core.generation, message.target_id, steady_s)
            self.velocity = self.shot = None
            self.track_id = None
            self.yaw_rate = 0.0
            if message.target_id == 7:
                self.track_id = self.track.track_id
                self.execution.phase = 'tracking'
            else:
                self.plan_path(self.execution)
        # shutdown/logging/cloth_collect are notifications only; no side effects.

    def quality_ok(self):
        ros_ns, steady_s = self.clocks()
        message = self.localization
        if not message or not self.streams['localization'].fresh(ros_ns, steady_s):
            return False
        if (not message.simulation_profile or not message.ready_quality_valid or
                not message.pose_valid or not message.pose_tf_valid or not message.last_match_success or
                message.lost or message.using_fallback_map or not message.map_loaded or
                ros_ns >= stamp_ns(message.quality_valid_until) or
                stamp_ns(message.pose_stamp) != stamp_ns(message.measurement_stamp) or
                message.pose.header.frame_id != self.profile['map_frame'] or
                pose_values(message.pose.pose) is None):
            return False
        return all(getattr(message, key) == self.profile[key] for key in
                   ('quality_id', 'map_id', 'team', 'extrinsic_id', 'implementation_id', 'configuration_id'))

    def mechanisms_ok(self):
        ros_ns, steady_s = self.clocks()
        message = self.simulation
        return bool(message and self.streams['simulation'].fresh(ros_ns, steady_s) and
                    message.profile_id == self.profile['profile_id'] and message.mock_drive_ready and
                    message.mock_hand_disabled and message.mock_launcher_disabled)

    def communication_ok(self):
        ros_ns, steady_s = self.clocks()
        return bool(self.bridge and self.streams['bridge'].fresh(ros_ns, steady_s) and
                    self.bridge.test_mode and self.bridge.loopback_only and self.bridge.transmission_enabled and
                    self.bridge.command_fresh and self.bridge.feedback_fresh and self.command and self.feedback and
                    steady_s < self.command_deadline and steady_s < self.feedback_deadline and
                    fresh_stamp(self.command.received_stamp, ros_ns, .15) and
                    fresh_stamp(self.feedback.received_stamp, ros_ns, .15) and
                    not self.feedback.emergency_stop_enabled and not self.feedback.over_current)

    def safety_ok(self, generation=None, path=False):
        ros_ns, steady_s = self.clocks()
        message = self.safety
        return bool(message and self.streams['safety'].fresh(ros_ns, steady_s) and
                    message.valid and message.safe and message.simulation_profile and
                    message.profile_id == self.profile['profile_id'] and message.manager_session == self.session and
                    ros_ns < stamp_ns(message.valid_until) and
                    (generation is None or message.generation == generation) and (not path or message.path_checked))

    def targets_ok(self):
        ros_ns, steady_s = self.clocks()
        message = self.targets
        if (not message or not self.streams['targets'].fresh(ros_ns, steady_s) or not message.valid or
                message.header.frame_id != self.profile['map_frame'] or message.map_id != self.profile['map_id'] or
                message.team != self.profile['team'] or message.revision != self.profile['target_revision']):
            return False
        ids = set()
        for target in message.targets:
            if target.target_id in ids or target.target_id not in range(7):
                return False
            ids.add(target.target_id)
            if (not target.stop_pose_valid or target.stop_pose.header.frame_id != self.profile['map_frame'] or
                    pose_values(target.stop_pose.pose) is None):
                return False
            if target.target_id != 0 and (not target.aim_valid or
                    target.aim_point.header.frame_id != self.profile['map_frame'] or
                    not all(math.isfinite(v) for v in (target.aim_point.point.x, target.aim_point.point.y, target.aim_point.point.z))):
                return False
        return ids == set(range(7))

    def track_ok(self):
        ros_ns, steady_s = self.clocks()
        message = self.track
        return bool(message and self.streams['track'].fresh(ros_ns, steady_s) and message.valid and
                    message.header.frame_id == self.profile['map_frame'] and message.observation_mode in (1, 2, 3) and
                    fresh_stamp(message.last_observation_stamp, ros_ns, self.profile['track_timeout_s']) and
                    (self.track_id is None or message.track_id == self.track_id) and
                    all(math.isfinite(x) for x in (message.position.x, message.position.y, message.position.z,
                                                   message.velocity.x, message.velocity.y, message.velocity.z,
                                                   message.confidence, *message.state_covariance)) and
                    0 <= message.confidence <= 1 and all(message.state_covariance[i] >= 0 for i in (0, 7, 14)))

    def capable(self, target_id):
        if not self.healthy():
            return False
        if target_id == 7:
            return self.track_ok()
        return self.targets_ok() and self.planner.service_is_ready() and self.follower.server_is_ready()

    def healthy(self):
        return self.communication_ok() and self.quality_ok() and self.mechanisms_ok() and self.safety_ok()

    def reset_fault(self, request, response):
        response.success, response.message = self.core.reset(time.monotonic(), self.healthy())
        return response

    def revoke_outputs(self):
        """Revoke synchronously; old execution metadata is retained for follower cleanup."""
        self.last_twist = Twist()
        self.output_enabled = False
        self.yaw_rate = 0.0
        self.shot = None
        if self.execution:
            self.execution.cancelling = True
            self.execution.phase = 'cancelling'
            self.cancel_goal(self.execution)
        self.publish_lease(False)
        self.publish_operation(Twist())
        self.publish_aim(None)

    def inhibit(self, reason, fault=False, sleep=False):
        if self.core.state == FAIL and not self.core.cancelling and self.execution is None and not self.restart_session_after_cancel:
            self.core.reason = reason
            return
        self.core.cancel(time.monotonic(), reason, sleep=sleep, fault=fault)
        self.revoke_outputs()

    def cancel_goal(self, execution):
        if execution.goal_handle and not execution.terminal and not execution.cancel_requested:
            execution.cancel_requested = True
            execution.goal_handle.cancel_goal_async()
        # Pending acceptance remains pending; its eventual handle is canceled below.

    def plan_path(self, execution):
        request = ComputePath.Request()
        request.start = copy.deepcopy(self.localization.pose)
        request.goal = copy.deepcopy(next(t.stop_pose for t in self.targets.targets if t.target_id == execution.target_id))
        request.generation = execution.generation
        self.planner.call_async(request).add_done_callback(lambda future: self.events.append(('plan', execution, future)))

    def start_following(self, execution):
        goal = FollowPath.Goal()
        goal.path, goal.producer_session, goal.generation = execution.plan, self.session, execution.generation
        execution.goal_pending, execution.terminal, execution.cleared = True, False, False
        execution.phase = 'awaiting_goal'
        self.follower.send_goal_async(goal).add_done_callback(lambda future: self.events.append(('accepted', execution, future)))

    def process_events(self):
        """Arrival is lower priority than latched cancellation and current fault checks."""
        while self.events:
            event, execution, future = self.events.popleft()
            try:
                result = future.result()
            except Exception as error:
                if event == 'accepted':
                    execution.goal_pending = False
                self.inhibit('Async operation failed: ' + str(error), fault=True)
                continue
            current = execution is self.execution and execution.generation == self.core.generation and not self.core.cancelling
            if event == 'plan':
                if not current:
                    continue
                if not result.success or result.generation != execution.generation or not result.path.poses:
                    self.inhibit('Path planning failed: ' + result.reason, fault=True)
                    continue
                execution.plan, execution.phase = result.path, 'awaiting_safety'
            elif event == 'accepted':
                execution.goal_pending = False
                if not result.accepted:
                    execution.terminal, execution.cleared = True, True
                    if current:
                        self.inhibit('FollowPath goal rejected', fault=True)
                    continue
                execution.goal_handle = result
                result.get_result_async().add_done_callback(lambda done, context=execution: self.events.append(('result', context, done)))
                if current:
                    execution.phase = 'following'
                    execution.received_velocity_s = time.monotonic()
                else:
                    self.cancel_goal(execution)
            elif event == 'result':
                execution.terminal = True
                if not current:
                    continue
                if (result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success and
                        result.result.generation == execution.generation):
                    # Arrival remains provisional until zero/history-clear is observed.
                    execution.phase = 'arrived'
                else:
                    self.inhibit('Follower terminated: ' + result.result.reason, fault=True)

    def publish_lease(self, enabled):
        if self.profile is None:
            return
        message = ControlLease()
        message.stamp = self.get_clock().now().to_msg()
        message.producer_session, message.update_sequence = self.session, self.lease_sequence + 1
        self.lease_sequence += 1
        message.generation = self.core.generation
        if self.core.cancelling:
            message.generation = self.core.cancel_generation
        message.enabled = enabled
        self.lease_pub.publish(message)

    def publish_path(self):
        message = ExecutionPath()
        message.stamp = self.get_clock().now().to_msg()
        message.producer_session, message.generation = self.session, self.core.generation
        if self.execution and self.execution.plan and not self.core.cancelling:
            message.valid, message.path = True, self.execution.plan
        self.path_pub.publish(message)

    def aim_point(self):
        if self.core.state == TRACKING and self.track_ok():
            return self.track.position
        if self.core.state in (MOVING, GOAL) and self.targets_ok() and self.core.target_id != 0:
            return next(t.aim_point.point for t in self.targets.targets if t.target_id == self.core.target_id)
        return None

    def publish_aim(self, point):
        message = AimTarget()
        message.header.stamp, message.header.frame_id = self.get_clock().now().to_msg(), self.profile['map_frame']
        message.producer_session, message.generation, message.target_id = self.session, self.core.generation, self.core.target_id
        if point is not None:
            message.valid, message.position, message.confidence = True, point, 1.0
            message.last_observation_stamp = message.header.stamp
            if self.core.state == TRACKING:
                message.predicted = self.track.observation_mode == MovingBucketTrack.PREDICTED
                message.last_observation_stamp = self.track.last_observation_stamp
                message.velocity_m_s = self.track.velocity
                message.position_covariance = [self.track.state_covariance[row * 6 + column] for row in range(3) for column in range(3)]
                message.confidence = self.track.confidence
        self.aim_pub.publish(message)

    def publish_operation(self, twist):
        """Emit loopback snapshots only; no physical acceptance/stop claims are inferred."""
        if self.profile is None or not self.bridge or not self.bridge.test_mode or not self.bridge.loopback_only:
            return
        message = RobotOperation()
        message.stamp = self.get_clock().now().to_msg()
        message.producer_session, message.generation = self.session, self.core.generation
        self.sequence += 1
        message.update_sequence = self.sequence
        message.navigation_status, message.target_id = self.core.state, self.core.target_id
        if self.core.state in (STANDBY, FAIL):
            message.target_id = 0
        message.velocity_x_m_s, message.velocity_y_m_s = twist.linear.x, twist.linear.y
        message.velocity_yaw_rad_s = twist.angular.z
        self.angle_valid = [False] * 6
        point = self.aim_point()
        if point is not None and self.quality_ok() and not self.core.cancelling:
            x, y, yaw = pose_values(self.localization.pose.pose)
            angle = wire_angle(wrap(math.atan2(point.y - y, point.x - x) - yaw), self.profile['angle_sign'], self.profile['angle_offset_deg'])
            index = {7: 0, 1: 1, 2: 2, 3: 3, 6: 4, 4: 5, 5: 5}.get(self.core.target_id)
            if angle is not None and index is not None:
                message.angles_deg[index], self.angle_valid[index] = angle, True
        ros_ns, steady_s = self.clocks()
        shot = self.shot
        if (shot and self.streams['shot'].fresh(ros_ns, steady_s) and shot.valid and shot.in_calibration_range and
                shot.producer_session == self.session and shot.generation == self.core.generation and
                shot.target_id == self.core.target_id and shot.calibration_id == self.profile['shot_calibration_id'] and
                self.core.state in (MOVING, GOAL) and not self.core.cancelling and self.quality_ok() and
                all(math.isfinite(v) and abs(v) <= 3.4e38 for v in (shot.initial_speed_m_s, shot.belt_speed_m_s))):
            message.belt_launcher_speed_m_s = shot.belt_speed_m_s
        # No arm sequence or physical trigger is available; zero is not a stop proof.
        self.operation_pub.publish(message)

    def tracking_twist(self):
        if not self.track_ok():
            self.inhibit('Track expired or ID changed', fault=True)
            return Twist()
        x, y, yaw = pose_values(self.localization.pose.pose)
        rate = yaw_velocity(x, y, yaw, self.track.position.x, self.track.position.y,
                            self.yaw_rate, self.profile['yaw_gain'], self.profile['max_yaw_speed_rad_s'],
                            self.profile['yaw_acceleration_rad_s2'], .02)
        if rate is None:
            self.inhibit('Track bearing undefined', fault=True)
            return Twist()
        self.yaw_rate = rate
        twist = Twist()
        twist.angular.z = self.yaw_rate
        return twist  # XY stays manual on the board; automatic XY is zero.

    def tick(self):
        if self.profile is None:
            self.publish_status(False)
            return
        ros_ns, steady_s = self.clocks()
        if self.last_ros and ros_ns < self.last_ros:
            self.restart_session_after_cancel = True
            self.inhibit('ROS clock rollback', fault=True)
            self.command_stamp = self.feedback_stamp = 0
            for key, stream in self.streams.items():
                stream.deadline_s = stream.stamp_ns = 0
                if key in ('targets', 'track', 'shot'):
                    stream.sequence = 0
        self.last_ros = ros_ns
        communication = self.communication_ok()
        if communication:
            self.ever_communicated = True
        elif self.ever_communicated:
            self.inhibit('Command, feedback or bridge expired', fault=True)
        if self.core.state in (READY, MOVING, TRACKING, GOAL) and not self.healthy():
            self.inhibit('Quality, safety evaluation or mechanism lease lost', fault=True)
        self.process_events()
        execution = self.execution
        if self.core.cancelling:
            if execution:
                self.cancel_goal(execution)
            complete = execution is None or (not execution.goal_pending and execution.terminal and execution.cleared)
            if complete:
                self.execution = None
                self.velocity = None
                self.core.cancel_complete()
                if self.return_ready_after_cancel and self.healthy() and self.core.state != FAIL:
                    self.core.state = READY
                    self.core.armed = False
                self.return_ready_after_cancel = False
                if self.restart_session_after_cancel:
                    self.session = session_id()
                    self.core = RequestState(state=FAIL, reason='Clock epoch changed; explicit reset required')
                    self.sequence = self.lease_sequence = self.status_sequence = 0
                    self.restart_session_after_cancel = False
                if self.core.overflow:
                    self.session, self.core = session_id(), RequestState()
                    self.command = self.feedback = None
                    self.ever_communicated = False
            elif steady_s - self.core.cancel_started_s >= .2:
                self.core.state, self.core.reason = FAIL, 'Cancellation completion exceeded 200ms'
        self.core.update_ready(steady_s, self.healthy())
        twist = Twist()
        permit = False
        execution = self.execution
        if execution and not self.core.cancelling and self.core.state == MOVING:
            safe_path = self.safety_ok(self.core.generation, path=True)
            if execution.phase in ('planning', 'awaiting_safety') and steady_s - execution.started_s > 1.0:
                self.inhibit('Planning/safety deadline exceeded', fault=True)
            elif execution.phase == 'awaiting_safety' and safe_path:
                permit = True
                if execution.permit_started_s == 0:
                    execution.permit_started_s = steady_s
                elif steady_s - execution.permit_started_s >= .04:
                    self.start_following(execution)
            elif execution.phase in ('awaiting_goal', 'following') and safe_path:
                permit = True
                if execution.phase == 'awaiting_goal' and steady_s - execution.permit_started_s > .5:
                    self.inhibit('Goal acceptance deadline exceeded', fault=True)
                elif execution.phase == 'following':
                    velocity = self.velocity
                    if velocity and velocity.producer_session == self.session and velocity.generation == self.core.generation and velocity.valid and self.streams['velocity'].fresh(ros_ns, steady_s):
                        values = (velocity.velocity.linear.x, velocity.velocity.linear.y, velocity.velocity.angular.z)
                        if (velocity.frame_id != self.profile['base_frame'] or not all(math.isfinite(v) for v in values) or
                                math.hypot(values[0], values[1]) > self.profile['max_linear_speed_m_s'] + 1e-6 or
                                abs(values[2]) > self.profile['max_yaw_speed_rad_s'] + 1e-6):
                            self.inhibit('Follower velocity violates profile', fault=True)
                        else:
                            twist = velocity.velocity
                    elif steady_s - execution.received_velocity_s >= .1 and not execution.terminal:
                        self.inhibit('PathVelocity expired', fault=True)
            elif execution.phase == 'arrived' and execution.cleared:
                target = next(t for t in self.targets.targets if t.target_id == self.core.target_id)
                measured, goal = pose_values(self.localization.pose.pose), pose_values(target.stop_pose.pose)
                if math.hypot(measured[0]-goal[0], measured[1]-goal[1]) <= self.profile['goal_position_tolerance_m'] and abs(wrap(measured[2]-goal[2])) <= self.profile['goal_yaw_tolerance_rad']:
                    self.core.state, execution.phase = GOAL, 'goal'
                else:
                    self.inhibit('Action arrival outside approved stop Pose', fault=True)
        elif self.core.state == TRACKING and self.safety_ok(self.core.generation):
            twist = self.tracking_twist()
        elif self.core.state == GOAL and self.targets_ok():
            target = next(t for t in self.targets.targets if t.target_id == self.core.target_id)
            measured, goal = pose_values(self.localization.pose.pose), pose_values(target.stop_pose.pose)
            if math.hypot(measured[0]-goal[0], measured[1]-goal[1]) > self.profile['goal_departure_m'] or abs(wrap(measured[2]-goal[2])) > self.profile['goal_departure_yaw_rad']:
                self.return_ready_after_cancel = True
                self.inhibit('Goal departed; execution invalidated')
        if self.core.cancelling or self.core.state == FAIL:
            twist, permit = Twist(), False
        self.publish_lease(permit)
        self.publish_path()
        self.publish_aim(self.aim_point())
        self.publish_operation(twist)
        self.last_twist = twist
        self.output_enabled = self.healthy() and not self.core.cancelling and (permit or self.core.state == TRACKING)
        self.publish_status(self.healthy())

    def publish_status(self, healthy):
        message = AutoModeStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.producer_session, message.generation = self.session, self.core.generation
        self.status_sequence += 1
        message.update_sequence, message.navigation_status, message.target_id = self.status_sequence, self.core.state, self.core.target_id
        message.reason = self.core.reason
        message.automatic_output_enabled = self.output_enabled
        message.contracts_confirmed = bool(self.profile and healthy)
        message.automatic_inhibited = not message.automatic_output_enabled
        message.cancellation_pending = self.core.cancelling
        message.board_applied = message.physical_stop_confirmed = AutoModeStatus.UNKNOWN
        if self.profile:
            message.profile_id, message.simulation_profile = self.profile['profile_id'], True
            message.navigation_enabled = healthy and self.targets_ok()
            message.tracking_enabled = healthy and self.track_ok()
            message.shot_calculation_enabled = healthy
            message.arm_enabled = False
            message.angle_valid = self.angle_valid
            message.execution_phase = 'idle'
            if self.execution:
                message.execution_phase = self.execution.phase
            message.fail_sent = bool(self.core.state == FAIL and self.bridge and self.bridge.last_send_success and self.bridge.operation_fresh and self.bridge.last_successful_send_age_ms < 100 and self.bridge.last_sent_navigation_status == FAIL and self.bridge.last_sent_producer_session == self.session and self.bridge.last_sent_generation == self.core.generation)
        self.status_pub.publish(message)


def main():
    rclpy.init()
    node = None
    try:
        node = AutoModeManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
