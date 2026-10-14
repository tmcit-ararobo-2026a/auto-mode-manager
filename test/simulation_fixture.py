#!/usr/bin/env python3
"""Test-only mainboard, synthetic sensors and inert mechanisms; no Operation producer."""
import math
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, Imu, PointField
from tf2_ros import TransformBroadcaster
from robot_port.msg import RobotCommand
from auto_mode_manager.msg import SimulationState
from gn10_pointcloud_localization.msg import LocalizationStatus
from field_target_pub.msg import TargetSet, FieldTarget
from lidar_perception_system.msg import MovingBucketTrack
from runtime_common import load_profile, session_id


class SimulationFixture(Node):
    """Generate synthetic inputs and integrate only loopback automatic wheel velocities."""

    def __init__(self):
        super().__init__('stm32_test_fixture')
        if not self.declare_parameter('test_mode', False).value:
            raise ValueError('Fixture requires explicit test_mode')
        self.profile = load_profile(self.declare_parameter('profile_file', '').value)
        self.command_port = self.declare_parameter('command_port', 26574).value
        self.operation_port = self.declare_parameter('operation_port', 39244).value
        if self.command_port == self.operation_port or not all(1 <= port <= 65535 for port in (self.command_port, self.operation_port)):
            raise ValueError('Invalid isolated ports')
        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.operation_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_socket.bind(('127.0.0.2', self.command_port))
        self.operation_socket.bind(('127.0.0.2', self.operation_port))
        self.operation_socket.setblocking(False)
        self.session = session_id()
        self.sequence = self.count = 0
        self.command = RobotCommand()
        self.fault = ''
        self.x, self.y, self.yaw = 2.5, 2.5, 0.0
        self.velocity = (0., 0., 0.)
        self.operation_received_s = 0.0
        self.last_tick_s = time.monotonic()
        self.create_subscription(RobotCommand, '/stm32_test/command', self.set_command, 100)
        self.create_subscription(String, '/stm32_test/fault', self.set_fault, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/livox/lidar', 10)
        self.imu_pub = self.create_publisher(Imu, '/livox/imu', 10)
        self.raw_pub = self.create_publisher(LocalizationStatus, '/localization/status_raw', 10)
        self.obstacle_pub = self.create_publisher(PointCloud2, '/obstacle_cloud', 10)
        self.target_pub = self.create_publisher(TargetSet, '/field/targets', 10)
        self.track_pub = self.create_publisher(MovingBucketTrack, '/moving_bucket_detector/target', 10)
        self.mechanism_pub = self.create_publisher(SimulationState, '/auto_mode/simulation_state', 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/costmap', qos)
        self.tf = TransformBroadcaster(self)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02, self.tick, clock=self.steady_clock)

    def set_command(self, message):
        self.command = message

    def set_fault(self, message):
        if self.fault == message.data:
            return
        self.fault = message.data
        if self.fault == 'goal_departure':
            self.x += .21
        elif self.fault == 'wrong_pose':
            self.x += 1.

    def tick(self):
        if self.count % 25 == 0:
            if any(not name.startswith(('stm32_test_', '_ros2cli', 'transform_listener_impl_')) for name in self.get_node_names()):
                raise RuntimeError('Non-test graph: refusing synthetic fixture')
            if self.count_publishers('/robot/operation') > 1:
                raise RuntimeError('Multiple Operation producers in test graph')
        current_s = time.monotonic()
        dt = min(.05, current_s - self.last_tick_s)
        self.last_tick_s = current_s
        while True:
            try:
                packet, source = self.operation_socket.recvfrom(65536)
            except BlockingIOError:
                break
            if source != ('127.0.0.1', self.operation_port) or len(packet) != 32 or packet[0] != 0xAB or packet[1] != 0:
                raise RuntimeError('Invalid source or operation layout')
            values = struct.unpack_from('<5f', packet, 4)
            if not all(math.isfinite(v) for v in values) or any(packet[24:26]) or values[4] != 0:
                raise RuntimeError('Invalid operation or forbidden hand output')
            self.velocity = values[:3]
            self.operation_received_s = current_s
        if current_s - self.operation_received_s > .1:
            self.velocity = (0., 0., 0.)
        if self.fault != 'freeze_pose':
            vx, vy, vyaw = self.velocity
            self.x += (math.cos(self.yaw) * vx - math.sin(self.yaw) * vy) * dt
            self.y += (math.sin(self.yaw) * vx + math.cos(self.yaw) * vy) * dt
            self.yaw = math.atan2(math.sin(self.yaw + vyaw * dt), math.cos(self.yaw + vyaw * dt))
        self.count += 1
        self.sequence = (self.sequence + 1) % 256
        if self.fault != 'stop_command':
            command = bytes((0x38, self.sequence, 0, 1, 0, self.command.target_id, self.command.navigation_command, 0))
            self.command_socket.sendto(command, ('127.0.0.1', self.command_port))
        if self.fault != 'stop_feedback':
            feedback = struct.pack('<4B10f', 0x55, self.sequence, 0, 0, *([0.] * 10))
            self.operation_socket.sendto(feedback, ('127.0.0.1', self.operation_port))
        stamp = self.get_clock().now().to_msg()
        if self.fault != 'stop_cloud':
            cloud = PointCloud2()
            cloud.header.stamp, cloud.header.frame_id = stamp, 'base_link'
            self.cloud_pub.publish(cloud)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'base_link'
        self.imu_pub.publish(imu)
        if self.fault != 'stop_quality':
            transform = TransformStamped()
            transform.header.stamp, transform.header.frame_id = stamp, self.profile['map_frame']
            transform.child_frame_id = self.profile['base_frame']
            transform.transform.translation.x, transform.transform.translation.y = self.x, self.y
            transform.transform.rotation.z, transform.transform.rotation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
            self.tf.sendTransform(transform)
            raw = LocalizationStatus()
            raw.header.stamp, raw.header.frame_id = stamp, self.profile['map_frame']
            raw.measurement_stamp, raw.pose_stamp = stamp, stamp
            raw.processor_session, raw.result_sequence = self.session, self.count
            raw.last_match_success = self.fault != 'bad_quality'
            raw.lost = not raw.last_match_success
            raw.map_loaded, raw.cloud_tf_available, raw.imu_tf_available = True, True, True
            raw.match_metrics_valid, raw.inlier_metrics_valid = True, True
            raw.best_cost, raw.inlier_count, raw.inlier_cost = .1, 100, .01
            for key in ('map_id', 'team', 'extrinsic_id', 'implementation_id', 'configuration_id'):
                setattr(raw, key, self.profile[key])
            raw.pose.header = raw.header
            raw.pose.pose.position.x, raw.pose.pose.position.y = self.x, self.y
            raw.pose.pose.orientation = transform.transform.rotation
            raw.pose_valid = True
            self.raw_pub.publish(raw)
        if self.fault != 'stop_obstacles':
            cloud = PointCloud2()
            cloud.header.stamp, cloud.header.frame_id = stamp, self.profile['map_frame']
            cloud.height, cloud.point_step = 1, 12
            cloud.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                            for name, offset in (('x', 0), ('y', 4), ('z', 8))]
            if self.fault == 'obstacle_point':
                cloud.width, cloud.row_step = 1, 12
                cloud.data = struct.pack('<3f', self.x + .1, self.y, 1.)
            self.obstacle_pub.publish(cloud)
        state = SimulationState()
        state.stamp, state.fixture_session, state.update_sequence = stamp, self.session, self.count
        state.profile_id = self.profile['profile_id']
        state.mock_drive_ready = self.fault != 'invalid_mechanism'
        state.mock_hand_disabled, state.mock_launcher_disabled = True, True
        self.mechanism_pub.publish(state)
        targets = TargetSet()
        targets.header.stamp, targets.header.frame_id = stamp, self.profile['map_frame']
        targets.team, targets.map_id, targets.revision, targets.valid = self.profile['team'], self.profile['map_id'], 1, True
        if self.fault == 'profile_changed':
            targets.revision += 1
        for target_id in range(7):
            target = FieldTarget()
            target.target_id, target.label = target_id, 'synthetic'
            target.stop_pose.header = targets.header
            target.stop_pose.pose.position.x, target.stop_pose.pose.position.y = 2.5 + .4 * min(target_id, 1), 2.5
            target.stop_pose.pose.orientation.w = 1.
            target.stop_pose_valid, target.aim_valid = True, target_id != 0
            target.aim_point.header = targets.header
            target.aim_point.point.x, target.aim_point.point.y, target.aim_point.point.z = 4.5, 2.5, 1.0
            targets.targets.append(target)
        self.target_pub.publish(targets)
        if self.fault != 'stop_map':
            grid = OccupancyGrid()
            grid.header = targets.header
            grid.info.resolution, grid.info.width, grid.info.height = .1, 50, 50
            grid.info.origin.orientation.w = 1.
            grid.data = [0] * 2500
            if self.fault == 'obstacle':
                grid.data[25 * 50 + 25] = -2  # custom unsigned lethal=254
            self.map_pub.publish(grid)
        if self.fault != 'stop_track':
            track = MovingBucketTrack()
            track.header = targets.header
            track.last_observation_stamp = stamp
            track.track_id, track.valid, track.observation_mode = 44, True, MovingBucketTrack.DIRECT
            track.position.x, track.position.y, track.position.z = 4.5, 3.2, 1.0
            track.confidence = 1.
            self.track_pub.publish(track)

    def close(self):
        self.command_socket.close()
        self.operation_socket.close()


def main():
    rclpy.init()
    node = None
    try:
        node = SimulationFixture()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
