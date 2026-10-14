#!/usr/bin/env python3
"""Independent contract-v1 safety evaluation, with original input deadlines."""
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from gn10_pointcloud_localization.msg import LocalizationStatus
from auto_mode_manager.msg import AutoModeStatus, SafetyState, ExecutionPath
from contract_core import Freshness, MOVING, TRACKING, wrap
from runtime_common import load_profile, session_id, stamp_ns, pose_values


class NavigationSafetyMonitor(Node):
    """Evaluate costmap, configured hazards and the current execution path."""

    def __init__(self):
        super().__init__('navigation_safety_monitor')
        self.profile = load_profile(self.declare_parameter('profile_file', '').value)
        self.session = session_id()
        self.sequence = 0
        self.last_ros = 0
        self.localization = self.status = self.map = self.path = None
        self.inputs = {key: Freshness(limit) for key, limit in
                       (('localization', .1), ('status', .1), ('map', self.profile['map_timeout_s']), ('path', .1))}
        self.map_stamp = 0
        self.map_sequence = 0
        self.create_subscription(LocalizationStatus, '/localization/status', self.localize, 10)
        self.create_subscription(AutoModeStatus, '/auto_mode/status', self.observe_status, 10)
        self.create_subscription(ExecutionPath, '/auto_mode/execution_path', self.observe_path, 10)
        self.create_subscription(OccupancyGrid, '/costmap', self.observe_map, 10)
        self.publisher = self.create_publisher(SafetyState, '/auto_mode/safety', 10)
        self.create_timer(.02, self.evaluate)

    def accept(self, key, session, sequence, stamp):
        return self.inputs[key].accept(session, sequence, stamp_ns(stamp),
                                       self.get_clock().now().nanoseconds, time.monotonic())[0]

    def localize(self, message):
        if self.accept('localization', message.monitor_session, message.report_sequence, message.header.stamp):
            self.localization = message

    def observe_status(self, message):
        if self.accept('status', message.producer_session, message.update_sequence, message.stamp):
            self.status = message

    def observe_path(self, message):
        self.path = message
        # Path publication is a current execution snapshot, not a plan result lease.
        self.accept('path', message.producer_session, stamp_ns(message.stamp), message.stamp)

    def observe_map(self, message):
        stamp = stamp_ns(message.header.stamp)
        if stamp <= self.map_stamp:
            return
        self.map_stamp = stamp
        self.map_sequence += 1
        if self.accept('map', 1, self.map_sequence, message.header.stamp):
            self.map = message

    def point_safe(self, x, y):
        """Conservatively reject every grid cell touched by the circular footprint."""
        grid = self.map
        radius = self.profile['footprint_radius_m']
        for x0, y0, x1, y1 in self.profile['danger_rectangles_m']:
            if x + radius >= x0 and x - radius <= x1 and y + radius >= y0 and y - radius <= y1:
                return False
        origin, resolution = grid.info.origin.position, grid.info.resolution
        low_x = math.floor((x - radius - origin.x) / resolution)
        high_x = math.floor((x + radius - origin.x) / resolution)
        low_y = math.floor((y - radius - origin.y) / resolution)
        high_y = math.floor((y + radius - origin.y) / resolution)
        if low_x < 0 or low_y < 0 or high_x >= grid.info.width or high_y >= grid.info.height:
            return False
        for row in range(low_y, high_y + 1):
            for column in range(low_x, high_x + 1):
                if grid.data[row * grid.info.width + column] & 255 >= 254:
                    return False
        return True

    def segment_safe(self, start, end):
        """Check a conservative swept corridor, including adjacent cells between samples."""
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        resolution = self.map.info.resolution
        count = max(1, math.ceil(distance / (resolution * .5)))
        if count > 100000:
            return False
        # Expand each sampled footprint by half a step, covering the full segment.
        original = self.profile['footprint_radius_m']
        self.profile['footprint_radius_m'] += resolution * .25
        try:
            return all(self.point_safe(start[0] + (end[0] - start[0]) * sample / count,
                                       start[1] + (end[1] - start[1]) * sample / count)
                       for sample in range(count + 1))
        finally:
            self.profile['footprint_radius_m'] = original

    def assess(self, ros_ns, steady_s):
        """Complete one real assessment; no heartbeat is substituted for input checks."""
        for key in ('status', 'localization', 'map'):
            if not self.inputs[key].fresh(ros_ns, steady_s):
                return False, False, key + ' expired'
        status, localization, grid = self.status, self.localization, self.map
        if (not status.simulation_profile or status.profile_id != self.profile['profile_id'] or
                not localization.simulation_profile or not localization.ready_quality_valid or
                localization.quality_id != self.profile['quality_id'] or
                localization.map_id != self.profile['map_id'] or localization.team != self.profile['team'] or
                ros_ns >= stamp_ns(localization.quality_valid_until)):
            return False, False, 'Profile or localization quality unavailable'
        pose = pose_values(localization.pose.pose)
        origin = pose_values(grid.info.origin)
        if (pose is None or origin is None or abs(origin[2]) > 1e-6 or
                grid.header.frame_id != self.profile['map_frame'] or grid.info.width <= 0 or
                grid.info.height <= 0 or not math.isfinite(grid.info.resolution) or grid.info.resolution <= 0 or
                len(grid.data) != grid.info.width * grid.info.height or len(grid.data) > 1000000):
            return False, False, 'Invalid pose or costmap'
        if not self.point_safe(*pose[:2]):
            return False, False, 'Footprint in lethal cost or danger region'
        checked = False
        if status.navigation_status == MOVING and status.execution_phase != 'planning' and self.path is not None and self.path.valid:
            path = self.path
            if status.execution_phase == 'awaiting_safety' and (path.producer_session != status.producer_session or path.generation != status.generation):
                return True, False, 'Awaiting current execution path'
            if (not self.inputs['path'].fresh(ros_ns, steady_s) or
                    path.producer_session != status.producer_session or path.generation != status.generation or
                    path.path.header.frame_id != self.profile['map_frame'] or not path.path.poses or
                    len(path.path.poses) > 10000):
                return False, False, 'Execution path identity invalid'
            previous = pose[:2]
            for point in path.path.poses:
                end = pose_values(point.pose)
                if end is None or not self.segment_safe(previous, end[:2]):
                    return False, False, 'Execution corridor blocked'
                previous = end[:2]
            checked = True
        if status.navigation_status == MOVING and status.execution_phase in ('awaiting_goal', 'following') and not checked:
            return False, False, 'Active execution path missing'
        return True, checked, 'Synthetic costmap and hazard assessment complete'

    def evaluate(self):
        ros_ns, steady_s = self.get_clock().now().nanoseconds, time.monotonic()
        safe, checked, reason = self.assess(ros_ns, steady_s)
        if self.last_ros and ros_ns < self.last_ros:
            safe, checked, reason = False, False, 'ROS clock rollback'
            for stream in self.inputs.values():
                stream.deadline_s = 0
        self.last_ros = ros_ns
        output = SafetyState()
        output.evaluated_stamp = self.get_clock().now().to_msg()
        output.monitor_session, output.evaluation_sequence = self.session, self.sequence + 1
        self.sequence += 1
        output.valid = True
        output.safe, output.path_checked, output.reason = safe, checked, reason
        output.profile_id, output.simulation_profile = self.profile['profile_id'], True
        if self.status:
            output.manager_session, output.generation = self.status.producer_session, self.status.generation
        stamps = [stream.stamp_ns for key, stream in self.inputs.items() if key != 'path' and stream.stamp_ns]
        if stamps:
            oldest = min(stamps)
            output.oldest_input_stamp.sec, output.oldest_input_stamp.nanosec = divmod(oldest, 1000000000)
        expiry = ros_ns + 100000000
        for key, stream in self.inputs.items():
            if key == 'path' and not checked:
                continue
            expiry = min(expiry, stream.stamp_ns + int(stream.timeout_s * 1e9))
        if self.localization:
            expiry = min(expiry, stamp_ns(self.localization.quality_valid_until))
        expiry = max(0, expiry)
        output.valid_until.sec, output.valid_until.nanosec = divmod(expiry, 1000000000)
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = None
    try:
        node = NavigationSafetyMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
