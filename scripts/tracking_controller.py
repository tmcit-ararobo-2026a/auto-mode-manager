"""Planar yaw-only tracking with explicit units, rate limits and no XY output."""
import math
from contract_core import wrap


def yaw_velocity(robot_x_m, robot_y_m, robot_yaw_rad, target_x_m, target_y_m,
                 previous_yaw_rad_s, gain, max_speed_rad_s, max_acceleration_rad_s2, elapsed_s):
    """Reject undefined geometry and bound both requested speed and its slew."""
    values = (robot_x_m, robot_y_m, robot_yaw_rad, target_x_m, target_y_m,
              previous_yaw_rad_s, gain, max_speed_rad_s, max_acceleration_rad_s2, elapsed_s)
    if not all(math.isfinite(value) for value in values):
        return None
    if min(gain, max_speed_rad_s, max_acceleration_rad_s2, elapsed_s) <= 0 or elapsed_s > .1:
        return None
    dx, dy = target_x_m - robot_x_m, target_y_m - robot_y_m
    if math.hypot(dx, dy) < 1e-6:
        return None
    desired = gain * wrap(math.atan2(dy, dx) - robot_yaw_rad)
    desired = max(-max_speed_rad_s, min(max_speed_rad_s, desired))
    step = max_acceleration_rad_s2 * min(.02, elapsed_s)
    return previous_yaw_rad_s + max(-step, min(step, desired - previous_yaw_rad_s))
