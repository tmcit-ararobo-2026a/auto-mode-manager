"""Shared validation for simulation processes; never enables a physical profile."""
import json
import math
import os
import random
from pathlib import Path


def session_id():
    """Produce a nonzero process identity."""
    return random.SystemRandom().randrange(1, 1 << 64)


def stamp_ns(stamp):
    """Reject malformed or negative ROS measurement time."""
    if stamp.sec < 0 or stamp.nanosec >= 1000000000:
        return 0
    return stamp.sec * 1000000000 + stamp.nanosec


def fresh_stamp(stamp, now_ns, limit_s):
    """Validate production age rather than receive time."""
    value = stamp_ns(stamp)
    return value > 0 and 0 <= (now_ns - value) * 1e-9 < limit_s


def load_profile(filename):
    """Require explicit synthetic evidence and a dedicated localhost domain."""
    with Path(filename).open() as stream:
        profile = json.load(stream)
    if profile.get('schema') != 1 or profile.get('mode') != 'simulation' or profile.get('approved') is not True:
        raise ValueError('Only explicit approved simulation profiles are implemented; HIL/full_robot unavailable')
    if (os.getenv('ROS_LOCALHOST_ONLY') != '1' or not 100 <= int(os.getenv('ROS_DOMAIN_ID', '0')) <= 232):
        raise ValueError('Simulation requires localhost DDS and a dedicated domain [100,232]')
    for key in ('profile_id', 'evidence_id', 'map_id', 'team', 'quality_id', 'extrinsic_id',
                'implementation_id', 'configuration_id', 'map_frame', 'base_frame', 'shot_calibration_id'):
        if not isinstance(profile.get(key), str) or not profile[key]:
            raise ValueError('Missing profile identity: ' + key)
    if not profile['evidence_id'].startswith('synthetic-') or profile['team'] not in ('red', 'blue'):
        raise ValueError('Simulation evidence and explicit team are required')
    if profile.get('command_policy') != 'observed_transition_v1' or profile['map_frame'] == profile['base_frame']:
        raise ValueError('Invalid command policy or frames')
    for key in ('max_linear_speed_m_s', 'max_yaw_speed_rad_s', 'yaw_gain', 'yaw_acceleration_rad_s2',
                'track_timeout_s', 'goal_position_tolerance_m', 'goal_yaw_tolerance_rad',
                'goal_departure_m', 'goal_departure_yaw_rad', 'footprint_radius_m', 'map_timeout_s', 'obstacle_clearance_m', 'obstacle_timeout_s'):
        if not isinstance(profile.get(key), (float, int)) or not math.isfinite(profile[key]) or profile[key] <= 0:
            raise ValueError('Invalid simulation limit: ' + key)
    if (profile['goal_departure_m'] <= profile['goal_position_tolerance_m'] or
            profile['goal_departure_yaw_rad'] <= profile['goal_yaw_tolerance_rad']):
        raise ValueError('Goal departure limits must exceed arrival limits')
    if profile.get('angle_sign') not in (-1, 1) or not math.isfinite(profile.get('angle_offset_deg', math.nan)):
        raise ValueError('Invalid simulation angle calibration')
    if not isinstance(profile.get('target_revision'), int) or profile['target_revision'] <= 0:
        raise ValueError('Invalid target revision')
    for rectangle in profile.get('danger_rectangles_m', []):
        if len(rectangle) != 4 or not all(math.isfinite(x) for x in rectangle) or rectangle[0] >= rectangle[2] or rectangle[1] >= rectangle[3]:
            raise ValueError('Invalid danger rectangle')
    return profile


def pose_values(pose):
    """Validate finite position and a normalizable level quaternion, then return x/y/yaw."""
    p, q = pose.position, pose.orientation
    if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
        return None
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-9:
        return None
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    # The navigation model is planar; refuse non-level poses.
    if abs(x) > 1e-4 or abs(y) > 1e-4:
        return None
    return p.x, p.y, math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
