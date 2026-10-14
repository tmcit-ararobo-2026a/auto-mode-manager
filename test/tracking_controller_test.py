"""Validate bearing wrap, left/right yaw and bounded rates independently of ROS."""
import sys
import math
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from tracking_controller import yaw_velocity


def test_yaw_slew_and_wrapped_heading():
    assert yaw_velocity(0, 0, 0, 1, 1, 0, 1, .5, 1, .02) == .02
    assert yaw_velocity(0, 0, 0, 1, -1, 0, 1, .5, 1, .02) == -.02
    # Heading and target on opposite sides of pi are close, not a full turn apart.
    angle = yaw_velocity(0, 0, math.pi - .01, -1, -.01, 0, 1, .5, 1, .02)
    assert 0 < angle <= .02
    assert yaw_velocity(0, 0, 0, 0, 0, 0, 1, .5, 1, .02) is None
    assert yaw_velocity(0, 0, 0, math.nan, 1, 0, 1, .5, 1, .02) is None
    assert yaw_velocity(0, 0, 0, 1, 1, 0, 1, .5, 1, .101) is None
    assert yaw_velocity(0, 0, 0, 1, 1, .5, 1, .5, 1, .02) == .5
