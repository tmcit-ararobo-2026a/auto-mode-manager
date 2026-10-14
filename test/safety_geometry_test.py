"""Test lethal unsigned cells, swept footprints and stopping-clearance boundaries."""
import sys
from pathlib import Path
from nav_msgs.msg import OccupancyGrid
sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from navigation_safety_monitor import NavigationSafetyMonitor


def test_costmap_hazards_and_obstacle_corridor():
    monitor = object.__new__(NavigationSafetyMonitor)
    monitor.profile = dict(footprint_radius_m=.1, obstacle_clearance_m=.15,
                           danger_rectangles_m=[[2., 2., 3., 3.]])
    monitor.map = OccupancyGrid()
    monitor.map.info.width = monitor.map.info.height = 40
    monitor.map.info.resolution = .1
    monitor.map.data = [0] * 1600
    assert monitor.point_safe(.5, .5)
    assert not monitor.point_safe(.05, .5)  # Body extends outside approved map.
    assert not monitor.point_safe(1.95, 2.5)  # Body overlaps configured danger region.
    monitor.map.data[5 * 40 + 10] = -2
    assert not monitor.segment_safe((.5, .5), (1.5, .5))
    assert not monitor.obstacle_corridor_safe((.5, .5), (1.5, .5), [(1., .7)])
    assert monitor.obstacle_corridor_safe((.5, .5), (1.5, .5), [(1., .76)])
