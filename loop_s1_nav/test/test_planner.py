"""The course and the planner, without ROS or Gazebo.

Run from the repository root with ``python3 -m pytest loop_s1_nav/test``.
"""

import math
import os
import random
import sys
import time
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'loop_s1_sim'))

from loop_s1_nav.grid import Costmap  # noqa: E402
from loop_s1_nav.mission import generate_waypoints  # noqa: E402
from loop_s1_sim.course import Circle, load  # noqa: E402

COURSE = load(os.path.join(HERE, '..', '..', 'loop_s1_sim', 'config', 'course.yaml'))
COSTMAP = Costmap(COURSE)


def test_world_is_valid_xml_with_every_obstacle():
    root = ET.fromstring(COURSE.world_sdf())
    names = {m.get('name') for m in root.iter('model')}
    assert {o.name for o in COURSE.obstacles} <= names
    assert 'vehicle' in names


def test_every_obstacle_is_on_the_map():
    for obstacle in COURSE.obstacles:
        fp = obstacle.footprint
        x, y = (fp.cx, fp.cy)
        i, j = COSTMAP.cell(x, y)
        if COSTMAP.in_bounds(i, j):
            assert COSTMAP.known[i, j], obstacle.name
        if isinstance(fp, Circle):
            assert COURSE.occupied(x + 0.95 * fp.r, y)
            assert not COURSE.occupied(x + 1.05 * fp.r, y) or obstacle.name.startswith('boundary')


def test_start_is_free():
    assert not COSTMAP.is_blocked(*COURSE.start[:2])


def test_random_waypoints_are_reachable_in_sequence():
    slowest = 0.0
    for seed in range(15):
        waypoints = generate_waypoints(COURSE, COSTMAP, 6, random.Random(seed))
        assert len(waypoints) == 6
        previous = COURSE.start[:2]
        for waypoint in waypoints:
            assert not COSTMAP.is_blocked(*waypoint)
            t0 = time.monotonic()
            path = COSTMAP.plan(previous, waypoint)
            slowest = max(slowest, time.monotonic() - t0)
            assert path, (seed, previous, waypoint)
            assert math.dist(path[-1], waypoint) < 1e-6
            for x, y in path[1:]:
                assert not COURSE.occupied(x, y)
                assert COSTMAP.clearance(x, y) >= COSTMAP.inflation
            previous = waypoint
    print(f'slowest plan {slowest * 1000:.0f} ms')


def test_lidar_hits_on_known_obstacles_add_nothing():
    cm = Costmap(COURSE)
    wall = next(o for o in COURSE.obstacles if o.name == 'wall_south').footprint
    xs = [wall.cx + d for d in (-3.0, 0.0, 3.0)] * 5
    ys = [wall.cy - wall.hy - 0.05] * 15
    assert cm.add_hits(xs, ys) == 0


def test_unknown_obstacle_reroutes_the_plan():
    cm = Costmap(COURSE)
    start, goal = (-12.0, -12.0), (-12.0, 12.0)
    before = cm.plan(start, goal)
    # A wall the course file does not have, straight across the plan.
    i = len(before) // 3
    cx, cy = before[i]
    xs = [cx + k * 0.1 for k in range(-15, 16)] * 3
    ys = [cy] * len(xs)
    assert cm.add_hits(xs, ys) > 0
    cm.update()
    assert cm.path_blocked(before)
    after = cm.plan(start, goal)
    assert after and not cm.path_blocked(after)
