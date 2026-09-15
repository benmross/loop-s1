"""Waypoint generation, and small geometry helpers. No ROS in here."""

import math

import numpy as np


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def generate_waypoints(course, costmap, count, rng, min_spacing=6.0, clearance=0.8):
    """Random XY waypoints the vehicle can actually reach.

    A candidate has to be in the free space connected to the start, keep
    ``clearance`` beyond the planner's inflation from every obstacle, and be
    ``min_spacing`` from the start and from every earlier waypoint. The
    spacing relaxes if the course is too crowded to honour it.
    """
    sx, sy, _ = course.start
    usable = costmap.reachable(sx, sy) & (costmap.known_dist >= costmap.inflation + clearance)
    cells = np.argwhere(usable)
    if len(cells) == 0:
        raise RuntimeError('there is no free space to put waypoints in')
    points = []
    spacing = min_spacing
    while len(points) < count:
        for _ in range(500):
            i, j = cells[rng.randrange(len(cells))]
            cx, cy = costmap.point(int(i), int(j))
            x = round(cx + rng.uniform(-0.4, 0.4) * costmap.res, 2)
            y = round(cy + rng.uniform(-0.4, 0.4) * costmap.res, 2)
            if all(math.hypot(x - px, y - py) >= spacing for px, py in [(sx, sy)] + points):
                points.append((x, y))
                break
        else:
            spacing *= 0.8
    return points


def vehicle_outline(length, width, per_side=6):
    """Points around the vehicle's rectangle, in its own frame."""
    hx, hy = length / 2, width / 2
    t = np.linspace(-1.0, 1.0, per_side, endpoint=False)
    xs = np.concatenate([hx * t, np.full(per_side, hx), -hx * t, np.full(per_side, -hx)])
    ys = np.concatenate([np.full(per_side, -hy), hy * t, np.full(per_side, hy), -hy * t])
    return xs, ys
