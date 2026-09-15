"""Drive the vehicle to the current goal, on its own.

Inputs are odometry, the lidar and a goal. The navigator plans with A* over a
costmap built from the course description and updated from the lidar,
follows the plan with pure pursuit, and will not drive forward into anything
the lidar says is close, whatever the map believes. If it stays blocked it
backs off and plans again.
"""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from loop_s1_nav.grid import Costmap
from loop_s1_nav.gz_markers import GzMarkers
from loop_s1_nav.mission import wrap_angle, yaw_from_quaternion
from loop_s1_sim.course import load

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
PLAN_COLOR = (0.1, 0.45, 1.0)


class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')
        param = self.declare_parameter
        course_file = param('course_file', '').value
        self.rate = param('control_rate', 20.0).value
        self.max_v = param('max_linear_speed', 1.0).value
        self.max_w = param('max_angular_speed', 1.5).value
        self.accel = param('max_linear_accel', 1.0).value
        self.ang_accel = param('max_angular_accel', 3.0).value
        self.lookahead = param('lookahead', 0.8).value
        self.rotate_angle = param('rotate_in_place_angle', 0.9).value
        self.goal_tolerance = param('goal_tolerance', 0.3).value
        self.stop_distance = param('stop_distance', 0.45).value
        self.stop_half_angle = param('stop_half_angle', 0.6).value
        self.blocked_timeout = param('blocked_timeout', 3.0).value
        self.off_path = param('off_path_distance', 1.0).value
        self.use_lidar = param('use_lidar', True).value
        self.hit_threshold = param('lidar_hit_threshold', 3).value
        self.map_range = param('lidar_map_range', 5.0).value
        resolution = param('resolution', 0.1).value
        inflation = param('inflation', 0.6).value
        preferred = param('preferred_clearance', 1.2).value
        weight = param('clearance_weight', 3.0).value
        if not course_file:
            raise RuntimeError('the course_file parameter is required')

        self.costmap = Costmap(load(course_file), resolution, inflation, preferred, weight)
        self.markers = GzMarkers(self.get_logger())

        self.pose = None
        self.turn_rate = 0.0
        self.goal = None
        self.path = None
        self.path_index = 0
        self.arrived = False
        self.cmd = (0.0, 0.0)
        self.next_plan_time = 0.0
        self.blocked_since = None
        self.recover_until = 0.0
        self.recovery_cmd = (0.0, 0.0)
        self.ranges = None
        self.angles = None
        self.last_scan = None
        self.map_dirty = True
        self.sensed_cells = 0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.plan_pub = self.create_publisher(Path, 'plan', LATCHED)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', LATCHED)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'goal_pose', self._on_goal, LATCHED)
        self.create_timer(1.0 / self.rate, self._control)
        self.create_timer(1.0, self._publish_map)
        self.started = time.monotonic()
        self.get_logger().info(
            f'Costmap {self.costmap.nx}x{self.costmap.ny} at {resolution} m, '
            f'inflation {inflation} m. Waiting for odometry and a goal.')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # Inputs

    def _on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self.turn_rate = msg.twist.twist.angular.z

    def _on_goal(self, msg):
        goal = (msg.pose.position.x, msg.pose.position.y)
        if goal == self.goal:
            return
        self.goal = goal
        self.path = None
        self.arrived = False
        self.next_plan_time = 0.0
        self.blocked_since = None
        self.get_logger().info(f'New goal ({goal[0]:.2f}, {goal[1]:.2f})')

    def _on_scan(self, msg):
        ranges = np.asarray(msg.ranges, dtype=float)
        if self.angles is None or len(self.angles) != len(ranges):
            self.angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        self.ranges = np.where(ranges >= msg.range_min, ranges, np.inf)
        self.last_scan = time.monotonic()
        # Mapping from a scan taken while turning smears it, because the pose
        # it is placed with is a few milliseconds newer than the scan.
        if self.pose is None or abs(self.turn_rate) > 0.3:
            return
        ok = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= self.map_range)
        if not ok.any():
            return
        x, y, yaw = self.pose
        a = self.angles[ok] + yaw
        added = self.costmap.add_hits(x + ranges[ok] * np.cos(a), y + ranges[ok] * np.sin(a),
                                      self.hit_threshold)
        if not added:
            return
        self.costmap.update()
        self.sensed_cells += added
        self.map_dirty = True
        self.get_logger().info(
            f'Lidar marked {added} cells the course file does not have ({self.sensed_cells} total)')
        if self.path is not None and self.costmap.path_blocked(self.path, self.path_index):
            self.get_logger().info('The plan runs through something the lidar saw; replanning')
            self.path = None

    # Control

    def _control(self):
        if self.use_lidar and self.last_scan is None and time.monotonic() - self.started > 10.0:
            self.get_logger().warn('No lidar scans yet; driving on the map alone',
                                   throttle_duration_sec=30.0)
        if self.pose is None or self.goal is None:
            return self._send(0.0, 0.0)
        now = self._now()
        x, y, yaw = self.pose
        dist_goal = math.hypot(self.goal[0] - x, self.goal[1] - y)
        if dist_goal <= self.goal_tolerance:
            if not self.arrived:
                self.arrived = True
                self.get_logger().info(f'Arrived, {dist_goal:.2f} m from the goal')
            return self._send(0.0, 0.0)
        if now < self.recover_until:
            return self._send(*self.recovery_cmd, smooth=False)
        if self.path is None and not self._replan(now):
            return self._send(0.0, 0.0)

        v, w = self._pursue(x, y, yaw, dist_goal)
        if self.path is None:
            return self._send(0.0, 0.0)

        front = self._clearance(0.0)
        if front >= self.stop_distance + 0.1:
            self.blocked_since = None
        if v > 0.0 and front < self.stop_distance:
            v = 0.0
            if self.blocked_since is None:
                self.blocked_since = now
                self.path = None
                self.get_logger().warn(f'Something {front:.2f} m ahead; stopping and replanning')
            elif now - self.blocked_since > self.blocked_timeout:
                return self._start_recovery(now)
        self._send(v, w)

    def _replan(self, now):
        if now < self.next_plan_time:
            return False
        t0 = time.monotonic()
        path = self.costmap.plan(self.pose[:2], self.goal)
        ms = (time.monotonic() - t0) * 1000
        if path is None:
            self.next_plan_time = now + 1.0
            self.get_logger().warn(f'No path to ({self.goal[0]:.2f}, {self.goal[1]:.2f}); retrying',
                                   throttle_duration_sec=5.0)
            return False
        self.path = path
        self.path_index = 0
        length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
        self.get_logger().info(f'Planned {length:.1f} m in {ms:.0f} ms')
        self._publish_plan()
        return True

    def _pursue(self, x, y, yaw, dist_goal):
        path = self.path
        # Nearest point, looking only a little way ahead of the last one so a
        # path that doubles back past itself is not skipped.
        window = np.asarray(path[self.path_index:self.path_index + 50])
        d2 = (window[:, 0] - x) ** 2 + (window[:, 1] - y) ** 2
        k = int(np.argmin(d2))
        self.path_index += k
        if math.sqrt(d2[k]) > self.off_path:
            self.get_logger().info('Too far off the plan; replanning')
            self.path = None
            return 0.0, 0.0

        target = path[-1]
        for px, py in path[self.path_index:]:
            if math.hypot(px - x, py - y) >= self.lookahead:
                target = (px, py)
                break
        dx, dy = target[0] - x, target[1] - y
        alpha = wrap_angle(math.atan2(dy, dx) - yaw)
        if abs(alpha) > self.rotate_angle:
            return 0.0, math.copysign(max(0.5, min(self.max_w, 2.0 * abs(alpha))), alpha)

        v = self.max_v * max(0.25, math.cos(alpha))
        v = min(v, 0.25 + 0.6 * dist_goal)
        # Slow down where the plan squeezes between obstacles.
        clearance = self.costmap.clearance(x, y)
        v = min(v, self.max_v * min(1.0, max(0.35, clearance / self.costmap.preferred)))
        curvature = 2.0 * math.sin(alpha) / max(math.hypot(dx, dy), 1e-3)
        w = v * curvature
        if abs(w) > self.max_w:
            v *= self.max_w / abs(w)
            w = math.copysign(self.max_w, w)
        return v, w

    def _clearance(self, center):
        """Closest lidar return within ``stop_half_angle`` of a bearing."""
        if not self.use_lidar or self.ranges is None or time.monotonic() - self.last_scan > 1.0:
            return math.inf
        offsets = np.abs(np.arctan2(np.sin(self.angles - center), np.cos(self.angles - center)))
        values = self.ranges[offsets <= self.stop_half_angle]
        return float(values.min()) if values.size else math.inf

    def _start_recovery(self, now):
        if self._clearance(math.pi) > self.stop_distance + 0.2:
            self.recovery_cmd = (-0.25, 0.0)
            what = 'backing up'
        else:
            self.recovery_cmd = (0.0, 0.6 * self.max_w)
            what = 'turning in place'
        self.recover_until = now + 1.5
        self.blocked_since = None
        self.path = None
        self.get_logger().warn(f'Blocked for {self.blocked_timeout:.0f} s; {what}, then replanning')
        self._send(*self.recovery_cmd, smooth=False)

    def _send(self, v, w, smooth=True):
        if smooth:
            dt = 1.0 / self.rate
            v = _slew(self.cmd[0], v, self.accel * dt)
            w = _slew(self.cmd[1], w, self.ang_accel * dt)
        self.cmd = (v, w)
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    # Outputs for people

    def _publish_plan(self):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        points = self.path[::2] + [self.path[-1]]
        for px, py in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.plan_pub.publish(msg)
        self.markers.line('plan', 1, self.path[::3] + [self.path[-1]], PLAN_COLOR)

    def _publish_map(self):
        if not self.map_dirty:
            return
        self.map_dirty = False
        cm = self.costmap
        msg = OccupancyGrid()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(cm.res)
        msg.info.width = cm.nx
        msg.info.height = cm.ny
        msg.info.origin.position.x = cm.origin[0]
        msg.info.origin.position.y = cm.origin[1]
        msg.info.origin.orientation.w = 1.0
        msg.data = cm.occupancy_data().tolist()
        self.map_pub.publish(msg)


def _slew(previous, target, step):
    """Limit how fast a command grows. Slowing down is never limited."""
    if abs(target) <= abs(previous) and target * previous >= 0.0:
        return target
    return previous + max(-step, min(step, target - previous))


def main():
    rclpy.init()
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
