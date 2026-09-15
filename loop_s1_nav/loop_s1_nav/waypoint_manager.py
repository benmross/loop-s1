"""Generate random waypoints and hand them to the navigator in sequence.

It also keeps score: time and distance per leg, the closest the vehicle's
outline came to an obstacle, and whether it ever touched one.
"""

import json
import math
import random

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from loop_s1_nav.grid import Costmap
from loop_s1_nav.gz_markers import GzMarkers
from loop_s1_nav.mission import generate_waypoints, vehicle_outline, yaw_from_quaternion
from loop_s1_sim.course import load

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
PENDING = (0.85, 0.2, 0.15)
ACTIVE = (1.0, 0.8, 0.0)
DONE = (0.2, 0.8, 0.3)


class WaypointManager(Node):

    def __init__(self):
        super().__init__('waypoint_manager')
        param = self.declare_parameter
        course_file = param('course_file', '').value
        count = param('num_waypoints', 6).value
        seed = param('seed', -1).value
        self.tolerance = param('goal_tolerance', 0.35).value
        min_spacing = param('min_spacing', 6.0).value
        clearance = param('clearance', 0.8).value
        resolution = param('resolution', 0.1).value
        inflation = param('inflation', 0.6).value
        if not course_file:
            raise RuntimeError('the course_file parameter is required')

        self.course = load(course_file)
        self.costmap = Costmap(self.course, resolution, inflation)
        if seed < 0:
            seed = random.SystemRandom().randrange(1_000_000)
        self.seed = seed
        self.waypoints = generate_waypoints(self.course, self.costmap, count,
                                            random.Random(seed), min_spacing, clearance)
        listing = ', '.join(f'{n}: ({x:.2f}, {y:.2f})' for n, (x, y) in enumerate(self.waypoints, 1))
        self.get_logger().info(f'Seed {seed} (rerun it with seed:={seed}). Waypoints {listing}')

        length, width, _ = self.course.vehicle
        self.outline = vehicle_outline(length, width)
        self.radius = math.hypot(length, width) / 2
        self.index = 0
        self.pose = None
        self.t_start = None
        self.t_leg = None
        self.t_nag = None
        self.distance = 0.0
        self.leg_distance = 0.0
        self.min_clearance = math.inf
        self.contacts = 0
        self.touching = False

        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', LATCHED)
        self.waypoints_pub = self.create_publisher(PoseArray, 'waypoints', LATCHED)
        self.markers_pub = self.create_publisher(MarkerArray, 'waypoint_markers', LATCHED)
        self.status_pub = self.create_publisher(String, 'mission_status', LATCHED)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_timer(0.1, self._tick)
        self.gz = GzMarkers(self.get_logger())

        poses = PoseArray()
        poses.header.frame_id = 'odom'
        for x, y in self.waypoints:
            pose = Pose()
            pose.position.x, pose.position.y, pose.orientation.w = x, y, 1.0
            poses.poses.append(pose)
        self.waypoints_pub.publish(poses)
        self._draw()
        self._status()

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg):
        p = msg.pose.pose
        x, y, yaw = p.position.x, p.position.y, yaw_from_quaternion(p.orientation)
        if self.pose is not None and self.index < len(self.waypoints):
            step = math.hypot(x - self.pose[0], y - self.pose[1])
            self.distance += step
            self.leg_distance += step
        self.pose = (x, y, yaw)
        if self.t_start is None or self.index >= len(self.waypoints):
            return

        ox, oy = self.outline
        c, s = math.cos(yaw), math.sin(yaw)
        touching = bool(self.course.occupied(x + c * ox - s * oy, y + s * ox + c * oy).any())
        if touching and not self.touching:
            self.contacts += 1
            self.get_logger().error(f'The vehicle is touching an obstacle at ({x:.2f}, {y:.2f})')
        self.touching = touching
        self.min_clearance = min(self.min_clearance,
                                 self.costmap.clearance(x, y, known_only=True) - self.radius)

    def _tick(self):
        if self.pose is None or self.index >= len(self.waypoints):
            return
        now = self._now()
        if self.t_start is None:
            self.t_start = self.t_leg = self.t_nag = now
            self._publish_goal()
            return
        x, y = self.pose[:2]
        wx, wy = self.waypoints[self.index]
        remaining = math.hypot(wx - x, wy - y)
        if remaining > self.tolerance:
            if now - self.t_nag > 30.0:
                self.t_nag = now
                self.get_logger().info(
                    f'Waypoint {self.index + 1}: {remaining:.1f} m to go after {now - self.t_leg:.0f} s')
            return

        self.get_logger().info(
            f'Reached waypoint {self.index + 1}/{len(self.waypoints)} at ({wx:.2f}, {wy:.2f}): '
            f'{self.leg_distance:.1f} m in {now - self.t_leg:.1f} s')
        self.index += 1
        self.t_leg = self.t_nag = now
        self.leg_distance = 0.0
        self._draw()
        if self.index < len(self.waypoints):
            self._publish_goal()
        else:
            self.get_logger().info(
                f'Mission complete: {len(self.waypoints)} waypoints in {now - self.t_start:.1f} s '
                f'of sim time, {self.distance:.1f} m driven, closest approach '
                f'{self.min_clearance:.2f} m, {self.contacts} contacts with obstacles')
        self._status()

    def _publish_goal(self):
        wx, wy = self.waypoints[self.index]
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.orientation.w = wx, wy, 1.0
        self.goal_pub.publish(msg)
        self.get_logger().info(f'Heading for waypoint {self.index + 1}/{len(self.waypoints)} '
                               f'at ({wx:.2f}, {wy:.2f})')

    def _status(self):
        done = self.index >= len(self.waypoints)
        elapsed = (self._now() - self.t_start) if self.t_start is not None else 0.0
        self.status_pub.publish(String(data=json.dumps({
            'seed': self.seed,
            'waypoints': self.waypoints,
            'reached': self.index,
            'complete': done,
            'elapsed_s': round(elapsed, 1),
            'distance_m': round(self.distance, 1),
            'min_clearance_m': None if math.isinf(self.min_clearance) else round(self.min_clearance, 3),
            'contacts': self.contacts,
        })))

    def _draw(self):
        array = MarkerArray()
        for n, (x, y) in enumerate(self.waypoints):
            color = DONE if n < self.index else ACTIVE if n == self.index else PENDING
            radius = 0.45 if n == self.index else 0.35
            self.gz.disc('waypoints', n + 1, x, y, radius, color)

            disc = Marker()
            disc.header.frame_id = 'odom'
            disc.ns, disc.id, disc.type = 'waypoints', n, Marker.CYLINDER
            disc.pose.position.x, disc.pose.position.y, disc.pose.position.z = x, y, 0.05
            disc.pose.orientation.w = 1.0
            disc.scale.x = disc.scale.y = 2 * radius
            disc.scale.z = 0.1
            disc.color.r, disc.color.g, disc.color.b, disc.color.a = (*color, 0.9)
            label = Marker()
            label.header.frame_id = 'odom'
            label.ns, label.id, label.type = 'labels', n, Marker.TEXT_VIEW_FACING
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, 0.6
            label.pose.orientation.w = 1.0
            label.scale.z = 0.6
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = str(n + 1)
            array.markers += [disc, label]
        self.markers_pub.publish(array)


def main():
    rclpy.init()
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
