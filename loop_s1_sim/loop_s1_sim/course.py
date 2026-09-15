"""The S1 course, described once.

``config/course.yaml`` lists the arena, the vehicle and every obstacle. The
Gazebo world is generated from it at launch and the planner rasterises the
same shapes, so what the simulator collides with and what the vehicle plans
around cannot drift apart.
"""

import math
from dataclasses import dataclass

import numpy as np
import yaml


@dataclass(frozen=True)
class Rect:
    """Ground footprint of a rectangle rotated by ``yaw`` about its centre."""

    cx: float
    cy: float
    yaw: float
    hx: float
    hy: float

    def contains(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = np.subtract(x, self.cx), np.subtract(y, self.cy)
        return (np.abs(c * dx + s * dy) <= self.hx) & (np.abs(-s * dx + c * dy) <= self.hy)


@dataclass(frozen=True)
class Circle:
    """Ground footprint of a circle."""

    cx: float
    cy: float
    r: float

    def contains(self, x, y):
        return np.subtract(x, self.cx) ** 2 + np.subtract(y, self.cy) ** 2 <= self.r ** 2


@dataclass(frozen=True)
class Obstacle:
    name: str
    kind: str
    footprint: object
    sdf: str


COLORS = {
    'wall': (0.55, 0.55, 0.58),
    'box': (0.25, 0.4, 0.75),
    'ramp': (0.8, 0.5, 0.2),
    'hill': (0.35, 0.55, 0.25),
}


def _fmt(values):
    return ' '.join(f'{v:.4f}' for v in values)


def _material(rgb):
    r, g, b = rgb
    return (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
            '<specular>0.1 0.1 0.1 1</specular></material>')


def _static_model(name, pose, geometry, rgb):
    return f'''
    <model name="{name}">
      <static>true</static>
      <pose>{_fmt(pose)}</pose>
      <link name="link">
        <collision name="collision"><geometry>{geometry}</geometry></collision>
        <visual name="visual"><geometry>{geometry}</geometry>{_material(rgb)}</visual>
      </link>
    </model>'''


def _box(name, kind, o):
    if kind == 'wall':
        sx, sy = float(o['length']), float(o.get('thickness', 0.3))
    else:
        sx, sy = float(o['size_x']), float(o['size_y'])
    x, y = float(o['x']), float(o['y'])
    yaw = math.radians(float(o.get('yaw_deg', 0.0)))
    h = float(o.get('height', 1.0))
    geometry = f'<box><size>{sx} {sy} {h}</size></box>'
    return Obstacle(name, kind, Rect(x, y, yaw, sx / 2, sy / 2),
                    _static_model(name, (x, y, h / 2, 0, 0, yaw), geometry, COLORS[kind]))


def _ramp(name, o):
    """A slab pitched so its top face climbs from the ground to ``height``.

    ``x, y`` is the centre of the slope's run and ``yaw`` points uphill.
    """
    x, y = float(o['x']), float(o['y'])
    yaw = math.radians(float(o.get('yaw_deg', 0.0)))
    run, width, height = float(o['length']), float(o['width']), float(o['height'])
    theta = math.atan2(height, run)
    slope = math.hypot(run, height)
    # Thick enough that the raised end face reaches the ground.
    thickness = height / math.cos(theta) + 0.05

    # Pitching by -theta raises the slab's +x end. Shift it back along x and
    # down so the centre of the top face sits at (x, y, height / 2).
    back = 0.5 * thickness * math.sin(theta)
    z = 0.5 * height - 0.5 * thickness * math.cos(theta)
    pose = (x + math.cos(yaw) * back, y + math.sin(yaw) * back, z, 0.0, -theta, yaw)
    geometry = f'<box><size>{slope} {width} {thickness}</size></box>'

    # Above ground, the raised end face slopes outward by height * tan(theta).
    overhang = height * math.tan(theta)
    shift = overhang / 2
    footprint = Rect(x + math.cos(yaw) * shift, y + math.sin(yaw) * shift, yaw,
                     (run + overhang) / 2, width / 2)
    return Obstacle(name, 'ramp', footprint, _static_model(name, pose, geometry, COLORS['ramp']))


def _hill(name, o):
    """A mostly buried sphere: ``radius`` on the ground, ``height`` at the top."""
    x, y = float(o['x']), float(o['y'])
    r, h = float(o['radius']), float(o['height'])
    sphere = (r * r + h * h) / (2 * h)
    geometry = f'<sphere><radius>{sphere}</radius></sphere>'
    return Obstacle(name, 'hill', Circle(x, y, r),
                    _static_model(name, (x, y, h - sphere, 0, 0, 0), geometry, COLORS['hill']))


class Course:

    def __init__(self, spec):
        arena = spec['arena']
        self.size_x = float(arena['size_x'])
        self.size_y = float(arena['size_y'])
        self.wall_thickness = float(arena.get('wall_thickness', 0.3))
        wall_height = float(arena.get('wall_height', 1.0))

        start = spec['start']
        self.start = (float(start['x']), float(start['y']),
                      math.radians(float(start.get('yaw_deg', 0.0))))
        v = spec['vehicle']
        self.vehicle = (float(v['length']), float(v['width']), float(v['height']))

        hx, hy, t = self.size_x / 2, self.size_y / 2, self.wall_thickness
        boundary = [
            ('boundary_north', 0.0, hy + t / 2, 0, self.size_x + 2 * t),
            ('boundary_south', 0.0, -hy - t / 2, 0, self.size_x + 2 * t),
            ('boundary_east', hx + t / 2, 0.0, 90, self.size_y),
            ('boundary_west', -hx - t / 2, 0.0, 90, self.size_y),
        ]
        self.obstacles = [
            _box(name, 'wall', {'x': x, 'y': y, 'yaw_deg': yaw, 'length': length,
                                'thickness': t, 'height': wall_height})
            for name, x, y, yaw, length in boundary
        ]

        parsers = {
            'wall': lambda n, o: _box(n, 'wall', o),
            'box': lambda n, o: _box(n, 'box', o),
            'ramp': _ramp,
            'hill': _hill,
        }
        for index, o in enumerate(spec.get('obstacles', [])):
            kind = o['type']
            if kind not in parsers:
                raise ValueError(f'obstacle {index}: unknown type {kind!r}')
            self.obstacles.append(parsers[kind](o.get('name', f'{kind}_{index}'), o))

        names = [o.name for o in self.obstacles]
        if len(set(names)) != len(names):
            raise ValueError('obstacle names must be unique')
        if self.occupied(self.start[0], self.start[1]):
            raise ValueError('the start position is inside an obstacle')

    def bounds(self, pad=0.0):
        """(xmin, xmax, ymin, ymax) of the arena interior, grown by ``pad``."""
        hx, hy = self.size_x / 2, self.size_y / 2
        return (-hx - pad, hx + pad, -hy - pad, hy + pad)

    def occupied(self, x, y):
        """Whether each point is inside an obstacle or outside the arena."""
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        occ = (np.abs(x) > self.size_x / 2) | (np.abs(y) > self.size_y / 2)
        for obstacle in self.obstacles:
            occ = occ | obstacle.footprint.contains(x, y)
        return occ

    def world_sdf(self, name='s1_course'):
        length, width, height = self.vehicle
        x, y, yaw = self.start
        mass = 10.0
        ixx = mass * (width ** 2 + height ** 2) / 12
        iyy = mass * (length ** 2 + height ** 2) / 12
        izz = mass * (length ** 2 + width ** 2) / 12
        ground = f'{self.size_x + 6} {self.size_y + 6}'
        body = f'<box><size>{length} {width} {height}</size></box>'
        models = ''.join(o.sdf for o in self.obstacles)
        return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{name}">
    <physics name="default" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
{_GUI}
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{ground}</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{ground}</size></plane></geometry>
          {_material((0.72, 0.68, 0.6))}
        </visual>
      </link>
    </model>
{models}

    <model name="vehicle">
      <pose>{_fmt((x, y, height / 2 + 0.002, 0, 0, yaw))}</pose>
      <link name="base_link">
        <inertial>
          <mass>{mass}</mass>
          <inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="collision">
          <geometry>{body}</geometry>
          <surface><friction><ode><mu>0.1</mu><mu2>0.1</mu2></ode></friction></surface>
        </collision>
        <visual name="body"><geometry>{body}</geometry>{_material((0.8, 0.15, 0.1))}</visual>
        <visual name="nose">
          <pose>{length / 2 - 0.06} 0 {height / 2 + 0.011} 0 0 0</pose>
          <geometry><box><size>0.1 {width * 0.8} 0.02</size></box></geometry>
          {_material((1.0, 0.85, 0.1))}
        </visual>
        <visual name="lidar_mount">
          <pose>0 0 {height / 2 + 0.03} 0 0 0</pose>
          <geometry><cylinder><radius>0.05</radius><length>0.06</length></cylinder></geometry>
          {_material((0.1, 0.1, 0.1))}
        </visual>
        <sensor name="lidar" type="gpu_lidar">
          <pose>0 0 {height / 2 + 0.08} 0 0 0</pose>
          <topic>scan</topic>
          <gz_frame_id>base_link</gz_frame_id>
          <update_rate>10</update_rate>
          <always_on>true</always_on>
          <visualize>false</visualize>
          <lidar>
            <scan>
              <horizontal>
                <samples>360</samples>
                <resolution>1</resolution>
                <min_angle>-3.14159</min_angle>
                <max_angle>3.12414</max_angle>
              </horizontal>
            </scan>
            <range><min>0.35</min><max>8.0</max><resolution>0.01</resolution></range>
          </lidar>
        </sensor>
      </link>
      <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl"/>
      <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
        <odom_frame>odom</odom_frame>
        <robot_base_frame>base_link</robot_base_frame>
        <odom_publish_frequency>30</odom_publish_frequency>
        <dimensions>2</dimensions>
      </plugin>
    </model>
  </world>
</sdf>
'''


def _floating(name):
    return f'''
      <plugin filename="{name}" name="{name}">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>'''


# A camera looking down on the whole arena from the south, and the plugins a
# <gui> block has to name once it replaces the default layout. MarkerManager
# is what serves the /marker service the navigator draws its plan with.
_GUI = f'''
    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.4 0.4 0.4</ambient_light>
        <background_color>0.75 0.82 0.9</background_color>
        <camera_pose>0 -21 23 0 0.9 1.5708</camera_pose>
      </plugin>
{''.join(_floating(n) for n in ("GzSceneManager", "InteractiveViewControl", "CameraTracking",
                                "MarkerManager", "SelectEntities", "VisualizationCapabilities"))}
      <plugin filename="WorldControl" name="World control">
        <gz-gui>
          <title>World control</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">72</property>
          <property type="double" key="width">121</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="left" target="left"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <play_pause>true</play_pause>
        <step>true</step>
        <start_paused>false</start_paused>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <gz-gui>
          <title>World stats</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">110</property>
          <property type="double" key="width">290</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <sim_time>true</sim_time>
        <real_time>true</real_time>
        <real_time_factor>true</real_time_factor>
        <iterations>true</iterations>
      </plugin>
    </gui>'''


def load(path):
    with open(path) as f:
        return Course(yaml.safe_load(f))
