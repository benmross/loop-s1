"""Gazebo with the S1 course, and the ROS bridge. No navigation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from loop_s1_sim.course import load


def _setup(context):
    share = get_package_share_directory('loop_s1_sim')
    course_file = LaunchConfiguration('course').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1')

    # The world is generated from the course file every launch, so the file
    # the planner reads is always the file the simulator was built from.
    world = os.path.join(os.path.expanduser('~'), '.ros', 'loop_s1_course.sdf')
    os.makedirs(os.path.dirname(world), exist_ok=True)
    with open(world, 'w') as f:
        f.write(load(course_file).world_sdf())

    gz_args = f'-r {world}' if gui else f'-r -s --headless-rendering {world}'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml'),
                     'use_sim_time': True}],
        output='screen')
    return [gazebo, bridge]


def generate_launch_description():
    default_course = os.path.join(get_package_share_directory('loop_s1_sim'), 'config', 'course.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('course', default_value=default_course),
        DeclareLaunchArgument('gui', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
