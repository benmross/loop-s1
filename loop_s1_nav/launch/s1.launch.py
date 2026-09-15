"""The whole S1 run: Gazebo, the bridge, the waypoint manager and the navigator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_share = FindPackageShare('loop_s1_nav')
    sim_share = FindPackageShare('loop_s1_sim')
    course = LaunchConfiguration('course')
    params = PathJoinSubstitution([nav_share, 'config', 'nav.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('course', default_value=PathJoinSubstitution(
            [sim_share, 'config', 'course.yaml'])),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI, or headless'),
        DeclareLaunchArgument('rviz', default_value='false', description='RViz with the costmap and plan'),
        DeclareLaunchArgument('seed', default_value='-1', description='waypoint seed; -1 picks one'),
        DeclareLaunchArgument('waypoints', default_value='6', description='how many waypoints'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([sim_share, 'launch', 'sim.launch.py'])),
            launch_arguments={'course': course, 'gui': LaunchConfiguration('gui')}.items()),

        Node(package='loop_s1_nav', executable='navigator', output='screen',
             parameters=[params, {'course_file': course}]),
        Node(package='loop_s1_nav', executable='waypoint_manager', output='screen',
             parameters=[params, {'course_file': course,
                                  'seed': LaunchConfiguration('seed'),
                                  'num_waypoints': LaunchConfiguration('waypoints')}]),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', PathJoinSubstitution([nav_share, 'rviz', 's1.rviz'])],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
