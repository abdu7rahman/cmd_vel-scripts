"""Waypoint following against odometry from the micro-ROS base board."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('cmd_vel_ros2'),
        'config', 'teleop_params.yaml')

    use_agent = LaunchConfiguration('use_agent')
    serial_port = LaunchConfiguration('serial_port')
    baud = LaunchConfiguration('baud')

    return LaunchDescription([
        DeclareLaunchArgument('use_agent', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud', default_value='115200'),

        Node(package='micro_ros_agent', executable='micro_ros_agent',
             name='micro_ros_agent', output='screen',
             condition=IfCondition(use_agent),
             arguments=['serial', '--dev', serial_port, '-b', baud]),

        Node(package='cmd_vel_ros2', executable='waypoint_navigator',
             name='waypoint_navigator', output='screen', parameters=[params]),

        Node(package='cmd_vel_ros2', executable='odom_echo',
             name='odom_echo', output='screen', parameters=[params]),
    ])
