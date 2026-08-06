"""Gamepad teleop: joy_node -> joy_twist -> /cmd_vel -> micro-ROS drive board."""

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

        Node(package='joy', executable='joy_node', name='joy_node',
             output='screen', parameters=[params]),

        Node(package='cmd_vel_ros2', executable='joy_twist', name='joy_twist',
             output='screen', parameters=[params]),

        Node(package='micro_ros_agent', executable='micro_ros_agent',
             name='micro_ros_agent', output='screen',
             condition=IfCondition(use_agent),
             arguments=['serial', '--dev', serial_port, '-b', baud]),
    ])
