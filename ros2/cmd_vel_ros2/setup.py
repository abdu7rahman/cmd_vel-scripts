from setuptools import setup
import os
from glob import glob

package_name = 'cmd_vel_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abdu7rahman',
    maintainer_email='mohammedabdulr.1@northeastern.edu',
    description='Teleop and waypoint-following nodes that drive a base over /cmd_vel.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_twist = cmd_vel_ros2.joy_twist:main',
            'joy_pwm_array = cmd_vel_ros2.joy_pwm_array:main',
            'joy_pwm_debug = cmd_vel_ros2.joy_pwm_debug:main',
            'keyboard_teleop = cmd_vel_ros2.keyboard_teleop:main',
            'waypoint_navigator = cmd_vel_ros2.waypoint_navigator:main',
            'odom_echo = cmd_vel_ros2.odom_echo:main',
            'swerve_controller = cmd_vel_ros2.swerve_controller:main',
        ],
    },
)
