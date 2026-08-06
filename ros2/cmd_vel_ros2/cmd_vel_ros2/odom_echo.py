#!/usr/bin/env python3
"""Print pose from /odom. ROS 2 port of scripts/coordinates.py.

`tf.transformations` does not exist in ROS 2, so the quaternion-to-Euler
conversion is done inline rather than imported.
"""

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


def euler_from_quaternion(x, y, z, w):
    """Return (roll, pitch, yaw) in radians."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class OdomEcho(Node):

    def __init__(self):
        super().__init__('odom_echo')
        self.declare_parameter('show_orientation', False)
        self.show_orientation = self.get_parameter('show_orientation').value
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.get_logger().info('odom_echo up, listening on /odom')

    def on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        line = f'position (x, y): ({x:.3f}, {y:.3f})'

        if self.show_orientation:
            q = msg.pose.pose.orientation
            roll, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
            line += (f'  rpy: ({math.degrees(roll):.1f}, '
                     f'{math.degrees(pitch):.1f}, {math.degrees(yaw):.1f}) deg')

        self.get_logger().info(line, throttle_duration_sec=0.2)


def main(args=None):
    rclpy.init(args=args)
    node = OdomEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
