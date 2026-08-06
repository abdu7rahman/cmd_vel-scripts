#!/usr/bin/env python3
"""Gamepad -> /cmd_vel as a Twist. ROS 2 port of scripts/ps.py.

Holonomic: left stick drives x and y, right stick X drives yaw.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class JoyTwist(Node):

    def __init__(self):
        super().__init__('joy_twist')

        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_linear_y', 0)
        self.declare_parameter('axis_angular_z', 3)
        self.declare_parameter('scale_linear', 2.0)
        self.declare_parameter('scale_angular', 2.0)
        self.declare_parameter('deadzone', 0.1)

        p = self.get_parameter
        self.axis_x = p('axis_linear_x').value
        self.axis_y = p('axis_linear_y').value
        self.axis_z = p('axis_angular_z').value
        self.scale_linear = p('scale_linear').value
        self.scale_angular = p('scale_angular').value
        self.deadzone = p('deadzone').value

        self.pub_move = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)

        self.get_logger().info('joy_twist up, waiting for /joy')

    def _axis(self, axes, index):
        # The ROS 1 scripts indexed axes[] unguarded, so a pad reporting fewer
        # axes than a PS4 raised IndexError inside the callback.
        if index < 0 or index >= len(axes):
            return 0.0
        value = axes[index]
        return 0.0 if abs(value) < self.deadzone else value

    def on_joy(self, data):
        move = Twist()
        move.linear.x = self._axis(data.axes, self.axis_x) * self.scale_linear
        move.linear.y = self._axis(data.axes, self.axis_y) * self.scale_linear
        move.angular.z = self._axis(data.axes, self.axis_z) * self.scale_angular
        self.pub_move.publish(move)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTwist()
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
