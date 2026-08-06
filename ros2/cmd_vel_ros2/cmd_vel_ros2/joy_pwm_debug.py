#!/usr/bin/env python3
"""Print the PWM values a gamepad would produce, without publishing anything.

ROS 2 port of scripts/psfull.py and scripts/psarduino.py, which were the same
node with the same behaviour — read /joy, map the sticks onto 0-255, print. Use
it to check axis mapping and deadzones before wiring a board up.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy


def translate(value, left_min, left_max, right_min, right_max):
    left_span = left_max - left_min
    right_span = right_max - right_min
    scaled = float(value - left_min) / float(left_span)
    return right_min + (scaled * right_span)


class JoyPwmDebug(Node):

    def __init__(self):
        super().__init__('joy_pwm_debug')

        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_linear_y', 0)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('pwm_max', 255)

        p = self.get_parameter
        self.axis_x = p('axis_linear_x').value
        self.axis_y = p('axis_linear_y').value
        self.deadzone = p('deadzone').value
        self.pwm_max = p('pwm_max').value

        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.get_logger().info('joy_pwm_debug up, printing stick -> PWM mapping')

    def _axis(self, axes, index):
        if index < 0 or index >= len(axes):
            return 0.0
        return axes[index]

    def on_joy(self, data):
        lx = self._axis(data.axes, self.axis_y)
        ly = self._axis(data.axes, self.axis_x)

        top = self.pwm_max
        channels = {
            'FORWARD': translate(ly, self.deadzone, 1.0, 0, top),
            'BACK': translate(ly, -self.deadzone, -1.0, 0, top),
            'LEFT': translate(lx, self.deadzone, 1.0, 0, top),
            'RIGHT': translate(lx, -self.deadzone, -1.0, 0, top),
        }
        channels = {k: int(min(max(v, 0), top)) for k, v in channels.items()}

        active = {k: v for k, v in channels.items() if v > 0}
        if active:
            self.get_logger().info(
                '  '.join(f'{k}={v}' for k, v in active.items()))


def main(args=None):
    rclpy.init(args=args)
    node = JoyPwmDebug()
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
