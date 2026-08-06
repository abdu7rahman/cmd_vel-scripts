#!/usr/bin/env python3
"""Gamepad -> /cmd_vel_pwm as four one-sided PWM magnitudes.

ROS 2 port of scripts/cmd.py. Each stick axis is split into two non-negative
channels — left/right from LX, forward/back from LY — which is the format the
ps4_receive_microros sketch expects.

The ROS 1 version published this on `cmd_vel`, which collides with the
conventional geometry_msgs/Twist meaning of that name; anything else on the
graph subscribing to /cmd_vel would fail to deserialise. It publishes on
`cmd_vel_pwm` here.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Int64MultiArray


def translate(value, left_min, left_max, right_min, right_max):
    """Map value from one range onto another. Unchanged from the ROS 1 helper."""
    left_span = left_max - left_min
    right_span = right_max - right_min
    scaled = float(value - left_min) / float(left_span)
    return right_min + (scaled * right_span)


class JoyPwmArray(Node):

    def __init__(self):
        super().__init__('joy_pwm_array')

        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_linear_y', 0)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('pwm_max', 50)
        self.declare_parameter('publish_rate', 20.0)

        p = self.get_parameter
        self.axis_x = p('axis_linear_x').value
        self.axis_y = p('axis_linear_y').value
        self.deadzone = p('deadzone').value
        self.pwm_max = p('pwm_max').value

        self.pub = self.create_publisher(Int64MultiArray, 'cmd_vel_pwm', 10)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)

        self.channels = [0, 0, 0, 0]

        # The ROS 1 version constructed a Publisher inside the callback on every
        # message and called publish() up to four times per joy message.
        # Published once per tick here.
        self.create_timer(1.0 / p('publish_rate').value, self.publish_channels)

        self.get_logger().info('joy_pwm_array up, publishing on cmd_vel_pwm')

    def _axis(self, axes, index):
        if index < 0 or index >= len(axes):
            return 0.0
        return axes[index]

    def on_joy(self, data):
        lx = self._axis(data.axes, self.axis_y)
        ly = self._axis(data.axes, self.axis_x)

        top = self.pwm_max
        forward = translate(ly, self.deadzone, 1.0, 0, top)
        backward = translate(ly, -self.deadzone, -1.0, 0, top)
        left = translate(lx, self.deadzone, 1.0, 0, top)
        right = translate(lx, -self.deadzone, -1.0, 0, top)

        # Clamp each channel into [0, pwm_max]; below the deadzone translate()
        # goes negative, which is what makes the channels one-sided.
        self.channels = [
            int(min(max(v, 0), top)) for v in (left, right, forward, backward)
        ]

    def publish_channels(self):
        msg = Int64MultiArray()
        msg.data = self.channels
        self.pub.publish(msg)

    def stop(self):
        """Send a zero command so the board does not hold its last throttle."""
        msg = Int64MultiArray()
        msg.data = [0, 0, 0, 0]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyPwmArray()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # The ROS 1 script had an unreachable `while rospy.is_shutdown()` block
        # meant to zero the outputs on exit. This actually runs.
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
