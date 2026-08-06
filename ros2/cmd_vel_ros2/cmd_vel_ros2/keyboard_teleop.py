#!/usr/bin/env python3
"""Keyboard -> /cmd_vel. ROS 2 port of scripts/keyboard.py.

Same key bindings as the ROS 1 teleop_twist_keyboard the original was based on.
"""

import sys
import select
import termios
import threading
import tty

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


BANNER = """
Reading from the keyboard and publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

MOVE_BINDINGS = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

SPEED_BINDINGS = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}


class PublishThread(threading.Thread):
    """Republish the latest command at a fixed rate on a background thread."""

    def __init__(self, node, rate):
        super().__init__()
        self.node = node
        self.publisher = node.create_publisher(Twist, 'cmd_vel', 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False
        self.timeout = (1.0 / rate) if rate != 0.0 else None
        self.start()

    def wait_for_subscribers(self):
        i = 0
        while rclpy.ok() and self.publisher.get_subscription_count() == 0:
            if i == 4:
                self.node.get_logger().info(
                    'waiting for a subscriber on cmd_vel')
            self.node.get_clock().sleep_for(
                rclpy.duration.Duration(seconds=0.5))
            i = (i + 1) % 5
        if not rclpy.ok():
            raise RuntimeError('shutdown requested before subscribers connected')

    def update(self, x, y, z, th, speed, turn):
        with self.condition:
            self.x = x
            self.y = y
            self.z = z
            self.th = th
            self.speed = speed
            self.turn = turn
            self.condition.notify()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while self.done is False:
            with self.condition:
                self.condition.wait(self.timeout)
                twist.linear.x = float(self.x * self.speed)
                twist.linear.y = float(self.y * self.speed)
                twist.linear.z = float(self.z * self.speed)
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = float(self.th * self.turn)
            self.publisher.publish(twist)

        self.publisher.publish(Twist())


def get_key(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = rclpy.create_node('keyboard_teleop')

    # ROS 1 private params (~speed) become declared node parameters.
    node.declare_parameter('speed', 0.5)
    node.declare_parameter('turn', 1.0)
    node.declare_parameter('repeat_rate', 0.0)
    node.declare_parameter('key_timeout', 0.0)

    speed = node.get_parameter('speed').value
    turn = node.get_parameter('turn').value
    repeat = node.get_parameter('repeat_rate').value
    key_timeout = node.get_parameter('key_timeout').value or None

    pub_thread = PublishThread(node, repeat)

    x = y = z = th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(BANNER)
        print(f'currently:\tspeed {speed}\tturn {turn}')

        while True:
            key = get_key(settings, key_timeout)
            if key in MOVE_BINDINGS:
                x, y, z, th = MOVE_BINDINGS[key]
            elif key in SPEED_BINDINGS:
                speed *= SPEED_BINDINGS[key][0]
                turn *= SPEED_BINDINGS[key][1]
                print(f'currently:\tspeed {speed}\tturn {turn}')
                if status == 14:
                    print(BANNER)
                status = (status + 1) % 15
            else:
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = y = z = th = 0
                if key == '\x03':  # Ctrl-C
                    break

            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as exc:  # noqa: BLE001 - mirror the ROS 1 behaviour
        print(exc)

    finally:
        pub_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
