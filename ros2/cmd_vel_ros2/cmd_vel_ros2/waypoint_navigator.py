#!/usr/bin/env python3
"""Drive to a list of (x, y, yaw) goals using odometry feedback.

ROS 2 port of scripts/nav_points.py and scripts/simple_planner.py, which were
the same turn-then-drive controller — simple_planner took one goal and stopped,
nav_points took three and re-prompted forever. Both behaviours are here:
`goals` sets the list, `loop_goals` decides whether reaching the end re-prompts
or shuts down.

The controller is unchanged in spirit: rotate until roughly facing the goal,
drive straight until within the position tolerance, then rotate to the final
heading.
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def normalize_angle(angle):
    """Wrap to [-pi, pi].

    The ROS 1 versions compared raw angle differences, so a robot at +170 deg
    heading for a -170 deg goal saw an error of 340 deg and turned most of the
    way around the long side instead of 20 deg the short way.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def euler_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        # Flat [x1, y1, yaw1_deg, x2, y2, yaw2_deg, ...]. Empty means prompt on
        # stdin, which is what the ROS 1 scripts always did.
        self.declare_parameter('goals', [0.0])
        self.declare_parameter('loop_goals', False)
        self.declare_parameter('position_tolerance', 0.25)
        self.declare_parameter('heading_tolerance', 0.1)
        self.declare_parameter('final_yaw_tolerance', 0.2)
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.15)
        self.declare_parameter('control_rate', 30.0)

        p = self.get_parameter
        self.loop_goals = p('loop_goals').value
        self.position_tolerance = p('position_tolerance').value
        self.heading_tolerance = p('heading_tolerance').value
        self.final_yaw_tolerance = p('final_yaw_tolerance').value
        self.linear_speed = p('linear_speed').value
        self.angular_speed = p('angular_speed').value

        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.x_current = 0.0
        self.y_current = 0.0
        self.yaw_current = 0.0

        self.goals = self._parse_goals(p('goals').value)
        if not self.goals:
            self.goals = self._prompt_for_goals(1)

        self.get_logger().info(f'{len(self.goals)} goal(s): {self.goals}')

        self.index = 0
        self.target = self.goals[0]
        self.settling = False
        self.settle_deadline = None

        self.create_timer(1.0 / p('control_rate').value, self.control_step)

    def _parse_goals(self, flat):
        """Group the flat parameter array into (x, y, yaw_radians) triples."""
        if not flat or len(flat) < 3:
            return []
        if len(flat) % 3 != 0:
            self.get_logger().warn(
                f'goals has {len(flat)} entries, not a multiple of 3; '
                'ignoring the trailing values')
        triples = []
        for i in range(0, len(flat) - len(flat) % 3, 3):
            triples.append((flat[i], flat[i + 1], math.radians(flat[i + 2])))
        return triples

    def _prompt_for_goals(self, count):
        goals = []
        for i in range(count):
            x = float(input(f'Enter x for goal {i + 1}: '))
            y = float(input(f'Enter y for goal {i + 1}: '))
            yaw = float(input(f'Enter yaw (degrees) for goal {i + 1}: '))
            goals.append((x, y, math.radians(yaw)))
        return goals

    def odom_callback(self, msg):
        self.x_current = msg.pose.pose.position.x
        self.y_current = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw_current = euler_from_quaternion(q.x, q.y, q.z, q.w)

    def _advance_goal(self):
        self.index += 1
        if self.index < len(self.goals):
            self.target = self.goals[self.index]
            self.get_logger().info(f'next goal: {self.target}')
            return True

        if self.loop_goals:
            # nav_points.py re-prompted for a fresh batch here.
            self.goals = self._prompt_for_goals(len(self.goals))
            self.index = 0
            self.target = self.goals[0]
            return True

        self.get_logger().info('all goals reached')
        self.cmd_vel_pub.publish(Twist())
        rclpy.shutdown()
        return False

    def control_step(self):
        # The ROS 1 versions called time.sleep(1) inside the control loop to
        # settle at a goal, which blocked the whole node including odometry
        # callbacks. A deadline lets the node keep spinning.
        if self.settling:
            self.cmd_vel_pub.publish(Twist())
            if self.get_clock().now() >= self.settle_deadline:
                self.settling = False
                self._advance_goal()
            return

        x_target, y_target, yaw_target = self.target

        delta_x = x_target - self.x_current
        delta_y = y_target - self.y_current
        distance = math.hypot(delta_x, delta_y)

        cmd = Twist()

        if distance > self.position_tolerance:
            angle_to_goal = math.atan2(delta_y, delta_x)
            heading_error = normalize_angle(angle_to_goal - self.yaw_current)

            if abs(heading_error) > self.heading_tolerance:
                # nav_points.py hard-coded +0.15 here regardless of which way it
                # needed to turn, so it could only ever converge by going the
                # long way round. Sign follows the error.
                cmd.angular.z = math.copysign(self.angular_speed, heading_error)
            else:
                cmd.linear.x = self.linear_speed
        else:
            yaw_error = normalize_angle(yaw_target - self.yaw_current)
            if abs(yaw_error) > self.final_yaw_tolerance:
                cmd.angular.z = math.copysign(self.angular_speed, yaw_error)
            else:
                self.get_logger().info(f'reached goal {self.index + 1}')
                self.settling = True
                self.settle_deadline = (
                    self.get_clock().now()
                    + rclpy.duration.Duration(seconds=1.0))
                self.cmd_vel_pub.publish(Twist())
                return

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
