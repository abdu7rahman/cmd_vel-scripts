#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from math import atan2, radians
import time

class NavPoints():
    def __init__(self):
        rospy.init_node('nav_points')
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd_vel = Twist()

        self.x_goal = []
        self.y_goal = []
        self.yaw_goal = []

        for p in range(0, 3):
            x_input = float(input("Enter 4 x points: "))
            self.x_goal.append(x_input)
            y_input = float(input("Enter 4 y points: "))
            self.y_goal.append(y_input)
            yaw_input = float(input("Enter 4 yaw angles: "))
            self.yaw_goal.append(yaw_input)
        rospy.loginfo(self.x_goal)
        rospy.loginfo(self.y_goal)
        rospy.loginfo(self.yaw_goal)
        self.x_current = 0
        self.y_current = 0
        self.yaw_current = 0
        self.x_target = 0
        self.y_target = 0
        self.yaw_target = 0
        self.i = 0
        self.result = True

    def spin(self):
        self.r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.sendgoal()
            self.r.sleep()
    def sendgoal(self):

        #rospy.loginfo(self.result)
        list_len = len(self.x_goal)
        if(self.result):
            #curr_idx = self.x_goal.index(self.i)
            #list_end = list_len - curr_idx
            if(self.i<list_len):
            #if list_end != 1:
                self.x_target = self.x_goal[self.i]
                self.y_target = self.y_goal[self.i]
                self.yaw_target = self.yaw_goal[self.i]
                self.i+=1
            else:
                for q in range(0, 3):
                    x_input = float(input("Enter next 4 x points: "))
                    self.x_goal[q] = (x_input)
                    y_input = float(input("Enter next 4 y points: "))
                    self.y_goal[q] = (y_input)
                    yaw_input = float(input("Enter next 4 yaw angles: "))
                    self.yaw_goal[q] = (yaw_input)
                self.i = 0
            self.result = False
        goal = Point()
        goal.x = self.x_target
        goal.y = self.y_target
        #rospy.loginfo(self.x_goal[self.i])
        #rospy.loginfo(self.y_goal[self.i])
        #rospy.loginfo(self.yaw_goal[self.i])
        #rospy.loginfo(goal)

        #while not rospy.is_shutdown():
        delta_x = goal.x - self.x_current
        delta_y = goal.y - self.y_current
        angle_to_goal = atan2(delta_y, delta_x)
        if((abs(self.x_current - goal.x) > 0.25) or (abs(self.y_current - goal.y) > 0.25)):
            if abs(angle_to_goal - self.yaw_current) > 0.1:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.15
            else:
                self.cmd_vel.linear.x = 0.3
                self.cmd_vel.angular.z = 0.0
        else:
            if (radians(self.yaw_target) - (self.yaw_current)) > 0.1:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.15
            elif (radians(self.yaw_target) - (self.yaw_current)) < -0.1:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = -0.15
            else:
                self.cmd_vel = Twist()
                self.cmd_vel_pub.publish(self.cmd_vel)
                time.sleep(1)
                self.result = True
                rospy.loginfo("REACHED")

        self.cmd_vel_pub.publish(self.cmd_vel)


    def odom_callback(self, msg):
        self.x_current = msg.pose.pose.position.x
        self.y_current = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.yaw_current) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        #rospy.loginfo(self.yaw_current)



if __name__ == '__main__':
    _sig()
    try:
        navpoints = NavPoints()
        navpoints.spin()
    except rospy.ROSInterruptException:
        pass


def _sig():
    """Author signature. stderr, tty-only, so redirected output stays clean."""
    import os, sys
    if os.environ.get("NO_BANNER") == "1" or not sys.stderr.isatty():
        return
    print("  " + "".join(chr(c - 7) for c in
          (104,105,107,124,115,39,121,104,111,116,104,117)), file=sys.stderr)
