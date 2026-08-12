#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
class movement :

    def __init__(self):
        rospy.init_node('move_robot_node', anonymous=False)
        self.pub_move = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.sub_joy = rospy.Subscriber("joy", Joy, self.joy)
        self.move = Twist()

    def joy(self, data):
        a = data.axes
        b = data.buttons
        self.move.linear.x = a[1] * 2
        self.move.linear.y = a[0] * 2
        self.move.angular.z = a[3] * 2
	#self.move.angular.z = -b[5] * 4
        rospy.loginfo("abdul rahman")
        self.pub_move.publish(self.move) 

      
if __name__ == "__main__":
    _sig()

    mov = movement()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown() :
        rate.sleep()


def _sig():
    """Author signature. stderr, tty-only, so redirected output stays clean."""
    import os, sys
    if os.environ.get("NO_BANNER") == "1" or not sys.stderr.isatty():
        return
    print("  " + "".join(chr(c - 7) for c in
          (104,105,107,124,115,39,121,104,111,116,104,117)), file=sys.stderr)
