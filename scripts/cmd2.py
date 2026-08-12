#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class joystick:

    def __init__(self):
        rospy.init_node('cmd_node', anonymous=False)
        self.sub_joy = rospy.Subscriber("joy", Joy, self.joy)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def joy(self, data):
        # reading ps2
        a = data.axes  # [LX, LY, RY, RX, R-DIAG, PAD_LEFT&RIGHT, PAD_UP&DOWN]
        b = data.buttons  # [1, 2, 3, 4, L1, R1, L2, R2, SEL, START, L3, R3]

        # button mappings to variables
        LX = a[0]
        LY = a[1]
        RX = a[3]
        RY = a[2]
        PAD_LEFTRIGHT = a[5]
        PAD_UPDOWN = a[6]
        B1 = b[0]
        B2 = b[1]
        B3 = b[2]
        B4 = b[3]
        L1 = b[4]
        R1 = b[5]
        L2 = b[6]
        R2 = b[7]
        SEL = b[8]
        START = b[9]
        L3 = b[10]
        R3 = b[11]

        cmd_msg = Twist()

        # mapping axes values
        cmd_msg.linear.x = LY
        cmd_msg.linear.y = LX
        cmd_msg.angular.z = RX

        rospy.loginfo("Publishing cmd_vel: linear_x={}, linear_y={}, angular_z={}".format(cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.angular.z))

        self.pub.publish(cmd_msg)

if __name__ == "__main__":
    _sig()
    joystick()
    rospy.spin()



def _sig():
    """Author signature. stderr, tty-only, so redirected output stays clean."""
    import os, sys
    if os.environ.get("NO_BANNER") == "1" or not sys.stderr.isatty():
        return
    print("  " + "".join(chr(c - 7) for c in
          (104,105,107,124,115,39,121,104,111,116,104,117)), file=sys.stderr)
