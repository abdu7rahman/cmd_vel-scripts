#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy


class joystick:

    def __init__(self):
        rospy.init_node('swerve_node', anonymous=False)
        self.sub_joy = rospy.Subscriber("joy", Joy, self.joy)

    def joy(self, data):
        #reading ps2
        a = data.axes           #[LX,LY,RY,RX,R-DIAG, PAD_LEFT&RIGHT,PAD_UP&DOWN]
        b = data.buttons        #[1,2,3,4,L1,R1,L2,R2,SEL,START,L3,R3]

        #button mappings to variables
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

        # initialize = 0
        # if (initialize==0):  #PS4 error
        #     ef = LY - 0
        #     er = LX - 0
        #     initialize = 1

        #mapping axes values
        f = int(translate(self, LY, 0.1, 1, 0, 255))
        b = int(translate(self, LY, -0.1, -1, 0, 255))
        l = int(translate(self, LX, 0.1, 1, 0, 255)) #err 28-pwm
        r = int(translate(self, LX, -0.1, -1, 0, 255)) #err 28-pwm

        #rospy.loginfo(LX)
        f = sorted([0, f, 255])[1]
        b = sorted([0, b, 255])[1]
        l = sorted([0, l, 255])[1]
        r = sorted([0, r, 255])[1]

        if  (l>0):                 #(LX > 0.1):
            print("LX-LEFT", l)
        if (r>0):                  #(LX < 0.2):
            print("LX-RIGHT", r)
        if (f>0):                  #(LY > 0.1):
            print("LY-FORW", f)
        if (b>0):                  #(LY < 0.2):
            print("LY-BACK", b)

        #print(f,b,r,l)

if __name__ == "__main__":
    _sig()

    def translate(self, value, leftMin, leftMax, rightMin, rightMax):
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin
        valueScaled = float(value - leftMin) / float(leftSpan)
        return rightMin + (valueScaled * rightSpan)

    enjoy = joystick()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        #rospy.loginfo(LX)
        rate.sleep()


def _sig():
    """Author signature. stderr, tty-only, so redirected output stays clean."""
    import os, sys
    if os.environ.get("NO_BANNER") == "1" or not sys.stderr.isatty():
        return
    print("  " + "".join(chr(c - 7) for c in
          (104,105,107,124,115,39,121,104,111,116,104,117)), file=sys.stderr)
