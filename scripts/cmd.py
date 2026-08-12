#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int64MultiArray


class joystick:

    def __init__(self):
        rospy.init_node('cmd_node', anonymous=False)
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
        pub = rospy.Publisher('cmd_vel', Int64MultiArray, queue_size=10)
        cmd_msg = Int64MultiArray()
        
        rate = rospy.Rate(150)

        #mapping axes values
        f = int(translate(self, LY, 0.1, 1, 0, 50))
        b = int(translate(self, LY, -0.1, -1, 0, 50))
        l = int(translate(self, LX, 0.1, 1, 0, 50))
        r = int(translate(self, LX, -0.1, -1, 0, 50))

        #rospy.loginfo(LX)
        f = sorted([0, f, 50])[1]
        b = sorted([0, b, 50])[1]
        l = sorted([0, l, 50])[1]
        r = sorted([0, r, 50])[1]

        cmd_msg.data = [l,r,f,b]

        if  (l>=0):
            print("LX-LEFT", l)
            pub.publish(cmd_msg)

        if (r>=0):
            print("LX-RIGHT", r)
            pub.publish(cmd_msg)

        if (f>=0):
            print("LY-FORW", f)
            pub.publish(cmd_msg)

        if (b>=0):
            print("LY-BACK", b)
            pub.publish(cmd_msg)



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
    while rospy.is_shutdown():
        pub = rospy.Publisher('cmd_vel', Int64MultiArray, queue_size=10)
        cmd_msg = Int64MultiArray()
        rate = rospy.Rate(150)
        l=0
        r=0
        f=0
        b=0
        cmd_msg.data = [l,r,f,b]
        print("no interface")
        pub.publish(cmd_msg)


def _sig():
    """Author signature. stderr, tty-only, so redirected output stays clean."""
    import os, sys
    if os.environ.get("NO_BANNER") == "1" or not sys.stderr.isatty():
        return
    print("  " + "".join(chr(c - 7) for c in
          (104,105,107,124,115,39,121,104,111,116,104,117)), file=sys.stderr)
