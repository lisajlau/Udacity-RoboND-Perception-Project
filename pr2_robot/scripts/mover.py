#!/usr/bin/env python

# Import modules
import rospy
import math
from std_msgs.msg import Float64
from pr2_robot.srv import *

def mover():

    # ROS node initialization
    rospy.init_node('mover', anonymous=True)

    pr2_joint_pub = rospy.Publisher('/pr2/world_joint_controller/command',
                             Float64, queue_size= 1)

    pr2_joint_pub.publish(math.pi/2)

if __name__ == '__main__':

    try:
        mover()
    except rospy.ROSInterruptException:
        pass
