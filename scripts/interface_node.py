#!/usr/bin/python

import rospy
from interface_class import Interface

# Init of program
if __name__ == '__main__':

    rospy.init_node('to_interface', anonymous=True)

    rospy.loginfo("Node init")

    Interface()

    rospy.spin()