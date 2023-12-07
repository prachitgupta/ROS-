#!/usr/bin/env python3
import rospy

if __name__ == '__main__':
    rospy.init_node("prachit")
    rospy.loginfo("hello")

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo("u did it bro")
        rate.sleep()

