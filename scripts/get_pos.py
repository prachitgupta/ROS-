#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose

def callback(msg):
    rospy.loginfo(f"( {msg.x} , {msg.y} )")

if __name__ == '__main__':
    rospy.init_node("get_pos")
    rospy.loginfo("common!! prachit")
    #define sub
    sub = rospy.Subscriber("/turtle1/pose",Pose,callback)

    rospy.spin()