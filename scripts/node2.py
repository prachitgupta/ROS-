#!/usr/bin/env python3
import rospy

from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard %s", data.data)


if __name__ == '__main__':
    rospy.init_node("listner")
    rospy.loginfo("node started")
    rospy.Subscriber("my_topic",String, callback)
    rospy.spin()