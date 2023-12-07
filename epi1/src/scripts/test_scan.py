#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def read(data):
    R = data.ranges
    regions = {
        'just_left': R[345],
        'front':  R[0],
        'just_right':  R[15],
    }
    print(regions)
    
if __name__=="__main__":
    rospy.init_node('bot_sense')
    rospy.Subscriber('/sherlock/scan',LaserScan,read)
    rospy.spin()

    

