#!/usr/bin/env python

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from direct_sherlock import *


def callback(img):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    Detected_ArUco_markers = detect_ArUco(cv_image)	                                ## detecting ArUco ids and returning ArUco dictionary
    img, turn = mark_ArUco(cv_image,Detected_ArUco_markers)                         ## marking the parameters of aruco which will help sherlock to solve maze
    cv2.namedWindow("Image Window", 1)
    cv2.imshow("Image Window", img)
    k = cv2.waitKey(1)
    

def laser():
    rospy.Subscriber('/turtlebot3_burger/camera/image_raw', Image, callback)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('detect_marker')
    try:
        laser()

    except rospy.ROSInterruptException:
        pass
