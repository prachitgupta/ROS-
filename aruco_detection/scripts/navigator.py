#!/usr/bin/env python
from logging import info
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from direct_sherlock import *


class AutoNavigator:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        self.scan_sub = rospy.Subscriber('scan',LaserScan,self.scan_callback)
        self.dectect_marker = rospy.Subscriber('/turtlebot3_burger/camera/image_raw', Image, self.detect_callback)
        self.node = rospy.init_node('navigator')
        self.command = Twist()
        self.command.linear.x = 0
        self.command.angular.z = 0
        self.rate = rospy.Rate(10)
        self.near_wall = False
        self.min_front = 1
        self.min_right = 1
        self.min_left = 1
        self.min_range = 1
        self.direction = "left"
        self.turn = "null"

    def scan_callback(self, msg):
        allranges = msg.ranges
        frontal = allranges[0:5] + allranges[-1:-5:-1]
        rightside = allranges[300:345]
        leftside = allranges[15:60]
        self.min_left = min(leftside)
        self.min_right = min(rightside)
        self.min_front = min(frontal)
        self.min_range = min(self.min_left,self.min_front,self.min_right)

    def detect_callback(self, img):
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(img, "bgr8")       # convering imahe to CV2 usable format
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e)) 
        
        Detected_ArUco_markers = {}
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)    		# converting image to grayscale
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)	# extracting a pre-defined dictionary of various aruco markers
        parameters = aruco.DetectorParameters_create()		# returns the parameters required by opencv to detect aruco markers. Leave it at default
        
        # get the coordinates of all the aruco markers in the scene along with their ID
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters) 
        i = 0
        
        # storing extracted data
        try:
            for id in ids:
                for id_Number in id:
                    Detected_ArUco_markers[id_Number] = corners[i][0]    

        except TypeError:
            print("No aruco in front of me")

        i += 1

        IDs = Detected_ArUco_markers.keys()   # IDs of all aruco markers in the scene
        #print(Detected_ArUco_markers)
        centre_aruco = {}
        top_centre = {}
        
        # drawing circles and line for each aruco marker by using coordinates found earlier
        try:
            for id in IDs:
                corners = Detected_ArUco_markers[id]
                for i in range(0, 4):
                    cv2.circle(cv_image,(int(corners[i][0]), int(corners[i][1])), 5, (0,0,255), -1)
                centre_aruco[id] = (corners[0]+corners[1]+corners[2]+corners[3])/4
                top_centre[id] = (corners[0]+corners[1])/2
                cv2.line(cv_image, (int(centre_aruco[id][0]), int(centre_aruco[id][1])),
                    (int(top_centre[id][0]), int(top_centre[id][1])), (255, 0, 0), 5)
                if centre_aruco[id][0] >= top_centre[id][0]:
                    self.turn = "left"
                else:
                    self.turn = "right"

        except TypeError:
            print("No aruco in front of me")
            self.turn = "null"
        
        cv2.imshow("prachit",cv_image)
        k = cv2.waitKey(1)	
        if k == "z":
            cv2.destroyAllWindows()
        ##############
        ## Here, you have to detect ArUcos and 
        ## mark the ArUco with notations, 
        ## similar to the one done in week and then display it.
        ## Line joining centre and top_centre must be displayed.
        ## Input: Image data detected by camera sensor
        ## Output : Display marked ArUcos 
        ####################################                             
        

        if self.turn == "left":
            self.direction = "left"
        elif self.turn == "right":
           self.direction = "right"


    def run(self):
        while not rospy.is_shutdown():
            ############################
            if self.direction == "left":
                self.right_wall_follow()
            elif self.direction == "right":
                self.left_wall_follow()
           
    def left_wall_follow(self):
        ###############################
        ## Follow left hand side wall of robot. 
        ## You can take help from navigator.py file of week3.
        ## You may need to change various values 
        ## form that file to get correct results.
        #################################
        if (self.min_front > 0.2):
            if (self.min_left < 0.12):
                self.command.angular.z = -1.2
                self.command.linear.x = -0.1
                
            #If too far from wall, go near wall
            elif self.min_left > 0.15:
                self.command.angular.z = 1.2
                self.command.linear.x = 0.15
                
            # If not too far, then go away from wall
            else:
                
                self.command.angular.z = -1.2
                self.command.linear.x = 0.15
                
        # If there's an obstacle in the front, rotate clockwise until that obstacle becomes the left wall.
        else:
            
            self.command.angular.z = -1.0
            self.command.linear.x = 0.0
            self.cmd_vel_pub.publish(self.command)

        self.cmd_vel_pub.publish(self.command)
    
       
    def right_wall_follow(self):
        ###############################
        ## Follow right hand side wall of robot. 
        ## You can take help from navigator.py file of week3.
        ## You may need to change various values 
        ## form that file to get correct results.
        #################################
        if (self.min_front > 0.2):
            if (self.min_right < 0.12):
                print("Range: {:.2f}m - Too close. Backing up.".format(self.min_right))
                self.command.angular.z = 1.2
                self.command.linear.x = -0.1
                
            #If too far from wall, go near wall
            elif self.min_right > 0.15:
                print("Range: {:.2f}m - Wall-following; turn left.".format(self.min_right))
                self.command.angular.z = -1.2
                self.command.linear.x = 0.15
                
            # If not too far, then go away from wall
            else:
                print("Range: {:.2f}m - Wall-following; turn right.".format(self.min_right))
                self.command.angular.z = 1.2
                self.command.linear.x = 0.15
                
        # If there's an obstacle in the front, rotate clockwise until that obstacle becomes the right wall.
        else:
            print("Front obstacle detected. Turning away.")
            self.command.angular.z = 1.0
            self.command.linear.x = 0.0
            self.cmd_vel_pub.publish(self.command)

        self.cmd_vel_pub.publish(self.command)
    
       


if __name__=='__main__':
    navigator = AutoNavigator()
    navigator.run()