## Proceeding to detect ArUco..

Open the Terminal and run following commands-
```bash
cd ~/catkin_ws/src
git clone https://github.com/Tejas2910/aruco_detection/tree/python3_noetic
cd ~/catkin_ws
catkin_make
```
Now you have a package aruco_detection, let's run it.
```bash
roslaunch aruco_detection maze_aruco.launch
```
Let's spwan the Turtlebot3 by running follwing command in another tab
```bash
roslaunch aruco_detection spawn_turtlebot3.launch
```
You can see ArUco marker in front of TurtleBot3(waffle_pi model).
Why we used waffle_pi ? Guess... Remember Investigation 1 of Episode 1. 

Yes, you guessed correctly. Let's check by executing ``` rostopic list ``` in another tab.
```
/clock
/cmd_vel
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/imu
/joint_states
/odom
/rosout
/rosout_agg
/scan
/tf
/turtlebot3_waffle_pi/camera/camera_info
/turtlebot3_waffle_pi/camera/image_raw
/turtlebot3_waffle_pi/camera/image_raw/compressed
/turtlebot3_waffle_pi/camera/image_raw/compressed/parameter_descriptions
/turtlebot3_waffle_pi/camera/image_raw/compressed/parameter_updates
/turtlebot3_waffle_pi/camera/image_raw/compressedDepth
/turtlebot3_waffle_pi/camera/image_raw/compressedDepth/parameter_descriptions
/turtlebot3_waffle_pi/camera/image_raw/compressedDepth/parameter_updates
/turtlebot3_waffle_pi/camera/image_raw/theora
/turtlebot3_waffle_pi/camera/image_raw/theora/parameter_descriptions
/turtlebot3_waffle_pi/camera/image_raw/theora/parameter_updates
/turtlebot3_waffle_pi/camera/parameter_descriptions
/turtlebot3_waffle_pi/camera/parameter_updates
```
Camera Sensor is publishing data of ```sensor_msgs/Image``` msg type to ```/turtlebot3_waffle_pi/camera/image_raw``` topic. Let's visualize this data throgh **Rviz**.

Run ```rviz``` in Terminal. Click on Add button, Under tab **By topic** add ```/turtlebot3_waffle_pi/camera/image_raw``` topic. You can see data published on this topic.  
### add image ###
Now, we will subscribe ```/turtlebot3_waffle_pi/camera/image_raw``` topic to convert ROS Image data to OpenCV Image data using **cv_bridge**.

Execute the following command in another tab.
```bash
rosrun aruco_detection detect_marker.py
```
On executing You should be able to see following screen.
### add image ###

Have a look at the detect_marker.py file

```python
#!/usr/bin/env python3

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
import cv2.aruco as aruco
import sys
import math
import time

def detect_ArUco(img):
	## function to detect ArUco markers in the image using ArUco library
	## argument: img is the test image
	## return:   dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners},
	## 	     where ArUco_id_no indicates ArUco id and corners indicates the four corner position 
	##	     of the aruco(numpy array)
	##	     for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
	## 				{0: array([[315, 163], [319, 263], [219, 267], [215,167]], dtype=float32)}
						
    Detected_ArUco_markers = {}
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters) 
    i = 0
    
    try:
        for id in ids:
            for id_Number in id:
                Detected_ArUco_markers[id_Number] = corners[i][0]    

    except TypeError:
        print("No aruco in front of me")

    i += 1
    return Detected_ArUco_markers


def mark_ArUco(img,Detected_ArUco_markers):
	## function to mark ArUco in the test image as per the instructions given in problem statement
	## arguments: img is the test image 
	##			  Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
	## return: image helping sherlock to solve maze 

    ids = Detected_ArUco_markers.keys()
    print(Detected_ArUco_markers)
    centre_aruco = {}
    top_centre = {}

    try:
        for id in ids:
            corners = Detected_ArUco_markers[id]
            for i in range(0, 4):
                cv2.circle(img,(int(corners[i][0]), int(corners[i][1])), 5, (0,0,255), -1)
            centre_aruco[id] = (corners[0]+corners[1]+corners[2]+corners[3])/4
            top_centre[id] = (corners[0]+corners[1])/2
            cv2.line(img, (int(centre_aruco[id][0]), int(centre_aruco[id][1])),
	    		(int(top_centre[id][0]), int(top_centre[id][1])), (255, 0, 0), 5)

    except TypeError:
        print("No aruco in front of me")

    return img

def callback(img):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    Detected_ArUco_markers = detect_ArUco(cv_image)	  
    img = mark_ArUco(cv_image,Detected_ArUco_markers)    
    cv2.namedWindow("Image Window", 1)
    cv2.imshow("Image Window", img)
    k = cv2.waitKey(1)
    

def laser():
    rospy.Subscriber('/turtlebot3_waffle_pi/camera/image_raw', Image, callback)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('detect_marker')
    try:
        laser()

    except rospy.ROSInterruptException:
        pass
	
```
Run ```roslaunch aruco_detection turtlebot3_teleop_key.launch``` in another window, and try to move the bot.

Now, we have seen ArUco detection,

# Let's Solve mAzE

At this stage, you have enough knowledge to escape from the maze created by Moriarty.

Open **maze_aruco.launch** file in launch folder and replace empty.world with maze_aruco.world. Required file is

```xml
<launch>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find aruco_detection)/worlds/maze_aruco.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

</launch>
```
Execute following command 
```bash
roslaunch aruco_detection maze_aruco.launch
roslaunch aruco_detection spawn_turtlebot3.launch
```
Upon execution, the following screen should be visible.
## add image 
Cool !

How will you come out of this maze, which is surrounded by walls from all the sides ?

Well, it's Moriarty's maze.

There is a trick- Bot can go through some of the walls present in the maze. But, how bot will found those walls ? 

ArUco says hi!! 

AruCo will guide you along the way to solve the maze. 

## add image 

*Blue line* in ArUco marker in "Image Window" is indicating that magic wall 

Execute ```rosrun aruco_detection detect_marker.py```. Open new terminal and execute ```roslaunch aruco_detection turtlebot3_teleop_key.launch``` to control bot.
Now, go and Solve the maze. :)

## add meme moriarty's maze solved by sherlock , moriarty defeat

