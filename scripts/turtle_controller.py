#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

def call_set_pen(r,g,b,width,off):
    try:
        set_pen = rospy.ServiceProxy("/turtle1/set_pen",SetPen)
        response = set_pen(r,g,b,width,off)
    except rospy.ServiceException as e:
        rospy.logwarn(e)

def make_semicircle(pose): 
    cmd = Twist()

    if pose.x < 5.544444561004639 and pose.y > 5.544444561004639:
        cmd.linear.x = 0
        cmd.linear.y = 2
        cmd.angular.z = 0
        call_set_pen(255,255,0,3,0)
    else:
        cmd.linear.x = 2
        cmd.angular.z = 1
        call_set_pen(255,0,0,3,0)
    if pose.y < 5.544444561004639:
        cmd.linear.x = 0
        cmd.angular.z = 0
        call_set_pen(255,255,0,3,0)
    pub.publish(cmd)


if __name__ == '__main__':
    rospy.init_node("semicircle")
    rospy.wait_for_service("/turtle1/set_pen")
    rospy.loginfo("This is the moment")
    #define pub and subscriber
    pub = rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=10)
    sub = rospy.Subscriber("/turtle1/pose",Pose,callback =make_semicircle)

    rospy.spin()