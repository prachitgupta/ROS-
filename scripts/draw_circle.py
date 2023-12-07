#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist


if __name__ == '__main__':
    rospy.init_node("my_circle")
    rospy.loginfo("u can do this")
    #define publisher
    pub = rospy.Publisher("/turtle1/cmd_vel",Twist,queue_size=10)
    
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        #define msg
        msg = Twist()
        msg.linear.x = 2
        msg.angular.z = 1

        #post
        pub.publish(msg)

        rate.sleep()

  

