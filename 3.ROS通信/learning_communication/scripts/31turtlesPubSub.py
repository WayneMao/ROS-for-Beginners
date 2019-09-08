#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将订阅/person_info话题，消息类型String

import rospy
from std_msgs.msg import String
import turtlesim.msg

from turtlesim.msg import Pose  # turtlesim.msg.Pose

from geometry_msgs.msg import Twist
# import turtlesim/pose

    
def poseCallback(msg):
    rospy.loginfo("Turtle Pose: %f, %f, angular %f", msg.x, msg.y, msg.theta)
    #rospy.loginfo("Successful")

def person_subscriber():
    # ROS节点初始化
    rospy.init_node('turtle_cmd', anonymous=True)
    
    # bulid a publisher
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    # 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数stringCallback
    # 消息类型 turtlesim.msg.Pose 
    rospy.Subscriber("/turtle1/pose", Pose, poseCallback)

    # delay
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # init Twist message
        twist = Twist()
        twist.linear.x = 2.0
        twist.angular.z = 1.0
        
        # publish message
        pub.publish(twist)
        
        # 循环等待回调函数
        #rospy.spin()

        rate.sleep() 


if __name__ == '__main__':
    person_subscriber()
