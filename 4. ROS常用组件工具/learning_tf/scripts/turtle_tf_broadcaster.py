#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将请求/show_person服务，服务数据类型learning_service::Person

import roslib
roslib.load_manifest('learning_tf')
import rospy

import tf
import turtlesim.msg

def handle_turtle_pose(msg, turtlename):
    # 创建tf广播器
    br = tf.TransformBroadcaster()
    
    # 广播world与海龟坐标系之间的tf数据
    br.sendTransform((msg.x, msg.y, 0),  # 平移变换， Z 轴为0
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta), # 平移旋转，欧拉角(弧度)转换四元数，或者直接使用四元数
                     rospy.Time.now(), # 时间属性
                     turtlename,  # 两个坐标系的名字 
                     "world") 

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    turtlename = rospy.get_param('~turtle')
    # 订阅海龟的位姿话题
    rospy.Subscriber('/%s/pose' % turtlename,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)
    # 等待回调函数
    rospy.spin() 


