#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将请求/show_person服务，服务数据类型learning_service::Person

import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')
    
    # 创建tf 监听器
    listener = tf.TransformListener()

    # 请求产生 turtle2
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')
    
    # 创建turtle2 速度控制指令 发布者
    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))#查询坐标之间关系
            # rospy.Time(0)表示当前时间
            # 返回两个参数 (x, y, z) linear transformation; (x, y, z, w) quaternion ,rotate
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        #  根据turtle1，turtle2坐标之间的位置关系，发布turtle2 的速度控制指令
        angular = 4 * math.atan2(trans[1], trans[0]) # 角速度 , math.atan2() 反正切值
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)  # 线速度 |X^2 + Y^2|
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)

        rate.sleep()


