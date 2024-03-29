#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将请求/show_person服务，服务数据类型learning_communication.srv

import sys
import rospy
from learning_communication.srv import PersonSrv, PersonSrvRequest

def person_client():
	# ROS节点初始化
    rospy.init_node('person_client')

	# 发现/show_person服务后，创建一个服务客户端，连接名为/show_person的service
    rospy.wait_for_service('/show_person')
    try:
        person_client = rospy.ServiceProxy('/show_person', PersonSrv)# 消息类型PersonSrv

		# 请求服务调用，输入请求数据
        response = person_client("Tom", 20, PersonSrvRequest.male)
        return response.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
	#服务调用并显示调用结果
    print "Show person result : %s" %(person_client())


''' srv request/response
string name
uint8  age
uint8  sex

uint8 unknown = 0
uint8 male    = 1
uint8 female  = 2

---
string result
```