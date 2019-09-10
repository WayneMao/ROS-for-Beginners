## ROS学习(五)：服务器客户端综合应用


### 1. 创建一个节点，在其中实现一个订阅者和一个发布者，完成以下功能：

- 发布者：发布海龟速度指令，让海龟圆周运动
- 订阅者：订阅海龟的位置信息，并在终端中周期打印输出

记得在CMakeLists.txt里头添加 turtlesim 包

```python
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  message_generation
  turtlesim
)
```

刚开始学习ROS 的时候，经常搞不清各种Topic，msg等，以turtlesim为例，

我们使用`rostopic list`查看有多少个topic存在:

> /rosout
> /rosout_agg
> /turtle1/cmd_vel
> /turtle1/color_sensor
> /turtle1/pose



获知某个topic的消息类型：

```shell
rostopic type /turtle1/pose
# 或者下面的命令，这两者效果是一样的，
rostoipc info turtle1/pose
```

> turtlesim/Pose



使用`rosmsg show [message type]`来检查ROS是否能够识别msg, (显示所含的信息参数)

```shell
rosmsg show turtlesim/Pose
```

> float32 x
> float32 y
> float32 theta
> float32 linear_velocity
> float32 angular_velocity

- turtlesim--消息所在的package
- Pose--消息名Pose



同理:

```shell
rostopic type /turtle1/cmd_vel  # /turtle1/cmd_vel 海龟的线性和弧度命令，即控制位移和方向命令
```
> geometry_msgs/Twist
```shell
rosmsg show geometry_msgs/Twist
```

> geometry_msgs/Vector3 linear
> float64 x
> float64 y
> float64 z
> geometry_msgs/Vector3 angular
> float64 x
> float64 y
> float64 z



完整代码：

```python
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
```

**注意：** `rospy.spin()` 函数python和C++的不同之处

> The final addition, `rospy.spin()` simply keeps your node from exiting until the node has been shutdown. Unlike roscpp, rospy.spin() does not affect the subscriber callback functions, as those have their own threads.



### 2 创建另外一个节点，在其中实现一个客户端，完成以下功能：

- 客户端：请求海龟诞生的服务，在仿真器中产生一直新的海龟



[32newturtles.py](learning_communication/scripts/32newturtles.py)

先清理一下turtlesim_node背景上的轨迹,

```shell
rosservice call clear
```



先查看有哪些service存在:

`rosservice list`

> /clear
> /kill
> /reset
> /rosout/get_loggers
> /rosout/set_logger_level
> /spawn
> /teleop_turtle/get_loggers
> /teleop_turtle/set_logger_level
> /turtle1/set_pen
> /turtle1/teleport_absolute
> /turtle1/teleport_relative
> /turtlesim/get_loggers
> /turtlesim/set_logger_level

查看`/spawn`类型：

`rosservice type /spawn`

> turtlesim/Spawn

查看服务参数(srv文件)：

`rossrv show turtlesim/Spawn `

> float32 x
> float32 y
> float32 theta
>
> string name
>
> \---
>
> string name

**srv**文件分为请求和响应两部分，由---分隔。**request/response**



```python
#!/usr/bin/env python

import rospy

import turtlesim.msg
 
from turtlesim.srv import Spawn,SpawnRequest
#from geometry_msgs.msg import Twist


def newturtle_client():
    # init node
    rospy.init_node('newturtle_client')

    # spawnClient = rospy.Service

    
    rospy.wait_for_service('/spawn')
    try:
        # build a client, 
        spawnClient = rospy.ServiceProxy('/spawn',Spawn)
        spawnClient(4.0, 3.0, 2.0, 'alpha')
        #return Spawn.name
    except rospy.ServiceException,e:
        print "Service call faild: %s" %e


if __name__ == "__main__":
    newturtle_client()
```

查看服务`rosservice list`,结果如下：

> /alpha/set_pen
> /alpha/teleport_absolute
> /alpha/teleport_relative
> /clear
> /kill
> /reset
> /rosout/get_loggers
> /rosout/set_logger_level
> /spawn
> /turtle1/set_pen
> /turtle1/teleport_absolute
> /turtle1/teleport_relative
> /turtlesim/get_loggers
> /turtlesim/set_logger_level



### 3 综合运用话题与服务编程、命令行使用，实现以下场景

实现一个海龟运动控制的功能包，需要具备以下功能（以下指令的接收方均为该功能包中的节点）：

- 通过命令行发送新生海龟的名字，即可在界面中产生一只海龟，并且位置不重叠；
- 通过命令行发送指令控制界面中任意海龟圆周运动的启动/停止，速度可通过命令行控制；



综合以上两题：

```python
#!/usr/bin/env python

import rospy
import turtlesim.msg
 
from turtlesim.srv import Spawn,SpawnRequest
from geometry_msgs.msg import Twist

import sys
import random

def newturtle_client():
    # init node
    rospy.init_node('newturtle_client')
    # wait service
    rospy.wait_for_service('/spawn')
    
    # random reduce float
    x = random.uniform(0,9)
    y = random.uniform(0,9)
    z = random.uniform(0,9)
    rospy.loginfo('This turtle\'s name is %s.', name)
    try:
        # build a client, turtlrCallback
        spawnClient = rospy.ServiceProxy('/spawn',Spawn)
        spawnClient(x, y, z, name)
    except rospy.ServiceException,e:
        print "Service call faild: %s" %e

def control_publisher():
    # init node
    #rospy.init_node('turtle_cmd', anonymous = True) # 不要再初始化节点
    
    # get input 
    name1,linearx, angularz = raw_input("Please input turtle\'s name, linear_velocity and angular: ").split( )
    linearx = float(linearx)
    angularz = float(angularz)
    print 'linear velocity: %f; angular: %f' %(linearx, angularz)

    # build a publisher
    pub = rospy.Publisher('/%s/cmd_vel' %name1, Twist, queue_size = 10)

    # delay
    rate = rospy.Rate(10)

    # loop
    while not rospy.is_shutdown():
        # init Twist
        twist = Twist()
        twist.linear.x = linearx 
        twist.angular.z = angularz 

        # publish message
        pub.publish(twist)

        rate.sleep()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print 'Please input turtle name.'
    else:
        name = sys.argv[1]
        newturtle_client()
        control_publisher()
```



先启动小海龟仿真：[33.py](learning_communication/scripts/33.py)

```shell
roscore
rosrun turtlesim turtlesim_node
```

接着：

```shell
rosrun learning_communication 33.py alpha # 最后一个参数是新产生海龟的名字
> This turtle's name is alpha.
> Please input turtle's name, linear_velocity and angular: alpha 2.0 3.0 # 指定对应名字海龟的速度，方向
> linear velocity: 2.000000; angular: 3.000000

```

结果： 产生一只新的海龟，并指定某只海龟的速度、方向。



目前存在一个问题：
需要多线程解决控制多只乌龟

