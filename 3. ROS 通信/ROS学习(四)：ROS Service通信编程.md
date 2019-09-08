## ROS学习(四)：ROS Service通信编程

![ROS3_service model.PNG](https://github.com/WayneMao/Markdown-images/blob/master/ROS%E5%AD%A6%E4%B9%A0/ROS3_service%20model.PNG?raw=true)

### 1. ROS Service通信编程

#### 1.1 如何实现一个服务器

- 初始化ROS节点；
- 创建Service实例；
- 循环等待服务请求，进入回调函数；
- 在回调函数中完成服务功能的处理，并反馈应答数据。

C++代码如下：

```c++
/**
 * 该例程将提供print_string服务，std_srvs::SetBool
 */
 
#include "ros/ros.h"
#include "std_srvs/SetBool.h"  //标准服务接口定义 可通过`rossrv show std_srvs/SetBool`命令查看

// service回调函数，输入参数req，输出参数res    ；request, response
bool print(std_srvs::SetBool::Request  &req, 
         std_srvs::SetBool::Response &res)
{
    // 打印字符串
    if(req.data)
	{
		ROS_INFO("Hello ROS!");
		res.success = true;
		res.message = "Print Successully";
	}
	else
	{
		res.success = false;
		res.message = "Print Failed";
	}

    return true;
}

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "string_server");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个名为print_string的server，注册回调函数print()
    ros::ServiceServer service = n.advertiseService("print_string", print);

    // 循环等待回调函数
    ROS_INFO("Ready to print hello string.");
    ros::spin();

    return 0;
}
```

Python代码如下：

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将提供print_string服务，std_srvs::SetBool

import rospy
from std_srvs.srv import SetBool, SetBoolResponse

def stringCallback(req):
	# 显示请求数据
    if req.data:
        rospy.loginfo("Hello ROS!")

		# 反馈数据
        return SetBoolResponse(True, "Print Successully")
    else:
        # 反馈数据
        return SetBoolResponse(False, "Print Failed")

def string_server():
	# ROS节点初始化
    rospy.init_node('string_server')

	# 创建一个名为/print_string的server，注册回调函数stringCallback
    s = rospy.Service('print_string', SetBool, stringCallback)

	# 循环等待回调函数
    print "Ready to print hello string."
    rospy.spin()

if __name__ == "__main__":
    string_server()
```



#### 1.2 如何实现一个客户端

- 初始化ROS节点；
- 创建Client实例；
- 发布服务请求数据；
- 等待Server处理之后的应答结果。

C++代码如下：

```c++
/**
 * 该例程将请求print_string服务，std_srvs::SetBool
 */
 
#include "ros/ros.h"
#include "std_srvs/SetBool.h"

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "string_client");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个client，service消息类型是std_srvs::SetBool
    ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("print_string");

    // 创建std_srvs::SetBool类型的service消息
    std_srvs::SetBool srv;
    srv.request.data = true;

    // 发布service请求，等待应答结果
    if (client.call(srv))
    {
        ROS_INFO("Response : [%s] %s", srv.response.success?"True":"False", 
									   srv.response.message.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service print_string");
        return 1;
    }

    return 0;
}
```

**编译运行**



Python代码：

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将请求print_string服务，std_srvs::SetBool

import sys
import rospy
from std_srvs.srv import SetBool, SetBoolRequest

def string_client():
	# ROS节点初始化
    rospy.init_node('string_client')

	# 发现print_string服务后，创建一个服务客户端，连接名为print_string的service
    rospy.wait_for_service('print_string')
    try:
        string_client = rospy.ServiceProxy('print_string', SetBool)

		# 请求服务调用，输入请求数据
        response = string_client(True)
        return response.success, response.message
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
	#服务调用并显示调用结果
    print "Response : [%s] %s" %(string_client())
```



### 2. 自定义ROS Service通信

#### 2.1 自定义ROS Service通信

- 定义**srv**文件；

在ROS中允许srv文件自定义话题消息。在**catkin_ws/src/learning_communication**路径下创建**srv**文件夹，以放置所有自定义消息文件。在msg文件夹里面使用命令`touch PersonMsg.srv`新建一个文件,在该文件中放入

> string name
> uint8  age
> uint8  sex
>
> uint8 unknown = 0
> uint8 male    = 1
> uint8 female  = 2
>
> \---
>
> string result

其中\---一下的内容是response

- 在**package.xml**中添加功能包依赖

在catkin_ws/src/learning_communication/package.xml 文件内底部相应位置添加

> <build_depend>message_generation</build_depend>  <exec_depend>message_runtime</exec_depend>

一个编译依赖，一个运行依赖；

- 在**CMakeLists.txt**中添加编译选项

在catkin_ws/src/learning_communication/CMakeLists.txt 文件中找到 find_package()函数,这个函数帮助我们查找功能包, 在函数里添加`message_generation`依赖包：

> find_package(...message_generation)

在动态产生头文件位置(## Declare ROS messages, services and actions ##下面)添加如下: 

> add_service_files(FILES PersonSrv.srv)
> generate_messages(DEPENDENCIES std_msgs) # 这句存在的话可以不添加



在catkin_package()函数里添加动态依赖 CATKIN_DEPENDS roscpp rospy std_msgs std_srvs message_runtime：

> catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs std_srvs message_runtime)



- 编译生成语言相关文件

`catkin_make` 编译，编译生成位于 ~/catkin_ws/devel/include/learning_communication/PersonSrvRequest.h和PersonSrvResponse.h的头文件



person_server.py:

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将执行/show_person服务，服务数据类型learning_communication::PersonSrv

import rospy
from learning_communication.srv import PersonSrv, PersonSrvResponse

def personCallback(req):
	# 显示请求数据
    rospy.loginfo("Person: name:%s  age:%d  sex:%d", req.name, req.age, req.sex)

	# 反馈数据
    return PersonResponse("OK")

def person_server():
	# ROS节点初始化
    rospy.init_node('person_server')

	# 创建一个名为/show_person的server，注册回调函数personCallback
    s = rospy.Service('/show_person', PersonSrv, personCallback)

	# 循环等待回调函数
    print "Ready to show person informtion."
    rospy.spin()

if __name__ == "__main__":
    person_server()
```

person_client.py:

```python
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
```



**注意：** 再次强调要注意给文件权限，`chmod +x talker.py `            #设置可执行








