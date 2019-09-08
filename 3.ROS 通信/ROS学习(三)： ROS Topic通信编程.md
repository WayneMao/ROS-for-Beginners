## ROS学习(三)： ROS Topic通信编程

### 1. ROS项目开发流程

创建工作空间 ----> 创建功能包  ----> 创建源代码(C++/Python)   ----> 配置编译规则(CMakeLists.txt)  ----> 编译运行

#### 1.1 工作空间

**工作空间（workspace）**是一个存放过程开发相关文件的文件夹。简单地说，是我们创建一个工程的根目录。其通常包含了如下文件夹：

- **src:** 代码空间(Source Space)
- **build:** 编译空间(Build Space)，存放编译时生成的中间文件、二进制文件
- **devel:** 开发空间(Development Space)，存放终止编译的可执行文件和环境变量；
- **install:** 安装空间(Install Space)

-----

**创建工作空间：** 

```shell
mkdir -p ~catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace # 初始化workspace，在src文件夹下会产生一个CMakeLists.txt文件
```

**编译工作空间：**

```shell
cd ..
catkin_make #回到catkin_ws文件夹下，进行编译
# 编译之后生成 build,devel
```

**设置环境变量：**

```shell
source devel/setup.bash # source命令通常用于重新执行刚修改的初始化文件，使之立即生效，而不必注销并重新登录。
```

**检查环境变量：**

```shell 
echo $ROS_PACKAGE_PATH # 打印环境变量

/home/mwx/catkin_ws/src:/opt/ros/melodic/share # 前半部分创建功能包的路径，后半部分ROS安装路径
# 注意! 只能在当前路径下生效
```

**编译安装空间：**

```shell
catkin_make install
```

----

#### 1.2 创建一个catkin程序包

一个功能包要想称为catkin功能包必须符合以下要求：

1. 该功能包必须包含catkin 版本的 `package.xml`文件，这个`package.xml`文件提供有关功能包的元信息。
2. 功能包必须包含一个catkin 版本的`CMakeLists.txt`文件，而Catkin metapackages中必须包含一个对`CMakeList.txt`文件的引用。
3. 每个目录下只能有一个功能包。这意味着**在同一个目录下不能有嵌套的或者多个功能包存在**。

最简单的程序包也许看起来就像这样：

```
my_package/
  CMakeLists.txt
  package.xml
```

**创建功能包：**

```shell
catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```

现在使用`catkin_create_pkg`命令来创建一个名为`learning_communication`的新程序包，这个程序包依赖于std_msgs、std_srvs、roscpp和rospy：

```shell
cd ~/catkin_ws/src
catkin_create_pkg learning_communication rospy roscpp std_msgs std_srvs
```
**编译功能包：**

```shell
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash 
```

----

**程序包依赖关系：**

我们可以使用`rospack`命令来查看**一级依赖包**

```shell
rospack depends1 learning_communication   # 最后为功能包名字
```

输出结果如下：

```shell
roscpp
rospy
std_msgs
std_srvs
```

使用` rospack depends1 rospy`来查看**间接依赖**。

---

**package.xml** 这个文件里包含了功能包的名字、版本、描述等，还显示了各种包的依赖关系。

**CMakeLists.txt** 这个文件主要描述了编译规则

**再次提醒注意!!!** 同一个工作空间下,不允许存在同名功能包;不同工作空间下,允许存在同名功能包.

这一节参考：http://wiki.ros.org/cn/ROS/Tutorials/catkin/CreatingPackage

### 2. ROS Topic 通信编程

![ROS3_Topic model.PNG](https://github.com/WayneMao/Markdown-images/blob/master/ROS%E5%AD%A6%E4%B9%A0/ROS3_Topic%20model.PNG?raw=true)

#### 2.1 如何实现一个发布

- 初始化ROS节点
- 向ROS Master注册节点信息，包括发布的话题名和话题中的消息类型；
- 创建消息数据；
- 按照一定频率循环发布消息。

C++代码如下：

```c++
/**
 * 该例程将发布chatter话题，消息类型String
 */
 
#include <sstream>
#include "ros/ros.h" // ROS头文件
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "string_publisher"); // 节点名 不可重名

    // 创建节点句柄：完成ROS核心资源的管理
    ros::NodeHandle n;

    // 创建一个Publisher，发布名为chatter的topic，消息类型为std_msgs::String， 缓冲区大小：1000
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    // 设置循环的频率 10HZ ~ 100ms
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        // 初始化std_msgs::String类型的消息
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        // 发布消息
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg); //数据进入缓冲区

        // 按照循环频率延时
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
```

Python：

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将发布chatter话题，消息类型String

import rospy
from std_msgs.msg import String

def velocity_publisher():
	# ROS节点初始化
    rospy.init_node('talker', anonymous=True) 
    '''
    The anonymous=True flag tells rospy to generate a unique name
    for the node so that you can have multiple listener.py nodes run easily.
    '''
    
	# 创建一个Publisher，发布名为/chatter的topic，消息类型为String，队列长度10
    pub = rospy.Publisher('chatter', String, queue_size=10)

	#设置循环的频率
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
		# 初始化PersonMsg类型的消息
        hello_str = "hello world %s" % rospy.get_time()

		# 发布消息
        pub.publish(hello_str)
        rospy.loginfo(hello_str) # 日志

		# 按照循环频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass
```




#### 2.2 如何实现一个订阅

- 初始化ROS节点；
- 订阅需要的话题；
- 循环等待话题消息，接受到消息后进入回调函数；
- 在回调函数中完成消息处理。

C++代码如下：

```c++
/**
 * 该例程将订阅chatter话题，消息类型String
 */
 
#include "ros/ros.h"
#include "std_msgs/String.h"

// 接收到订阅的消息后，会进入消息回调函数
void chatterCallback(const std_msgs::String::ConstPtr& msg) // count 是个常指针
{
    // 将接收到的消息打印出来
    ROS_INFO("I heard: [%s]", msg->data.c_str()); // ROS_INFO 类似于printf()
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "string_subscriber");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Subscriber，订阅名为chatter的topic，注册回调函数chatterCallback；缓冲区大小1000
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
	// 发现有数据进来后，调用 回调函数 

    // 循环等待回调函数; 这个是死循环
    ros::spin();

    return 0;
}
```



#### 2.3 如何配置CMakeLists.txt中的编译规则

- 设置需要编译的代码和生成的可执行文件；
- 设置链接库

在catkin_ws/src/learning_communication/CMakeLists.txt文件中 ## Install ## 之前添加如下：

> add_executable(string_publisher src/string_publisher.cpp)  # 代码名和可执行文件名是一样的
> target_link_libraries(string_publisher ${catkin_LIBRARIES})  # 跟ROS库 链接
>
> add_executable(string_subscriber src/string_subscriber.cpp)
> target_link_libraries(string_subscriber ${catkin_LIBRARIES})

```shell
cd ~/catkin_ws
catkin_make  # 结果在catkin_ws/devel/lib/learning_communication 下产生两个可执行文件string_publisher,string_subscriber

roscore #在新的Terminal中运行 roscore

source devel/setup.bash # 更新环境，这个脚本告诉系统功能包在哪个位置,
rosrun learning_communication string_publisher

# 打开新的Terminal
source devel/setup.bash
rosrun learning_communication string_subscriber
```

如果不想要每次运行`source devel/setup.bash`,我们可以把这句话添加到`bashrc`文件里头。具体操作如下：

Home文件夹下`Ctrl+h`显示隐藏文件，找到配置文件`.bashrc`,在最后添加

`source ~/catkin_ws/devel/setup.bash`, 这样就可以直接打开一个新的Terminal，直接运行`rosrun learning_communication string_subscriber`了。

**小结**

至此我们已经完整地实现了一遍ROS开发流程：

创建工作空间 ----> 创建功能包  ----> 创建源代码(C++/Python)   ----> 配置编译规则(CMakeLists.txt)  ----> catkin_make编译工作空间,rosrun运行节点

**PS:** python比C++方便简单不少，不需要配置CMakeLists.txt文件，直接运行如下命令就可以了

```shell
roscore
rosrun learning_communication string_publisher.py
rosrun learning_communication string_subscriber.py
```



注意：发现Couldn't find executable named person_subscriber.py.......这类的错误，可以修改文件权限

```shell
chmod +x talker.py             #设置可执行
```

参考：[编写简单的消息发布器和订阅器 (C++)](http://wiki.ros.org/cn/ROS/Tutorials/WritingPublisherSubscriber(c%2B%2B))

python参考：[ROS与Python入门教程-写简单发布器和订阅器](https://www.ncnynl.com/archives/201611/1059.html)

---

---

#### 2.4 自定义话题消息

##### 2.4.1 如何自定义话题消息

- 定义**msg**文件；

在ROS中允许msg文件自定义话题消息。在**catkin_ws/src/learning_communication**路径下创建**msg**文件夹，以放置所有自定义消息文件。在msg文件夹里面使用命令`touch PersonMsg.msg`新建一个文件,在该文件中放入

> string name
> uint8  age
> uint8  sex
>
> uint8 unknown = 0
> uint8 male    = 1
> uint8 female  = 2

- 在**package.xml**中添加功能包依赖

在catkin_ws/src/learning_communication/package.xml 文件内底部相应位置添加  //C++/python 都要

> <build_depend>message_generation</build_depend>  <exec_depend>message_runtime</exec_depend>

一个编译依赖，一个运行依赖；

- 在**CMakeLists.txt**中添加编译选项

在catkin_ws/src/learning_communication/CMakeLists.txt 文件中找到 find_package()函数,这个函数帮助我们查找功能包, 在函数里添加`message_generation`依赖包：

> find_package(...message_generation)

在动态产生头文件位置(## Declare ROS messages, services and actions ##)添加如下: 

> add_message_files(FILES PersonMsg.msg)  # 把PersonMsg.msg编译成对应不同语言的相应头文件
> generate_messages(DEPENDENCIES std_msgs) # 依赖std_msgs库



在catkin_package()函数里添加动态依赖 CATKIN_DEPENDS roscpp rospy std_msgs std_srvs message_runtime：

> catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs std_srvs message_runtime)



- 编译生成语言相关文件

`catkin_make` 编译，编译生成位于 ~/catkin_ws/devel/include/learning_communication/PersonMsg.h的头文件

##### 2.4.2 如何实现一个发布者
- 初始化ROS节点
- 向ROS Master注册节点信息，包括发布的话题名和话题中的消息类型；
- 创建消息数据；
- 按照一定频率循环发布消息。

C++代码如下：

```c++
/**
 * 该例程将发布/person_info话题，learning_communication::PersonMsg
 */
 
#include <ros/ros.h>
#include "learning_communication/PersonMsg.h"

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "person_publisher");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_communication::PersonMsg，队列长度10
    ros::Publisher person_info_pub = n.advertise<learning_communication::PersonMsg>("/person_info", 10);
// person_info_pub 话题名； 
// NodeHandle::advertise() 返回一个 ros::Publisher 对象,它有两个作用： 
//1) 它有一个 publish() 成员函数可以让你在topic上发布消息； 2) 如果消息类型不对,它会拒绝发布。
//  消息类型 learning_communication::PersonMsg 类似 std_msgs/String 消息类型

    // 设置循环的频率
    ros::Rate loop_rate(1);

    int count = 0;
    while (ros::ok()) //Ctrl-C 退出
    {
        // 初始化learning_communication::Person类型的消息
    	learning_communication::PersonMsg person_msg;
		person_msg.name = "Tom";
		person_msg.age  = 18;
		person_msg.sex  = learning_communication::PersonMsg::male;

        // 发布消息
		person_info_pub.publish(person_msg);

       	ROS_INFO("Publish Person Info: name:%s  age:%d  sex:%d", 
				  person_msg.name.c_str(), person_msg.age, person_msg.sex);

        // 按照循环频率延时
        loop_rate.sleep();
    }

    return 0;
}
```



Python：

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将发布/person_info话题，自定义消息类型learning_communication::PersonMsg

import rospy
from learning_communication.msg import PersonMsg

def velocity_publisher():
	# ROS节点初始化
    rospy.init_node('person_publisher', anonymous=True)

	# 创建一个Publisher，发布名为/person_info的topic，消息类型为PersonMsg，队列长度10
    person_info_pub = rospy.Publisher('/person_info', PersonMsg, queue_size=10)

	#设置循环的频率
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
		# 初始化PersonMsg类型的消息
    	person_msg = PersonMsg()
    	person_msg.name = "Tom";
    	person_msg.age  = 18;
    	person_msg.sex  = PersonMsg.male;

		# 发布消息
        person_info_pub.publish(person_msg)
    	rospy.loginfo("Publsh person message[%s, %d, %d]", 
				person_msg.name, person_msg.age, person_msg.sex)

		# 按照循环频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass
```



##### 2.4.3 如何实现一个订阅者

- 初始化ROS节点；
- 订阅需要的话题；
- 循环等待话题消息，接受到消息后进入回调函数；
- 在回调函数中完成消息处理。

C++代码如下：

```c++
/**
 * 该例程将订阅/person_info话题，自定义消息类型learning_communication::PersonMsg
 */
 
#include <ros/ros.h>
#include "learning_communication/PersonMsg.h" // ~catkin_ws/devel/include/learning_communication/PersonMsg.h

// 接收到订阅的消息后，会进入消息回调函数
void personInfoCallback(const learning_communication::PersonMsg::ConstPtr& msg) // 指针
{
    // 将接收到的消息打印出来
    ROS_INFO("Subcribe Person Info: name:%s  age:%d  sex:%d", 
			 msg->name.c_str(), msg->age, msg->sex); // 指针调用
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "person_subscriber");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    ros::Subscriber person_info_sub = n.subscribe("/person_info", 10, personInfoCallback);

    // 循环等待回调函数
    ros::spin();

    return 0;
}
```



Python:

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将订阅/person_info话题，自定义消息类型PersonMsg

import rospy
#from learning_topic.msg import PersonMsg
from learning_communication.msg import PersonMsg

def personInfoCallback(msg):
    rospy.loginfo("Subcribe Person Info: name:%s  age:%d  sex:%d", 
			 msg.name, msg.age, msg.sex)

def person_subscriber():
	# ROS节点初始化
    rospy.init_node('person_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    rospy.Subscriber("/person_info", PersonMsg, personInfoCallback)

	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    person_subscriber()
```





##### 2.4.4 配置CMakeLists.txt编译规则

- 设置需要编译的代码和生成可执行文件
- 设置链接库
- 添加依赖项       gencpp:generation cpp

> add_executable(person_publisher src/person_publisher.cpp)
> target_link_libraries(person_publisher ${catkin_LIBRARIES})
> add_dependencies(person_publisher ${PROJECT_NAME}_gencpp)  
>
> add_executable(person_subscriber src/person_subscriber.cpp)
> target_link_libraries(person_subscriber ${catkin_LIBRARIES})
> add_dependencies(person_subscriber ${PROJECT_NAME}_gencpp)



**编译：**

```shell
cd ~/catkin_ws
catkin_make  # 结果在catkin_ws/devel/lib/learning_communication 下产生两个可执行文件person_publisher ,person_subscriber 

roscore #在新的Terminal中运行 roscore

#source devel/setup.bash # 因为我们前面把这句话写进了bashrc，所以这步可以省略
rosrun learning_communication person_publisher

# 打开新的Terminal
source devel/setup.bash
rosrun learning_communication person_subscriber
```



python:

```shell
roscore

rosrun learning_communication person_publisher.py

rosrun learning_communication person_subscriber.py
```



结果如下图所示：

![ROS3_r1.png](https://github.com/WayneMao/Markdown-images/blob/master/ROS%E5%AD%A6%E4%B9%A0/ROS3_r1.png?raw=true)

![ROS3_r2.PNG](https://github.com/WayneMao/Markdown-images/blob/master/ROS%E5%AD%A6%E4%B9%A0/ROS3_r2.PNG?raw=true)



