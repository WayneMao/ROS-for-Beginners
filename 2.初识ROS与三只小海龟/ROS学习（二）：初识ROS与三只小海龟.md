### ROS学习（二）：初识ROS与三只小海龟

#### 1. 什么是ROS

ROS (Robot Operating System, 机器人操作系统) 提供一系列程序库和工具以帮助软件开发者创建机器人应用软件。它提供了硬件抽象、设备驱动、函数库、可视化工具、消息传递和软件包管理等诸多功能。ROS遵循BSD开源许可协议。具体的参见[What is ROS](http://wiki.ros.org/ROS/Introduction)

#### 2. ROS核心概念与通信机制

##### 2.1 节点(Node)---执行单元

- 执行具体任务的**进程**、独立运行的可执行文件；
- 不同节点可以使用不同编程语言，可分布式运行在不同的主机；
- 节点名称在系统中**唯一**；

**简单的说：**节点就像公司里干活的人或者部门。

##### 2.2 节点管理器(ROS Master)---控制中心

- 为节点提供**命名、注册**服务；
- 跟踪和记录话题/服务通信，辅助节点相互查找、建立连接；
- 提供**参数服务器**，节点使用此服务器存储和检索运行时的参数。

**简单的说：**就是管理节点的东西，就像是公司里管理干活的人的人或者干活的部门的上级部门


##### 2.3 话题(Topic)---异步通信机制

- 节点间用来传输数据的重要**总线**;
- 使用**发布/订阅**模型，数据由发布者传输到订阅者，同一个话题的订阅者或发布者**可以不唯一**。

**小结：**就跟我们以前学的SPI、IIC、CAN总线类似，其本质上并没有很大的区别，都是总线。通常来说总线（Bus）是计算机各种功能部件之间传送信息的公共通信干线；话题就是各个节点之间传递信息的公共通信干线。

##### 2.4 消息(Message) --- 话题数据

- 具有一定的类型和数据结构，包括ROS提供的标准类型和用户自定义类型；
- 使用编程语言无关的**.msg**文件定义，编译过程中生成对应的代码文件。

**小结:**过多的解释反而让概念变得复杂，message这个词本身就能让我们明白它是干什么的。

##### 2.5 服务(Service) --- 同步通信机制

- 使用**客户端/服务器**(C/S)，客户端发送**请求**数据，服务器完成处理后返回**应答**数据
- 使用编程语言无关的**.srv**文件定义请求和应答数据结构，编译过程中生成对应的代码文件

**小结：** 服务也是ROS里通信的一种，要明白ROS里头Topic和Service的区别，主要还是要明白异步通信和同步通信的区别。

同步通信实时性比较高，有应答，收到请求后需要尽快处理并回复，如SPI、IIC就是同步通信；

异步实时性比较弱，没有应答，收到请求后，什么时候去做不一定，如UART就是异步通信。

下表是话题与服务的一些异同点。

|              |     话题      |        服务        |
| :----------: | :-----------: | :----------------: |
|  __同步性__  |     异步      |        同步        |
| __通信模型__ |   发布/订阅   |   服务器/客户端    |
| **底层协议** | ROSTCP/ROSUDP |   ROSTCP/ROSUDP    |
| **反馈机制** |      无       |         有         |
|  **缓冲区**  |      有       |         无         |
|  **实时性**  |      弱       |         强         |
| **节点关系** |    多对多     | 一对多(一个服务器) |
| **适用场景** |   数据传输    |      逻辑处理      |

##### 2.5 参数(Paramater) --- 全局共享字典

- 可通过**网络**访问的共享、多变量**字典**；
- 节点使用此服务器来存储和检索**运行时的参数**;
- 适合存储静态、非二进制的**配置参数**，不适合存储动态配置的数据。
- - `rosparam`使得我们能够存储并操作ROS [参数服务器（Parameter Server）](http://wiki.ros.org/参数服务器（Parameter Server）)上的数据。参数服务器能够存储整型、浮点、布尔、字符串、字典和列表等数据类型。
  - rosparam使用YAML标记语言的语法。

**小结：**思考python中的字典类型，

参考：[Understanding ROS Services and Parameters](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams)

###### 2.6.1 功能包(Package)

- ROS软件中的基本单元，包含节点源码、配置文件、数据定义等

###### 2.6.2 功能包 清单(Package manifest)

- 记录功能包的基本信息，包含作者信息、许可信息、依赖选项、编译标志等

###### 2.6.3 元功能包(Meta Packages)

- 组织多个用于同一目的功能包

**小结：** 按照层级来分 软件仓库 >  元功能包 > 功能包

参考：[手动创建ROS package]([http://wiki.ros.org/cn/ROS/Tutorials/Creating%20a%20Package%20by%20Hand](http://wiki.ros.org/cn/ROS/Tutorials/Creating a Package by Hand))



#### 3. 三只小海龟

##### 3.1 小海龟仿真

在第一篇文章：[ROS学习（一）：安装ROS Melodic](https://zhuanlan.zhihu.com/p/77987884)中介绍了启动小海龟的过程，在此再次简单复述一下：

 **启动ROS Master** `roscore`

**启动小海龟仿真器** `rosrun turtlesim turtlesim_node`

**启动海龟控制节点** `rosrun turtlesim turtle_teleop_key ` 控制小海龟运动，*teleop* 是teleoperation（遥操作）的缩写，是指人通.过远程发送运动指令控制机器人。

##### 3.2 可视化工具

使用`rqt_graph`可视化工具查看系统中运行的计算图，在新的Terminal中输入命令`rqt_graph`便可以见到如下图所示结果：

![ROS2_rqt_graph1.PNG](https://github.com/WayneMao/Markdown-images/blob/master/ROS%E5%AD%A6%E4%B9%A0/ROS2_rqt_graph1.PNG?raw=true)

![ROS2_rqt_graph2.PNG](https://github.com/WayneMao/Markdown-images/blob/master/ROS%E5%AD%A6%E4%B9%A0/ROS2_rqt_graph2.PNG?raw=true)



##### 3.3 常用命令

- **查看话题列表** `rosnode list`,可以查看现在所运行的节点,此时结果如下所示：

  > /rosout
  > /rosout_agg
  > /statistics
  > /turtle1/cmd_vel
  > /turtle1/color_sensor
  > /turtle1/pose

- `rostopic acho` **显示某个话题上的发布的数据**，`rostopic echo /turtle1/pose` 此处查看小海龟的姿态数据，`echo`跟shell脚本里头`echo`的使用方法类似。

  > x :    # x,y 代表位置信息，原点为左下角， 单位为 米
  >
  > y：
  >
  > theta： 角度信息，单位是： 弧度
  >
  > linear_velocity: 线速度
  >
  > angular_velocity: 角速度 

- 运行`rostopic pub /turtle1/cmd_vel` 之后，应该会看到 `geometry_msgs/Twist`; 一般来说，`cmd_vel`命令都是代表控制机器人的速度,`Twist`包含线速度和角速度。

- **发布话题消息** `rostopic pub /turtle1/cmd_vel geometry_msgs/Twist ` Tab键后，能看到线速度和角速度信息，修改相应参数就能改变相应的信息， `rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist ` 其中`-r 10` 是以每秒钟10次的频率来发布命令。

  当然可以直接用命令`rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'`

  `--`（双破折号）这会告诉命令选项解析器接下来的参数部分**都不是**命令选项。这在参数里面包含有破折号`-`（比如负号）时是必须要添加的。

- **发布服务请求** `rosservice call /spawn`  此处`spawn`命令再请求一只海龟

  > rosservice call /spawn "x:2.0
  >
  > y:2.0
  >
  > theta: 0.0
  >
  > name: ‘turtles2' "  #给新产生的乌龟一个新的名字

  当然我们也可以直接使用命令`rosservice call /spawn 2 2 0.2 ""`，ROS会自动命名新的海龟--

  `name: "turtle2"` 

  

- GUI插件，2D 曲线 `rqt_plot`，r 是 ros ，qt是一个开发工具，详见[Qt](https://zh.wikipedia.org/wiki/Qt)。

  `rqt_plot`的 主要作用是在ROS中查看变量时间趋势

##### 3.4 三只小海龟

按照前面所讲启动ROS，并发布第一只小海龟

**发布第二只海龟** `rosservice call /spawn 2 2 0.2 ""` 系统自动给了名字：turtle2

**第二种命令发布第三只海龟：** `rosservice call /spawn` Tab键出来如下信息：修改x,y和名字：turtle4

> rosservice call /spawn "x:2.0
>
> y:2.0
>
> theta: 0.0
>
> name: ‘turtles4' "  

到此我们启动了三只小海龟，结果如下图所示：

![ROS2_three turtles.PNG](https://github.com/WayneMao/Markdown-images/blob/master/ROS%E5%AD%A6%E4%B9%A0/ROS2_three%20turtles.PNG?raw=true)

我们打开三个新的Terminal，分别输入：

> rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]
>
> rostopic pub -r 10 /turtle2/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]
>
> rostopic pub -r 10 /turtle4/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]

此时三只小海龟边能进行圆周运动了，其结果如下图所示：(下图为GIF动图)

![ROS2_ä¸åªå°æµ·é¾.gif](https://github.com/WayneMao/Markdown-images/blob/master/ROS%E5%AD%A6%E4%B9%A0/ROS2_%E4%B8%89%E5%8F%AA%E5%B0%8F%E6%B5%B7%E9%BE%9F.gif?raw=true)



`rqt_graph`查看三只海龟的计算图结构 ：

![ROS2_ä¸åªæµ·é¾è®¡ç®å¾.PNG](https://github.com/WayneMao/Markdown-images/blob/master/ROS%E5%AD%A6%E4%B9%A0/ROS2_%E4%B8%89%E5%8F%AA%E6%B5%B7%E9%BE%9F%E8%AE%A1%E7%AE%97%E5%9B%BE.PNG?raw=true)

 `rqt_plot`查看某只海龟运动时候的位置曲线：(在Topic输入/turtle1/pose 查看第一只小海龟的位置曲线) 

![ROS2_ä½ç½®æ²çº¿.PNG](https://github.com/WayneMao/Markdown-images/blob/master/ROS%E5%AD%A6%E4%B9%A0/ROS2_%E4%BD%8D%E7%BD%AE%E6%9B%B2%E7%BA%BF.PNG?raw=true)