

## ROS学习(六)：Launch,TF坐标变换,视化显示与仿真工具

### 1. Launch 启动文件

在ROS运行时很多时候需要打开多个终端，比较繁琐。为了解决这个问题，我们可以使用Launch文件来启动脚本。

Launch文件：通过XML文件实现多节点的的配置和启动(**自动启动ROS Master**)。如果命令行没有运行`roscore`, 那么`roslanuch`会启动`/roscore`节点；如果有运行`roscore`,那么`roslanuch`不会再启动`/roscore`节点。

```shell
<launch>

  <group ns="turtlesim1">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>

</launch>
```

`catkin_make`编译之后，运行相应的launch文件

`roslaunch learning_launch xxx.launch`



> - pkg: 节点所在功能包名称
> - type: 节点可执行文件名称 # .py 后缀不可省略
> - name: 节点运行时名称 # 将覆盖节点中init()赋予节点的名称
> - output = "screen"：将节点的标准输出打印到终端屏幕，默认输出为日志文档；
> - respawn = "true"：复位属性，该节点停止时，会自动重启，默认为false；
> - required = "true"：必要节点，当该节点终止时，launch文件中的其他节点也被终止；
> - ns = "namespace"：命名空间，为节点内的相对名称添加命名空间前缀；
> - args = "arguments"：节点需要的输入参数。



### 2. 参数

\<param>、\<arg> 都为参数，但意义完全不同

```python
# 获取全局参数
rospy.get_param('/global_param_name')

# 获取目前命名空间的参数
rospy.get_param('param_name')

# 获取私有命名空间参数
rospy.get_param('~private_param_name')
# 获取参数，如果没，使用默认值
rospy.get_param('foo', 'default_value')
```







### 3. TF坐标变换

参考：《机器人学导论》



两只海龟跟随运动：(不完全跟随，带有路径优化)

```shell
sudo apt-get install ros-melodic-turtle-tf
roslaunch turtle_tf turtle_tf_demo.launch
rosrun turtlesim turtle_teleop_key  # 打开一个按键控制 
rosrun tf view_frames # 当前路径下产生一个 frame.pdf 文件， 揭示 两只海龟之间的关系
```



```shell
rosrun tf tf_echo turtle1 turtle2 # 查看两个值的位置信息、关系:平移、旋转
```

> At time 1567404677.720
> - Translation: [0.000, 0.000, 0.000]
> - Rotation: in Quaternion [0.000, 0.000, -0.227, 0.974] # 四元数
>             in RPY (radian) [0.000, 0.000, -0.458]   # 弧度
>             in RPY (degree) [0.000, 0.000, -26.266]  # 角度
>
> Rotation:



调用可视化工具Rviz：**Rviz** 三维可视化工具
```shell
rosrun rviz rviz -d `rospack find turtle_tf` /rviz/turtle_rviz.rviz
```

四元数，一个向量 + 一个角度，



### 4. 实战演练

(1) 完成[ROS通信 ](../3.ROS通信) **服务器客户端综合应用.md**中三道题目的启动和测试，将每道题中所使用的`rosrun`命令替换为一个`roslaunch`命令。

a. 运行如下命令： 

```shell
  roslaunch learning_communication 31py.launch 
```

结果：将会出现一只做圆周运动的小海龟,并且在终端中输出打印海龟位置

b. 运行如下命令：

```shell
roslaunch learning_communication 32py.launch
```

将会在产生一只新的海龟

c. 

修改了之前的 [33.py](learning_communication\scripts) 代码 , 并新建了 33py.launch 文件

```python
<launch>
     <node pkg="turtlesim" type="turtlesim_node" name="sim" />
     <node pkg="learning_communication" type="33.py" name="new" />
        <param name='name' value='alpha' />
</launch>

```

想要修改新产生turtle的名字就在 33py.launch 中修改 param 的value。

运行如下命令：

```shell
roslaunch learning_communication 33py.launch
> alpha 2.0 3.0  # 输入相应的海龟名字 及其 速度 方向
```

结果：随机位置产生一只名为“alpha” 的新海龟，

输入 相应的海龟名字 及其 速度 方向后，这只海龟开始做圆周运动。



33new.py + 33pynew.launch






**Reference:**

[ROS技术点滴 —— launch文件](https://mp.weixin.qq.com/s/qY_NpuEiKl5cDH0NexyP5g)

关于四元数的讲解下面这篇文章讲得不错

[【Unity技巧】四元数（Quaternion）和旋转](https://blog.csdn.net/candycat1992/article/details/41254799)

四元数与三维空间: 如何形象地理解四元数？ - 知乎
https://www.zhihu.com/question/23005815/answer/483589712