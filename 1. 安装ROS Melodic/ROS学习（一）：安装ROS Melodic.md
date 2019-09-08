![melodic](http://wiki.ros.org/melodic?action=AttachFile&do=get&target=melodic.jpg)

### ROS学习（一）：安装ROS Melodic

ROS(Robot Operating System)是机器人操作系统的缩写。

此篇文章作为ROS学习笔记的第一篇，简要说明一下。学习平台主要依赖Ubuntu 18.04 LTS，ROS 选择了Melodic版本。

Ubuntu的安装网上有较为丰富的资料，可以参考其他的资源。

#### 1. 安装ROS Melodic

ROS安装的时候主要参考的是[Ubuntu install of ROS Melodic ](http://wiki.ros.org/melodic/Installation/Ubuntu)

在这之前需要先配置软件源

打开Software & Updates,确保Ubuntu Software里面各项都是勾选上的

![Ubuntu Software](C:\Users\mwx\Pictures\Ubuntu Software.png)

##### 1.1 添加ROS软件源

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

##### 1.2 设置秘钥

```shell
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

##### 1.3 安装ROS

安装之前先更新

```shell
sudo apt update
```

此时选择**完整安装**，这也是比较推荐的一种安装方式，其他方式有兴趣的可以以后再修改。

```shell
sudo apt install ros-melodic-desktop-full
```

##### 1.4 初始化rosdep

```shell
sudo rosdep init
rosdep update
```

##### 1.5 环境配置

我们需要让Ubuntu知道到哪里去寻找命令对应的执行程序。

```shell
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

##### 1.6 构建包的依赖关系

```shell
sudo apt install python-rosinstall python-rosinstall-generator python -wstool build-essential
```

### 2.小海龟仿真

一般在安装完成后我们可以运行一下小海龟，看是否安装成功

1. 打开一个终端，必须首先运行`roscore`命令，回车；
2. roscore终端的窗口不关闭，打开一个新的终端，输入命令：`rosrun turtlesim turtlesim_node`，回车；
3. 再打开第三个终端，输入：`rosrun turtlesim turtle_teleop_key`  回车。//当用上下左右四个箭头操作小乌龟前，必须把鼠标放在含有命令`osrun turtlesim turtle_teleop_key`终端上。

运行结果如下图所示：

![小海龟](C:\Users\mwx\Pictures\小海龟.PNG)

