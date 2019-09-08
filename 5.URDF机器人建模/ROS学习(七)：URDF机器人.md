## ROS学习(七)：URDF机器人

### 1. 机器人的定义与组成

### 2. URDF建模方法

**<link>**

<joint>

<robot>



### 3. 实践

**创建一个机器人功能包** (~/catkin_ws/src)

```shell
catkin_create_pkg mbot_description urdf xacro
```

包含以下文件夹:

- urdf
- meshes
- launch
- config



启动：

```shell
roslaunch mbot_description display_mbot_base_urdf.launch
```



举个例子

```xml
<?xml version="1.0" ?>
<robot name="mbot">

    <link name="base_link">
        <visual>
            <origin xyz=" 0 0 0" rpy="0 0 0" />
            <geometry>
                <cylinder length="0.16" radius="0.20"/>
            </geometry>
            <material name="yellow">
                <color rgba="1 0.4 0 1"/>
            </material>
        </visual>
    </link>

    <joint name="left_wheel_joint" type="continuous">
        <origin xyz="0 0.19 -0.05" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="left_wheel_link"/>
        <axis xyz="0 1 0"/>
    </joint>

    <link name="left_wheel_link">
        <visual>
            <origin xyz="0 0 0" rpy="1.5707 0 0" />
            <geometry>
                <cylinder radius="0.06" length = "0.025"/>
            </geometry>
            <material name="white">
                <color rgba="1 1 1 0.9"/>
            </material>
        </visual>
    </link>
</robot>
```

`rpy` 旋转的弧度，

`rgba`颜色RGB，a是透明度





**检查URDF模型整体结构**

```shell
urdf_to_graphiz # 先看看是否安装

# 若没有安装，则
sudo apt install liburdfdom-tools

# 相应文件夹下 run
urdf_to_graphiz mbot_base.urdf
```

