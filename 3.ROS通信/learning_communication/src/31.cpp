#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

void poseCallback(const turtlesim::ConstPtr& msg)
{
	// 将接收到的消息打印出来
	ROS_INFO("Turtle Pose: x-[%f]; Y-[%f]",msg->x,msg->y);
}

int main(int argc, chaer **argv)
{
	// ROS 节点初始化
	ros::init(argc, argv, "turtle_cmd");
	
	// 创建节点句柄
	ros::NodeHandle nh_;
	
	// 创建一个发布者，命名为turtle/cmd_vel的话题， 消息类型为geometry_msgs::Twist
	ros::Publisher twist_pub_;
	twist_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle/cmd_vel",1);
	
	// 创建一个订阅者，回调函数poseCallback
	ros::Subsctiber pose_sub_ = nh_.subscribe("/turtle/pose", 1, poseCallback);
	
	// 设置循环频率
	ros::Rate loop_rate(10);
	
	while(ros::ok())
	{
		// 初始化 geometry_msgs::Twist类型 消息
		geometry_msgs::Twist twist;
		twist.linear.x = 2;
		twist.angular.z = 1;
		
		// 发布消息
		twist_pub_.publish(twist);
		
		// 循环等待回调函数
		ros::spinOnce();
		
		// 按照循环频率延时
		loop_rate.sleep();
	}
	return 0;
}
