#include "ros/ros.h"
#include <string>
#include "geomerty_msgs/Twist.h>

int main(int argc, char **argv)
{
	std::string turtle_name = argv[2];
	
	//init node
	ros::init(argc, argv, turtle_name);
	
	if(argc != 3 && argc!=5){
		ROS_INFO("please input the 'stop' 'the name of turtle' or \
		please input the 'start' 'the name of turtle' 'linear velocity' 'angular vrlocity'");
		return 1;
	}
	
	//创建节点句柄
	ros::NodeHandle n;
	std::string vel_topic = turtle_name + "/cmd_vel";
	
	// 创建一个发布者 velocity_circle_pub, 发布的消息类型为 geometry_msgs::Twist
	ros::string velocity_circle_pub = n.advertise<geomerty_msgs::Twist>(vel_topic,1000);
	
	// loop rate
	ros::Rate loop_rate(10);
	
	while(ros::ok())
	{
		//创建 geometry_msgs::Twist类型的消息
		geomerty_msgs::Twist msg;
		
		if(!strcmp(argv[1],"start") || !strcmp(argv[1],"START"))
		{
			msg.linear.x = atol(argv[3]);
			msg.angular.z = atol(argv[4]);
			ROS_INFO("publish velocity: \n linear.x %1f\n angular.z %1f", msg.linear.x, msg.angular.z);
			velocity_circle_pub.publish(msg);
		}
		else if(!strcmp(argv[1],"stop") || !strcmp(argv[1],"STOP"))
		{
			msg.linear.x = 0;
			msg.angular.z = 0;
			ROS_INFO("STOP");
			velocity_circle_pub.publish(msg);
		}
		else{
			ROS_INFO("Please input the right command 'start' or 'START' or 'stop' or 'STOP'");
			return 1;
		}
		
		//循环延时
		loop_rate.sleep();
	}
	return 0;
}
