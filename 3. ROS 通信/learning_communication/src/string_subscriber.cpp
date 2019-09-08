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


