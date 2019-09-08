#include "ros/ros.h"
#include "turtlesim/Spawn.h"

int main(int argc, char **argv)
{
	// init node
	ros::init(argc, argv, "new_turtle");
	
	ros::NodeHandle nh_;
	
	ros::ServiceClient spawnClient = nh_.serviceClient<turtlesim::Spawn>("spawn");
	
	// 创建turtlesim::Spawn 类型的服务
	turtlesim::Spawn srv;
	srv.request.x = 5.0;
	srv.request.y = 5.0;
	srv.request.theta = 2.0;
	
	// 发布service请求，等待应答结果
	if(spawnClient.call(srv))
	{
		ROS_INFO("A new turtle named '%S' was spawned", srv.request.name.c_str());
	}
	else
	{
		ROS_ERROR("Failed to call service spawn");
		return 1;
	}
	
	return 0;
}
