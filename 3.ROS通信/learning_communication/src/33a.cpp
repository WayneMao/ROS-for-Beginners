#include <cstdlib>
#include <ctime>
#include "ros/ros.h>
#include "turtlesim/Spawn.h"

int main(int argc,char **argv){
	// init node
	ros::init(argc, argv, "client_generate_turtleWithParameter");
	
	if (argc !=2)
	{
		ROS_INIF("Please input the turtle's name behind the command");
		return 1;
	}
	
	//
	ros::NodeHandle n;
	
	//创建一个SpawnClient，服务类型 turtlesim::Spawn
	ros::ServiceClient spawnClient = n.serviceClient<turtlesim::Spawn>("/spawn");
	
	//创建 turtlesim::Spawn类型的服务消息
	turtlesim::Spawn srv;
	srand(static_cast<unsigned int>(time(NULL)));
	srv.request.x = float(rand())/float(RAND_MAX)*11;
	srv.request.y = float(rand())/float(RAND_MAX)*12;
	srv.request.theta = 0;
	srv.request.name = argv[1];
	
	//   发布service请求，等待应答结果
	if(spawnClient.call(srv)){
		ROS_INFO("A new turtle named '%s' was spawned",srv.response.name.c_str());
	}
	else
	{
		ROS_ERROR("Failed to call service spawn");
		return 1;
	}
	
	return 0;
}