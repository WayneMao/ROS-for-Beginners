#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msg/PointStamped.h>

int main(int argc, char**argv){
	// init node
	ros::init(argc, argv, "robot_tf_listener");
	
	// 
	ros::NodeHandle node;
	
	// build listener
	tf::TransformListenser listener;
	
	ros::Rate rate(10.0);
	while (node.ok()){
		//创建激光数据点
		geometry_msg::PointStamped laser_point;
		laser_point.header.frame_id = "base_laser;
		laser_point.header.stamp = ros::Time();
		
		// 设置激光数据点的坐标
		laser_point.point.x = 0.3;
		laser_point.point.y = 0.0;
		laser_point.point.z = 0.0;
		
		try{
			// 监听base_link坐标系和base_laser坐标系之间的位姿变换
			listener.waitForTransform("base_link", "base_laser", ros::Time(0), ros::Duration(3.0));
			geometry_msg::PointStamped base_point;
			
			//将激光坐标下的点坐标转换到机器人坐标下
			listener.transformPoint("base_link", laser_point, base_laser);
			
			Ros_INFO("base_laser": (%.2f, %.2f, %.2f) -----> base_link:(%.2f, %.2f, %.2f) at time %.2f",
				laser_point.point.x, laser_point.point.y, laser_point.point.z, 
				base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec();
		}
		catch(tf::TransformException ex){
			ROS_ERROR("Received an ecpection trying to transform a point form \"base_lase\" to "base_link\": %s", ex.what());
		}
		
		rate.sleep();
	}
	return 0;
};
			