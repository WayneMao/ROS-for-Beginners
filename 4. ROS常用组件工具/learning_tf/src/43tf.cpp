#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
	// init node
	ros::init(argc, argv, "robot_tf_broadcaster");
	
	//
	ros::NodeHandle nh;
	
	static tf::TransformBroadcaster br;
	
	while(node.ok()){
		tf::Transform transform;
		transform.setOrigin(tf::Vector(0.1, 0.0, 0.2));
		transform.setRotation( tf::Quaternion(0,0,0,1)):
		
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", “base_laser"));
	}
	return 0;
}