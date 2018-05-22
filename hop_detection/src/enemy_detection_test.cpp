#include <ros/ros.h>
#include "hop_detection/enemy_detection_class.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "enemy_detection");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	
	hop_detection::EnemyDetection ed(nh, pnh, "enemy_detection");
	
	
	ros::spin();
	return 0;

}

