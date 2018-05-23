// This node takes care of the tf between the base link and the gimbal
// Version: 0.1
// Author: RUi

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "hop_msgs/GimbalAngle.h"

void gimbalCallback(const hop_msgs::GimbalAngle& msg){
    tf::TransformBroadcaster br;
    tf::Transform base_gimbal_tf;
    base_gimbal_tf.setOrigin(tf::Vector3(0.13,0.0,0.24));
    tf::Quaternion q;
    q.setRPY(0,msg.pitch,msg.yaw);
    base_gimbal_tf.setRotation(q);
    br.sendTransform(tf::StampedTransform(base_gimbal_tf,ros::Time::now(),"base_link","gimbal_link"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "hop_tf_broadcaster");
  ros::NodeHandle node;


  ros::Subscriber sub = node.subscribe("gimbal_angle", 10, &gimbalCallback);
  
  ros::spin();
  return 0;
};
