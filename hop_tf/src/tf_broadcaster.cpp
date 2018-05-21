// This node takes care of the tf between the base link and the gimbal
// Version: 0.1
// Author: RUi

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "hopai_msgs/GimbalAngle.h"

void gimbalCallback(hopai_msgs::GimbalAngle& msg){
    tf::TransfromBroadcaster br;
    tf::Transform base_gimbal_tf;
    base_gimbal_tf.setOrigin(0.13,0,0.24);
    tf::Quaternion q;
    q.setRPY(0,msg.pitch,msg.yaw);
    base_gimbal_tf.setRotation(q);
    br.sendTransform(tf::StampedTransfrom(base_gimbal_tf,ros::Time::now()),"base_link","gimbal_link");
}

int main(int argc, char** argv){
  ros::init(argc, argv, "hopai_tf_broadcaster");
  ros::NodeHandle node;


  ros::Subscriber sub = node.subscribe("gimbal_angle", 10, &gimbalCallback);
  
  ros::spin();
  return 0;
};
