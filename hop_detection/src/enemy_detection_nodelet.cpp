#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "hop_detection/enemy_detection_nodelet.h"

PLUGINLIB_EXPORT_CLASS(hop_detection::EnemyDetectionNodelet, nodelet::Nodelet)
namespace hop_detection
{
void EnemyDetectionNodelet::onInit()
{
    NODELET_DEBUG("Initializing nodelet");
    inst_.reset(new EnemyDetection(&getNodeHandle(), &getPrivateNodeHandle(), getName()));
} // class 
} // namespace hd_depth
