#ifndef HD_DEPTH_OBSTACLE_DETECTION_NODELET_H
#define HD_DEPTH_OBSTACLE_DETECTION_NODELET_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "hop_detection/enemy_detection_class.h"

namespace hop_detection
{
class EnemyDetectionNodelet: public nodelet::Nodelet
{

public:
    EnemyDetectionNodelet(){}
private:
    ~EnemyDetectionNodelet(){}
    virtual void onInit();
    boost::shared_ptr<EnemyDetection> inst_;

}; // class EnemyDetectionNodelet
} // namespace hd_depth
#endif
