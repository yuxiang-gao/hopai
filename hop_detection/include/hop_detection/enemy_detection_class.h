#ifndef HD_DEPTH_OBSTACLE_DETECTION_CLASS_H
#define HD_DEPTH_OBSTACLE_DETECTION_CLASS_H

#include <vector>
#include <cmath>
#include <ros/ros.h>
//#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <tf/tf.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float64.h>

#include <dynamic_reconfigure/server.h>
#include <hop_detection/HopDetectionConfig.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry> 
#include <Eigen/Dense>
#include <Eigen/Core>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace hop_detection
{
struct ColorDetectParams
{
	double light_max_aspect_ratio;
	double light_min_aspect_ratio;
	double light_min_area;
	int light_max_area;
	double light_max_angle;
	double light_max_angle_diff;
	double light_length_ratio;
	
	double armor_max_angle;
	int armor_min_area;
	double armor_max_aspect_ratio;
	double armor_max_pixel_val;
	double armor_max_stddev;
	
	double armor_blank_max_angle;
	int armor_blank_min_area;
	double blank_light_length_ratio;
	
};

class EnemyDetection
{
public: 
    EnemyDetection(ros::NodeHandle *nh, ros::NodeHandle *nh_priv, const std::string & name);
    virtual ~EnemyDetection(){};   
private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    ros::Subscriber cloud_sub_;
    std::string name_;
    dynamic_reconfigure::Server<hop_detection::HopDetectionConfig> dyn_cfg_server_;
    dynamic_reconfigure::Server<hop_detection::HopDetectionConfig>::CallbackType dyn_cfg_f_;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    void dynCfgCallback(hop_detection::HopDetectionConfig& config, uint32_t level);
    
}; // class EnemyDetection
} // namespace hop_detection

#endif
