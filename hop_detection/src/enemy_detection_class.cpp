#include "hop_detection/enemy_detection_class.h"

#define DEBUG 1
#if DEBUG
#include <chrono>
#endif

namespace hop_detection
{
EnemyDetection::EnemyDetection (ros::NodeHandle *nh, ros::NodeHandle *nh_priv, const std::string & name):
    nh_(*nh), 
    nh_priv_(*nh_priv), 
    it_(nh_), 
    name_(name)
{
    ROS_DEBUG_ONCE_NAMED(name_, "Starting obstacle avoidance.");
    // start dynamic reconfigure 
    dyn_cfg_f_ = boost::bind(&EnemyDetection::dynCfgCallback, this, _1, _2);
    dyn_cfg_server_.setCallback(dyn_cfg_f_);

    image_sub_ = it_.subscribe("/primesense/rgb/image_rect_color", 1, &EnemyDetection::imageCallback, this);
    // nh_priv_.param("openni_enc", openni_enc_, openni_enc_);
    // image_pub_ = it_.advertise("out", 1);
    //obstacle_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/hd/perception/stereo_obstacle", 10);
    //repulsive_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/hd/perception/potfield/stereo_obstacle", 10);

} //constructor

void EnemyDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
#if DEBUG
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#endif
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR_NAMED(name_, "cv_bridge exception: %s", e.what());
        return;
    }
    

#if DEBUG
    std::chrono::steady_clock::time_point t2 =std:: chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>( t2-t1 );
    //OS_DEBUG_STREAM_NAMED(name_, "Time used: " << time_used.count() << " s.");
    std::cout<<"Time used: "<<time_used.count()<<" s."<<std::endl;
#endif

} // image_callback

void EnemyDetection::dynCfgCallback(hop_detection::HopDetectionConfig& config, uint32_t level)
{
	
}

} // namespace hd_depth
