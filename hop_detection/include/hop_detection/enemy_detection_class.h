#ifndef HD_DEPTH_OBSTACLE_DETECTION_CLASS_H
#define HD_DEPTH_OBSTACLE_DETECTION_CLASS_H

#include <mutex>
#include <vector>
#include <cmath>
#include <thread>
#include <queue>
#include <functional>
#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
//#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

#include <tf/tf.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float64.h>

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
using namespace message_filters::sync_policies;
class EnemyDetection
{
public: 
    EnemyDetection(ros::NodeHandle &nh, ros::NodeHandle &nh_priv, const std::string & name);
    virtual ~EnemyDetection(){};   
private:

    ros::NodeHandlePtr nh_;
    ros::NodeHandlePtr pnh_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber image_sub_;

    image_transport::SubscriberFilter sub_cam_, sub_cam1_, sub_cam2_, sub_cam3_, sub_cam4_;
    //message_filters::Subscriber<sensor_msgs::CameraInfo> sub_cam0_info_, sub_cam1_info_, sub_cam2_info_, sub_cam3_info_, sub_cam4_info_;
    typedef ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ExactPolicy;
    typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ApproximatePolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    boost::shared_ptr<ExactSync> exact_sync_;
    boost::shared_ptr<ApproximateSync> approximate_sync_;

    ros::Subscriber cloud_sub_;
    std::string name_;

    // boost::ptr_vector<boost::mutex> queue_locks_;
    // for (int i=0; i++; i<5)
    //     queue_locks_.push_back(new boost::mutex);
    boost::shared_mutex queue_lock_;
    std::vector<std::queue<boost::shared_ptr<const sensor_msgs::Image>>> image_queues_;

    int queue_length_；

    void imageCallback(const sensor_msgs::ImageConstPtr& msg0,
                        const sensor_msgs::ImageConstPtr& msg1,
                        const sensor_msgs::ImageConstPtr& msg2,
                        const sensor_msgs::ImageConstPtr& msg3,
                        const sensor_msgs::ImageConstPtr& msg4);
    void service();
    void process_one(const sensor_msgs::ImageConstPtr& msg, int cam_id);
    
}; // class EnemyDetection
} // namespace hop_detection

#endif
