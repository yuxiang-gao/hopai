#include "hop_detection/enemy_detection_class.h"

#define DEBUG 1
#if DEBUG
#include <chrono>
#endif

namespace hop_detection
{
EnemyDetection::EnemyDetection (ros::NodeHandle &nh, ros::NodeHandle &pnh, const std::string & name):
    nh_(boost::make_shared<ros::NodeHandle>(nh)),
    pnh_(boost::make_shared<ros::NodeHandle>(pnh)),
    name_(name)，
    queue_length_(2)
{
    ROS_DEBUG_ONCE_NAMED(name_, "Starting obstacle avoidance.");
    it_ = boost::make_shared<image_transport::ImageTransport>(nh_));

    // image0_sub_ = it_.subscribe("/camera/left", 2, boost::bind(&EnemyDetection::imageCallback, this, _1, 0));
    // image1_sub_ = it_.subscribe("/camera/right", 2, boost::bind(&EnemyDetection::imageCallback, this, _1, 1));
    // image2_sub_ = it_.subscribe("/camera/middle", 2, boost::bind(&EnemyDetection::imageCallback, this, _1, 2));
    // image3_sub_ = it_.subscribe("/primesense/rgb/image_rect_color", 2, boost::bind(&EnemyDetection::imageCallback, this, _1, 3));
    // image4_sub_ = it_.subscribe("/primesense/depth", 2, boost::bind(&EnemyDetection::imageCallback, this, _1, 4));
    // nh_priv_.param("openni_enc", openni_enc_, openni_enc_);
    // image_pub_ = it_.advertise("out", 1);
    //obstacle_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/hd/perception/stereo_obstacle", 10);
    //repulsive_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/hd/perception/potfield/stereo_obstacle", 10);
    int queue_size;
    pnh_->param("queue_size", queue_size, 5);
    bool approx;
    pnh_->param("approximate_sync", approx, false);

    if (approx)
    {
        approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size),
                                                    sub_cam0_, sub_cam1_,
                                                    sub_cam2_, sub_cam3_, sub_cam4_) );
        approximate_sync_->registerCallback(boost::bind(&EnemyDetection::imageCallback,
                                                        this, _1, _2, _3, _4, _5));
    }
    else
    {
        exact_sync_.reset( new ExactSync(ExactPolicy(queue_size),
                                        sub_cam0_, sub_cam1_,
                                                    sub_cam2_, sub_cam3_, sub_cam4_) );
        exact_sync_->registerCallback(boost::bind(&EnemyDetection::imageCallback,
                                                    this, _1, _2, _3, _4, _5));
    }

    sub_cam0_.subscribe(*it_, "camleft", 1);
    sub_cam1_.subscribe(*it_, "camright", 1);
    sub_cam2_.subscribe(*it_, "cammiddle", 1);
    sub_cam3_.subscribe(*it_, "ps", 1);
    sub_cam4_.subscribe(*it_, "depth", 1);

} //constructor

// void EnemyDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg, int cam_id)
// {
// #if DEBUG
//     std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
// #endif
//     cv_bridge::CvImageConstPtr cv_ptr;
//     try
//     {
//         cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::B8);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR_NAMED(name_, "cv_bridge exception: %s", e.what());
//         return;
//     }
    

// #if DEBUG
//     std::chrono::steady_clock::time_point t2 =std:: chrono::steady_clock::now();
//     std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>( t2-t1 );
//     //OS_DEBUG_STREAM_NAMED(name_, "Time used: " << time_used.count() << " s.");
//     std::cout<<"Time used: "<<time_used.count()<<" s."<<std::endl;
// #endif

// } // image_callback
// void EnemyDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg, int cam_id)
// {
//     boost::upgrade_lock<boost::shared_mutex> lock(queue_lock_);
//     boost::upgrade_to_unique_lock<boost::shared_mutex> unique_lock(lock);
//     if (image_queues_[cam_id].size() == queue_length_)
//         image_queues_[cam_id].pop();
//     image_queues_[cam_id].push(msg);
    
// }

void EnemyDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg0,
                                    const sensor_msgs::ImageConstPtr& msg1,
                                    const sensor_msgs::ImageConstPtr& msg2,
                                    const sensor_msgs::ImageConstPtr& msg3,
                                    const sensor_msgs::ImageConstPtr& msg4)
{
    boost::upgrade_lock<boost::shared_mutex> lock(queue_lock_);
    boost::upgrade_to_unique_lock<boost::shared_mutex> unique_lock(lock);
    for (int i = 0; i < 5; i++)
    {
        if (image_queues_[i].size() == queue_length_)
            image_queues_[i].pop();
        image_queues_[i].push(msg);
    }
}
void EnemyDetection::service(int cam_id)
{
    ROS_INFO("Deteection: Started service thread\n");
    sensor_msgs::ImageConstPtr thisMsg;
    while (true)
    {
        this_msg = NULL;
        boost::shared_lock<boost::shared_mutex> lock(queue_lock_);
        for (auto & q : image_queues_)
        {
            if (q.size() > 0)
            {
                this_msg = q.front();
                q.pop();
            }
            else
                this_msg = NULL;
        }

        if (thisMsg)
        {
            //ROS_INFO("Processing message");
            process_one(this_msg);
            this_msg = NULL;
        }
        else
        {
            //No message to process yet...
            usleep(1000);
        }
        
    }
}

void EnemyDetection::process_one(const sensor_msgs::ImageConstPtr& msg, int cam_id)
{

}

} // namespace hd_depth
