#include "hop_detection/enemy_detection_class.h"
#include <std_msgs/Bool.h>

#define DEBUG 1
#if DEBUG
#include <chrono>
#endif

namespace hop_detection
{
EnemyDetection::EnemyDetection (ros::NodeHandle &nh, ros::NodeHandle &pnh, const std::string & name):
    nh_(boost::make_shared<ros::NodeHandle>(nh)),
    pnh_(boost::make_shared<ros::NodeHandle>(pnh)),
    name_(name),
    queue_length_(2)
    //is_depth_(is_depth)
{
    ROS_DEBUG_ONCE_NAMED(name_, "Starting obstacle avoidance.");
    it_ = boost::make_shared<image_transport::ImageTransport>(*nh_);
    pnh_->getParam("side_cam", side_cam);
    if (side_cam)
        object_detect_ = boost::make_shared<ObjectDetectorClass>(*nh_, *pnh_, 0);
    else
        color_detect_ = boost::make_shared<ColorDetection>(*nh_, *pnh_, 0);

    enemy_pos_pub_ = nh_->advertise<std_msgs::Bool>("enemy", 1);
    armor_pos_pub_ = nh_->advertise<EnemyPos>("enemy_pos", 1);
    int queue_size;
    
    pnh_->param("queue_size", queue_size, 5);
    pnh_->param("use_depth", is_depth_, false);

    if (is_depth_)
    {
        bool approx;
        pnh_->param("approximate_sync", approx, false);

        if (approx)
        {
            approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size),
                                                        sub_cam_, sub_depth_) );
            approximate_sync_->registerCallback(boost::bind(&EnemyDetection::depthCallback,
                                                            this, _1, _2));
        }
        else
        {
            exact_sync_.reset( new ExactSync(ExactPolicy(queue_size),
                                            sub_cam_, sub_depth_) );
            exact_sync_->registerCallback(boost::bind(&EnemyDetection::depthCallback,
                                                            this, _1, _2));
        }

        sub_cam_.subscribe(*it_, "image_in", 1);
        sub_depth_.subscribe(*it_, "depth_in", 1);
    }
    else
    {
        image_sub_ = it_->subscribe("image_in", 1, &EnemyDetection::imageCallback, this);
    }
} //constructor

void EnemyDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // boost::upgrade_lock<boost::shared_mutex> lock(queue_lock_);
    // boost::upgrade_to_unique_lock<boost::shared_mutex> unique_lock(lock);
    // if (image_queue_.size() == queue_length_)
    //     image_queue_.pop();
    // image_queue_.push(msg);
    sensor_msgs::ImageConstPtr this_depth;
    this_depth = NULL;
    process_one(msg, this_depth);
}

void EnemyDetection::depthCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::ImageConstPtr& depth_msg)
{
    // boost::upgrade_lock<boost::shared_mutex> lock(queue_lock_);
    // boost::upgrade_to_unique_lock<boost::shared_mutex> unique_lock(lock);
    // if (image_queue_.size() == queue_length_)
    //     image_queue_.pop();
    // image_queue_.push(msg);

    // if (depth_queue_.size() == queue_length_)
    //     depth_queue_.pop();
    // depth_queue_.push(msg);
    process_one(msg, depth_msg);
}
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

/*
void EnemyDetection::service()
{
    ROS_INFO("Deteection: Started service thread\n");
    sensor_msgs::ImageConstPtr this_msg;
    sensor_msgs::ImageConstPtr this_depth;

    this_msg = NULL;
    this_depth = NULL;
    //boost::shared_lock<boost::shared_mutex> lock(queue_lock_);
    // queue_lock_.lock();
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
    if (is_depth_)
    {
        for (auto & q : depth_queues_)
        {
            if (q.size() > 0)
            {
                this_depth = q.front();
                q.pop();
            }
            else
                this_depth = NULL;
        }
    }
    // queue_lock_.unlock();
    if (this_msg)
    {
        ROS_DEBUG("Processing message");
        process_one(this_msg, this_depth);
        this_msg = NULL;
    }
    else
        ROS_DEBUG"Waiting for message");
}
*/
void EnemyDetection::process_one(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::ImageConstPtr& depth_msg)
{
    cv_bridge::CvImagePtr src_ptr;
    cv_bridge::CvImagePtr depth_ptr;
    int image_height = msg->height;
    int image_width = msg->width;
    double distance;
    double pitch;
    double yaw;

    try 
    {
        src_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        if (is_depth_ && depth_msg != NULL) 
        {
            depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            color_detect_->updateDepth(depth_ptr->image);
        }
        if (!side_cam)
        {
			color_detect_->updateFrame(src_ptr->image);
	        if (color_detect_->detectArmor(distance, pitch, yaw).IsOK())
	        {
	            //TODO
	            EnemyPos pos_msg;
	            pos_msg.enemy_dist = distance;
	            pos_msg.enemy_pitch = pitch;
	            pos_msg.enemy_yaw = yaw;
	            enemy_pos_pub_.publish(pos_msg);
	        }
		}
		else
		{
			
			std::vector<cv::Rect> result = object_detect_->detect(src_ptr->image);
			if (result.size() > 0)
			{
				enemy_pos_pub_.publish(true);
			}
			else
				enemy_pos_pub_.publish(false);
		}
//TODO

    } 
    catch(cv_bridge::Exception & e) 
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}
} // namespace hd_depth
