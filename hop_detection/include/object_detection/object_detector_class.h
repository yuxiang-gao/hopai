#include <ros/ros.h>
#include <ros/package.h>
#include "object_detection/fhog_object_detector/fhog_object_detector.h"
#include "object_detection/tracking/multi_track.h"

namespace hop_detection
{
namespace object_detectors
{
//template<class T>
class ObjectDetectorClass
{
public:
    typedef boost::shared_ptr<ObjectDetectorClass> Ptr;
    ObjectDetectorClass(ros::NodeHandle *nh, ros::NodeHandle *pnh, int cam_id):
        nh_ptr_(boost::make_shared<ros::NodeHandle>(*nh)),
        pnh_ptr_(boost::make_shared<ros::NodeHandle>(*pnh, "enemy_detection")),
        camera_id_(cam_id),
        ns_("fhog_object_detector")

    {
        std::vector<std::string> detectors;
        
        pnh_ptr_->getParam(ns + "/display", display_);
        pnh_ptr_->getParam(ns + "/detectors", detectors);
        pnh_ptr_->getParam(ns + "/threshold", threshold_ );

        std::string path = ros::package::getPath("hop_detection");
        for (auto &detector : detectors)
        {
            detector = path + detector;
        }

        detector_ptr_ = boost::make_shared<FHOGObjectDetector> (detectors, display_);
        if (threshold_ != 0.0)
        {
            detector_ptr_->setThreshold(threshold_);
        }
        tracking_ptr_ = boost::make_shared<TrackingSystem>(2);
    }

    std::vector<cv::Rect> detect(const cv::Mat& image_in)
    {
        std::vector<dlib::rectangle> detections = tracking_ptr_->update(image_in, detector_ptr_->detect(image_in));
        std::vector<cv::Rect> cv_detections;
        for (auto & d : detections)
        {
            cv_detections.push_back(Utils::dRectToCvRect(d));
        }
        if (display_)
            detector_ptr_->dispaly(detections;
        return cv_detections;
    }
private:
    ros::NodeHandlePtr nh_ptr_;
    ros::NodeHandlePtr pnh_priv_;
    int camera_id_;
    std::string ns_;
    bool display_;
    double threshold_;
    FHOGObjectDetector::Ptr detector_ptr_;
    TrackingSystem::Ptr tracking_ptr_;
    

}; // class FHOGObjectDetectorClass

//typedef boost::shared_ptr<ObjectDetectorClass<FHOGObjectDetector>> FHOGObjectDetectorPtr;
} // namespace detector
} // namespace hop_detection
