#include <ros/ros.h>
#include <ros/package.h>
#include "fhog_object_detector/fhog_object_detector.h"

namespace hop_detection
{
namespace object_detectors
{
template<class T>
class ObjectDetectorClass
{
public:
    ObjectDetectorClass(int cam_id):
        nh_ptr_(new ros::NodeHandle),
        pnh_ptr_(new ros::NodeHandle("~enemy_detection")),
        camera_id_(cam_id),
        ns_("fhog_object_detector")

    {
        std::string path = ros::package::getPath("hop_detection");
        pnh_ptr_->getParam(ns + "/display", display_);
        pnh_ptr_->getParam(ns + "/detectors", detectors_);
        pnh_ptr_->getParam(ns + "/threshold", threshold_ );
        for (auto &detector : detectors)
        {
            detector = path + detector;
        }
        detector_ptr_ = boost::make_shared<T> (detectors_, display_);
        if (threshold_ != 0.0)
        {
            detector_ptr_->setThreshold(threshold_);
        }
    }
    void detect(const cv::Mat& image_in)
    {
        detector_ptr_->detect(image_in);
        if (display_)
            detector_ptr_->dispaly();
    }
private:
    ros::NodeHandlePtr nh_ptr_;
    ros::NodeHandlePtr pnh_priv_;
    int camera_id_;
    std::string ns_;
    bool display_;
    double threshold_;
    T::Ptr detector_ptr_;

    std::vector<std::string> detectors_;
    

}; // class FHOGObjectDetectorClass

typedef boost::shared_ptr<ObjectDetectorClass<FHOGObjectDetector>> FHOGObjectDetectorPtr;
} // namespace detector
} // namespace hop_detection
