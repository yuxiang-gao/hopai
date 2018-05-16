#include <ros/ros.h>
#include "fhog_object_detector/fhog_object_detector.h"

namespace hop_detection
{
namespace object_detectors
{
template<class T>
class ObjectDetectorClass
{
public:
    ObjectDetectorClass(std::vector<std::string>& detectors, ros::NodeHandle &nh, ros::NodeHandle &nh_priv, std::string ns):
        nh_ptr_(boost::make_shared<ros::NodeHandle> (nh),
        nh_priv_ptr_(boost::make_shared<ros::NodeHandle> (nh_priv),
    {
        nh_priv_ptr_->getParam(ns + "/display", display_);
        nh_priv_ptr_->getParam(ns + "/threshold", display_);
        detector_ptr_ = boost::make_shared<T> (detectors, display_);
        if (threshold_ != 0.0)
        {
            detector_ptr_->setThreshold(threshold_);
        }

    }
private:
    boost::shared_ptr<ros::NodeHandle> nh_ptr_;
    boost::shared_ptr<ros::NodeHandle> nh_priv_ptr_;
    bool display_;
    double threshold_;
    boost::shared_ptr<T> detector_ptr_;
    

}; // class FHOGObjectDetectorClass
} // namespace detector
} // namespace hop_detection
