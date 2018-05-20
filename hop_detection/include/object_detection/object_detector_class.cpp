#include <ros/ros.h>
#include <ros/package.h>
#include "object_detection/fhog_object_detector/fhog_object_detector.h"
#include "object_detection/tracking/multi_track.h"
#include "object_detection/object_detector_class.h"

namespace hop_detection
{
namespace object_detectors
{
ObjectDetectorClass::ObjectDetectorClass(ros::NodeHandle *nh, ros::NodeHandle *pnh, int cam_id):
    nh_ptr_(nh),
    pnh_ptr_(pnh),
    camera_id_(cam_id),
    ns_("enemy_detection/fhog_object_detector")

{
    std::vector<std::string> detectors;
        
    pnh_ptr_->getParam(ns_ + "/display", display_);
    pnh_ptr_->getParam(ns_ + "/detectors", detectors);
    pnh_ptr_->getParam(ns_ + "/threshold", threshold_ );

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

std::vector<cv::Rect> ObjectDetectorClass::detect(const cv::Mat& image_in)
{
    cv::Mat gray_image;
    cv::cvtColor(image_in, gray_image, cv::COLOR_BGR2GRAY);
    std::vector<dlib::rectangle> detections = tracking_ptr_->update(gray_image, detector_ptr_->detect(image_in));
    std::vector<cv::Rect> cv_detections;
    for (auto & d : detections)
    {
        cv_detections.push_back(Utils::dRectToCvRect(d));
    }
    if (display_)
        detector_ptr_->display(detections);
    return cv_detections;
}
} // namespace detector
} // namespace hop_detection
