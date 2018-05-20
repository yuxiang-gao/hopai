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
    ObjectDetectorClass(ros::NodeHandle *nh, ros::NodeHandle *pnh, int cam_id);

    std::vector<cv::Rect> detect(const cv::Mat& image_in);
    ~ObjectDetectorClass()
    {
        detector_ptr_.reset();
        tracking_ptr_.reset();
    }
private:
    ros::NodeHandlePtr nh_ptr_;
    ros::NodeHandlePtr pnh_ptr_;
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
