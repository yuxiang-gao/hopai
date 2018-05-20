#ifndef OBJECT_DETECTOR_BASE_H
#define OBJECT_DETECTOR_BASE_H

#include "common/error_code.h"
#include <boost/shared_ptr.hpp>

#include <opencv2/opencv.hpp>

#include <vector>
#include <iostream>
#include <fstream>


namespace hop_detection
{
namespace armor_detectors
{
using hop_detection::common::ErrorCode;
using hop_detection::common::ErrorInfo;

struct DetectionResult
{
    double distance;
    double pitch;
    double yaw;
    double dist_belief;
    ErrorInfo error_info;
};

class ArmorDetectionBase
{
public:
    typedef boost::shared_ptr<ArmorDetectionBase> Ptr;
    ArmorDetectionBase() {};
    virtual bool updateFrame(const cv::Mat& image_in);
	virtual ErrorInfo detectArmor(double &distance, double &pitch, double &yaw) {};
	virtual void display() {};
	virtual ~ArmorDetectionBase() = default;;

}; // class ArmorDetector
} // namespace detector
} // namespace hop_detection

#endif
