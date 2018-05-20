#ifndef FHOG_OBJECT_DETECTOR_H
#define FHOG_OBJECT_DETECTOR_H

#include "object_detection/util.h"

#include <dlib/opencv.h>
#include <dlib/svm_threaded.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_processing.h>
#include <dlib/data_io.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <vector>
#include <iostream>
#include <fstream>


namespace hop_detection
{
namespace object_detectors
{
// TODO: scan_image_boxes
class FHOGObjectDetector
{
public:
    typedef boost::shared_ptr<FHOGObjectDetector> Ptr;
    
    FHOGObjectDetector(std::vector<std::string>& detectors, bool display);

    std::vector<dlib::rectangle> detect(const cv::Mat& image_in);

    void filterDetections(std::vector<dlib::rectangle>& detections);
    
    void display(std::vector<dlib::rectangle> & detections);

    std::vector<dlib::rectangle> getDetections() { return  detections_; }
    
    void setThreshold(double threshold)
    {
        threshold_ = threshold;
    }

    ~FHOGObjectDetector() {};
private:
    typedef dlib::scan_fhog_pyramid<dlib::pyramid_down<6> > image_scanner_type;
    typedef dlib::cv_image<dlib::bgr_pixel> cv_image_type;
    
    std::vector<dlib::object_detector<image_scanner_type> > detectors_; // list of detectors
    std::vector<dlib::rectangle> detections_; //detect results
    //boost::shared_ptr<cv_image_type> image_ptr_;
    cv_image_type image_;
    double threshold_;
    bool display_;
    dlib::image_window win_;
}; // class FHOGObjectDetector
} // namespace detector
} // namespace hop_detection

#endif
