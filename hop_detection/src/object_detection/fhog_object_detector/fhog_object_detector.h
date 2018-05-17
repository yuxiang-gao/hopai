#ifndef FHOG_OBJECT_DETECTOR_H
#define FHOG_OBJECT_DETECTOR_H

#include <dlib/opencv.h>
#include <dlib/svm_threaded.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_processing.h>
#include <dlib/data_io.h>

#include <boost/shared_ptr.hpp>

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
	void detect(const cv::Mat& image_in);
	void display();
    void setThreshold(double threshold)
    {
        threshold_ = 0.0;
    }
	~FHOGObjectDetector() {};
private:
    typedef dlib::scan_fhog_pyramid<dlib::pyramid_down<6> > image_scanner_type;
    typedef dlib::cv_image<dlib::bgr_pixel> cv_image_type;
    
	std::vector<dlib::object_detector<image_scanner_type> > detectors_; // list of detectors
	std::vector<dlib::rectangle> detections_; //detect results
	boost::shared_ptr<cv_image_type> image_ptr_;
    double threshold_;
    bool dispaly_;
    dlib::image_window win_;
}; // class FHOGObjectDetector
} // namespace detector
} // namespace hop_detection

#endif
