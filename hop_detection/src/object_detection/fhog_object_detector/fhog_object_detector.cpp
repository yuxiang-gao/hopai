#include "fhog_object_detector/fhog_object_detector.h"

namespace hop_detection
{
namespace object_detectors
{
FHOGObjectDetector::FHOGObjectDetector(std::vector<std::string>& detectors_path, std:string display):
	display_(display)
{
	// clear and populate the detectors
	detectors_.clear();
	for (auto const& d: detectors_path)
	{
		dlib::object_detector<image_scanner_type> detector;
        dlib::deserialize(d) >> detector;
        // detector = dlin::threshold_filter_singular_values(detector, 0.1);
        detectors_.push_back(detector);
	}
}

void FHOGObjectDetector::detect(const cv::Mat& image_in)
{
	image_ptr_ = boost::make_shared<cv_image_type>(image_in)
	detections_ = dlib::evaluate_detectors(detectors_, *image_ptr_, threshold_);
}

void FHOGObjectDetector::display()
{
	if (dispaly_)
	{
		// Display it all on the screen
		win_.clear_overlay();
		win_.set_image(*image_ptr_);
		win_.add_overlay(detections_, dlib::rgb_pixel(255,0,0));
	}
}

} // namespace detector
} // namespace hop_detection
