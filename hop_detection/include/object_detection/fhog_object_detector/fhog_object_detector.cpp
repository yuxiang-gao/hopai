#include "fhog_object_detector/fhog_object_detector.h"

namespace hop_detection
{
namespace object_detectors
{
FHOGObjectDetector::FHOGObjectDetector(std::vector<std::string>& detectors_path, std:string display):
    display_(display),
    threshold_(0.0)
{
    // clear and populate the detectors
    //detectors_.reserve(detectors_path.size());
    for (auto const& d: detectors_path)
    {
        dlib::object_detector<image_scanner_type> detector;
        dlib::deserialize(d) >> detector;
        // detector = dlin::threshold_filter_singular_values(detector, 0.1);
        detectors_.push_back(detector);
    }
}

std::vector<dlib::rectangle> FHOGObjectDetector::detect(const cv::Mat& image_in)
{
    std::vector<dlib::rectangle> detections;
    image_ = image_in;
    detections = dlib::evaluate_detectors(detectors_, *image_, threshold_);
    filterDetections(detections);
    detections_ = detections;
    return detections;
}

void FHOGObjectDetector::filterDetections(std::vector<dlib::rectangle>& detections)
{
    if (detections.size() > 1)
    {
        for (auto iter = detections.begin() + 1; iter != detections.end(); ) {
        {
            if (Utils::computeIOU(detections_[0], *iter) > 0.8) // overlapped
            {
                detections.erase(iter);
            }
            else
                ++iter;
        }
    } 
}

void FHOGObjectDetector::display(std::vector<dlib::rectangle> detections)
{
	if (dispaly_)
	{
		// Display it all on the screen
		win_.clear_overlay();
		win_.set_image(*image_ptr_);
		win_.add_overlay(detections, dlib::rgb_pixel(255,0,0));
	}
}

} // namespace detector
} // namespace hop_detection
