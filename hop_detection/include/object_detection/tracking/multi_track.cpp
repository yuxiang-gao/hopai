#include "object_detection/tracking/multi_track.h"


namespace hop_detection
{
namespace object_detectors
{
/* ---------------------------------------------------------------------------------
Function : startSingleTracking
Initialize dlib::correlation_tracker tracker using dlib::start_track function
---------------------------------------------------------------------------------*/
bool SingleTracker::initTracking(const cv::Mat& _mat_img, cv::Rect &_init_rect)
{
    // Exception
    if (_mat_img.empty())
    {
        std::cout << "====================== Error Occured! =======================" << std::endl;
        std::cout << "Function : int SingleTracker::startSingleTracking" << std::endl;
        std::cout << "Parameter cv::Mat& _mat_img is empty image!" << std::endl;
        std::cout << "=============================================================" << std::endl;

        return false;
    }

    this->setRect(_init_rect);
    this->setCenter(_init_rect);

    if (this->use_opencv)
    {
        this->cv_tracker->init(_mat_img, this->getRect());
    }
    else
    {
        // Convert _mat_img to dlib::array2d<unsigned char>
        //dlib::array2d<unsigned char> dlib_frame = Util::cvMatToArray2d(_mat_img);
        //dlib::array2d<unsigned char> 
        dlib::cv_image<unsigned char> dlib_frame(_mat_img);
        // Convert SingleTracker::rect to dlib::drectangle
        dlib::drectangle dlib_rect = Utils::cvRectToDrect(this->getRect());

        // Initialize SingleTracker::tracker
        this->tracker.start_track(dlib_frame, dlib_rect);
    }
    this->setIsTrackingStarted(true);
    this->out_of_frame_counter = 0;

    return true;
}

/*---------------------------------------------------------------------------------
Function : isTargetInsideFrame
Check the target is inside the frame
If the target is going out of the frame, need to SingleTracker stop that target.
---------------------------------------------------------------------------------*/
bool SingleTracker::isTargetInsideFrame(int _frame_width, int _frame_height)
{
    int cur_x = this->getCenter().x;
    int cur_y = this->getCenter().y;

    bool is_x_inside = ((0 <= cur_x) && (cur_x < _frame_width));
    bool is_y_inside = ((0 <= cur_y) && (cur_y < _frame_height));

    if (is_x_inside && is_y_inside)
        return true;
    else
        return false;
}

/* ---------------------------------------------------------------------------------
Function : doSingleTracking
Track 'one' target specified by SingleTracker::rect in a frame.
(It means that you need to call doSingleTracking once per a frame)
SingleTracker::rect is initialized to the target position in the constructor of SingleTracker
Using correlation_tracker in dlib, start tracking 'one' target
--------------------------------------------------------------------------------- */
bool SingleTracker::update(const cv::Mat& _mat_img)
{
    if (this->is_tracking_started)
    {
            //Exception
        if (_mat_img.empty())
        {
            std::cout << "====================== Error Occured! ======================= " << std::endl;
            std::cout << "Function : int SingleTracker::doSingleTracking" << std::endl;
            std::cout << "Parameter cv::Mat& _mat_img is empty image!" << std::endl;
            std::cout << "=============================================================" << std::endl;

            return false;
        }

        int width = _mat_img.cols;
        int height = _mat_img.rows;
        double confidence;

        if(this->use_opencv)
        {
            cv::Rect2d updated_rect;
            if (this->cv_tracker->update(_mat_img, updated_rect))
                confidence = 1;
            else
                confidence = 0;

            this->setCenter(updated_rect);
            this->setRect(updated_rect);
        }
        else
        {
            // Convert _mat_img to dlib::array2d<unsigned char>
            dlib::array2d<unsigned char> dlib_img = Utils::cvMatToArray2d(_mat_img);

            // Track using dlib::update function
            confidence = this->tracker.update(dlib_img);

            // New position of the target
            dlib::drectangle updated_rect = this->tracker.get_position();

            this->setCenter(updated_rect);
            this->setRect(updated_rect);
        }
        

        // Update variables(center, rect, confidence)
        this->setConfidence(confidence);
        this->is_inside_frame = this->isTargetInsideFrame(width, height);
        if (!this->is_inside_frame) 
            this->out_of_frame_counter++;
        else 
            this->out_of_frame_counter = 0;
        return true;
    }
    else
        return false;
    
}

TrackingSystem::TrackingSystem(int _target_num): target_num(_target_num)
{
    for (int i = 0; i < target_num; i++)
    {
        this->trackers.push_back(boost::make_shared<SingleTracker>());
    }
}

std::vector<dlib::rectangle> TrackingSystem::update(const cv::Mat& img, std::vector<dlib::rectangle> detections)
{
    std::vector<dlib::rectangle> tracking_results;
    if (detections.size() == 0)
    {
        for (auto& tracker: this->trackers)
        {
            if (tracker->getIsTrackingStarted() && tracker->update(img))
            {
                tracking_results.push_back(tracker->getDlibRect());
            }
            else if (tracker->getOutOfFrameCounter() > 50)
            {
                tracker->disable();
            }
        }
    }
    else
    {
        if (Utils::computeIOU(this->trackers[0]->getDlibRect(), this->trackers[1]->getDlibRect()) > 0.8)
        {
            this->trackers[1]->disable();
        }
        for (auto& tracker: this->trackers)
        {
            dlib::rectangle result;
            if (tracker->getIsTrackingStarted() && tracker->update(img))
            {
                result = tracker->getDlibRect();
                std::vector<dlib::rectangle>::iterator comp_result;
                comp_result = std::max_element(detections.begin(), detections.end(), [&result]
                                    (const dlib::rectangle &rect1, const dlib::rectangle &rect2)
                                    {
                                        return Utils::computeIOU(result, rect1) < Utils::computeIOU(result, rect2);
                                    });
                //int max_index = std::distance(detections.begin(), comp_result);
                if (Utils::computeIOU(*comp_result, result) > 0.5)
                {
                    cv::Rect result_compare = Utils::dRectToCvRect(*comp_result);
                    tracker->initTracking(img, result_compare);
                    detections.erase(comp_result);
                }
                tracking_results.push_back(tracker->getDlibRect());
            }
        }
        if (detections.size() != 0)
        {
            for (auto& tracker: this->trackers)
            {
                if (!tracker->getIsTrackingStarted())
                {
                    cv::Rect detections_cv_rect = Utils::dRectToCvRect(*detections.begin());
                    tracker->initTracking(img, detections_cv_rect);
                    detections.erase(detections.begin());
                }
            }
        }
    }
    return tracking_results;
}



} // namespace detector
} // namespace hop_detection
