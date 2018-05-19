#include "object_detection/tracking/multi_track.h"
#include "object_detection/tracking/util.h"

namespace hop_detection
{
namespace object_detectors
{
/* ==========================================================================
Class : Util
Many useful but not fundamental functions are implemented in this class.
All functions are static functions so don't need to make class Util object
to use these functions.
========================================================================== */
// class Util
// {
// public:

//     /* --------------------------------------------
//     Function : cvtRectToRect
//     Convert cv::Rect to dlib::drectangle
//     ----------------------------------------------- */
//     static dlib::drectangle cvtRectToDrect(cv::Rect _rect)
//     {
//         return dlib::drectangle(_rect.tl().x, _rect.tl().y, _rect.br().x - 1, _rect.br().y - 1);
//     }


//     /* -------------------------------------------------
//     Function : cvtMatToArray2d
//     convert cv::Mat to dlib::array2d<unsigned char>
//     ------------------------------------------------- */
//     static dlib::array2d<unsigned char> cvtMatToArray2d(cv::Mat _mat) // cv::Mat, not cv::Mat&. Make sure use copy of image, not the original one when converting to grayscale
//     {

//         //Don't need to use color image in HOG-feature-based tracker
//         //Convert color image to grayscale
//         if (_mat.channels() == 3)
//             cv::cvtColor(_mat, _mat, cv::COLOR_BGR2GRAY);

//         //Convert opencv 'MAT' to dlib 'array2d<unsigned char>'
//         //dlib::cv_image<dlib::bgr_pixel> dlib_img
//         dlib::array2d<unsigned char> dlib_img;
//         dlib::assign_image(dlib_img, dlib::cv_image<unsigned char>(_mat));

//         return dlib_img;
//     }


//     /* -----------------------------------------------------------------
//     Function : setRectToImage
//     Put all tracking results(new rectangle) on the frame image
//     Parameter _rects is stl container(such as vector..) filled with
//     cv::Rect
//     ----------------------------------------------------------------- */
//     template <typename Container>
//     static void setRectToImage(cv::Mat& _mat_img, Container _rects)
//     {
//         std::for_each(_rects.begin(), _rects.end(), [&_mat_img](cv::Rect rect) {
//             cv::rectangle(_mat_img, rect, cv::Scalar(0, 0, 255));
//         });
//     }
// };

/* ---------------------------------------------------------------------------------
Function : startSingleTracking
Initialize dlib::correlation_tracker tracker using dlib::start_track function
---------------------------------------------------------------------------------*/
bool SingleTracker::initTracking(cv::Mat& _mat_img, cv::Rect _init_rect)
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
        //dlib::array2d<unsigned char> dlib_frame = Util::cvtMatToArray2d(_mat_img);
        dlib::array2d<unsigned char> dlib_frame(_mat_img)
        // Convert SingleTracker::rect to dlib::drectangle
        dlib::drectangle dlib_rect = Utils::cvtRectToDrect(this->getRect());

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
bool SingleTracker::update(cv::Mat& _mat_img)
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
            if (this->cv_tracker.update(_mat_img, updated_rect))
                confidence = 1;
            else
                confidence = 0;

        }
        else
        {
            // Convert _mat_img to dlib::array2d<unsigned char>
            dlib::array2d<unsigned char> dlib_img = Util::cvtMatToArray2d(_mat_img);

            // Track using dlib::update function
            confidence = this->tracker.update(dlib_img);

            // New position of the target
            dlib::drectangle updated_rect = this->tracker.get_position();
        }
        

        // Update variables(center, rect, confidence)
        this->setCenter(updated_rect);
        this->setRect(updated_rect);
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

TrackingSystem::TrackingSystem(int _target_num): target_num(_targte_num)
{
    for (int i = 0; i < target_num; i++)
    {
        boost::make_shared<SingleTracker> new_tracker();
        this->trackers.push_back(new_tracker);
    }
}

void TrackingSystem::update(const cv::Mat& img, std::vector<dlib::rectangle> detections)
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
            else if (tracker->getOutOfFrameCounter > 50)
            {
                tracker->disable();
            }
        }
    }
    else
    {
        if (Utils::computeIous(this->trackers[0]->getDlibRect(), this->trackers[1]->getDlibRect()) > 0.8)
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
                    tracker->initTracking(img, Utils::dRectToCvRect(*comp_result));
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
                    tracker->initTracking(img, *detections.begin());
                    detections.erase(detections.begin());
                }
            }
        }
    }
    return tracking_results;
}



} // namespace detector
} // namespace hop_detection