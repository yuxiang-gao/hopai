#ifndef MULTI_TRACK_H
#define MULTI_TRACK_H

#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/dir_nav.h>
#include <dlib/opencv.h>

#include <boost/shared_ptr.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

namespace hop_detection
{
namespace object_detectors
{
#define FAIL   -1
#define SUCCESS 1
#define FALSE   0
#define TRUE    1
/* ==========================================================================
Class : SingleTracker
This class is aim to track 'One' target for running time.
'One' SingleTracker object is assigned to 'One' person(or any other object).
In other words, if you are trying to track 'Three' people,
then need to have 'Three' SingleTracker object.
========================================================================== */
class SingleTracker
{
private:
    int         out_of_frame_counter;
    //int         target_id;			// Unique Number for target
    double      confidence;			// Confidence of tracker
    cv::Rect    rect;				// Initial Rectangle for target
    cv::Point   center;				// Current center point of target
    bool        is_tracking_started;// Is tracking started or not? (Is initializing done or not?)
    //cv::Scalar  color;				// Box color
    bool        use_opencv;
    bool        is_inside_frame;

    cv::Ptr<cv::Tracker> cv_tracker;
    dlib::correlation_tracker tracker;  // Correlation tracker

public:
    typedef boost::shared_ptr<SingleTracker> Ptr;
    /* Member Initializer & Constructor*/
    SingleTracker()//, cv::Scalar _color)
        : target_id(0), 
        confidence(0), 
        is_tracking_started(false), 
        use_opencv(false)
    {
        if (use_opencv)
            cv_tracker = cv::TrackerKCF::create();
    }

    /* Get Function */
    int			getOutOfFrameCounter() { return this->out_of_frame_counter; }
    bool        getIsInsideFrame() { return this->is_inside_frame; }
    int			getTargetID() { return this->target_id; }
    cv::Rect	getRect() { return this->rect; }
    dlib::rectangle getDlibRect() { return Util::cvRectToDlibRect(this->rect); }
    cv::Point	getCenter() { return this->center; }
    double		getConfidence() { return this->confidence; }
    bool		getIsTrackingStarted() { return this->is_tracking_started; }
    //cv::Scalar	getColor() { return this->color; }

    /* Set Function */
    void setTargetId(int _target_id) { this->target_id = _target_id; }
    void setRect(cv::Rect _rect) { this->rect = _rect; }
    void setRect(cv::Rect2d _rect) { this->rect = _rect; }
    void setRect(dlib::drectangle _drect) { this->rect = cv::Rect(_drect.tl_corner().x(), _drect.tl_corner().y(), _drect.width(), _drect.height()); }
    void setCenter(cv::Point _center) { this->center = _center; }
    void setCenter(cv::Rect _rect) { this->center = cv::Point(_rect.x + (_rect.width) / 2, _rect.y + (_rect.height) / 2); }
    void setCenter(cv::Rect2d _rect) { this->center = cv::Point(_rect.x + (_rect.width) / 2, _rect.y + (_rect.height) / 2); }
    void setCenter(dlib::drectangle _drect) { this->center = cv::Point(_drect.tl_corner().x() + (_drect.width() / 2), _drect.tl_corner().y() + (_drect.height() / 2)); }
    void setConfidence(double _confidence) { this->confidence = _confidence; }
    void setIsTrackingStarted(bool _b) { this->is_tracking_started = _b; }
    void disable() 
    { 
        this->setIsTrackingStarted(false); 
        this->out_of_frame_counter = 0;
    }
    // void enable()
    // { 
    //     this->setIsTrackingStarted(true); 
    //     this->out_of_frame_counter = 0;
    // }
    //void setColor(cv::Scalar _color) { this->color = _color; }

    /* Core Function */
    // Initialize
    bool initTracking(cv::Mat& _mat_img, cv::Rect _init_rect);

    // Do tracking
    bool update(cv::Mat& _mat_img);

    // Check the target is inside of the frame
    bool isTargetInsideFrame(int _frame_width, int _frame_height);
};


class TrackingSystem
{
public:
    typedef boost::shared_ptr<TrackingSystem> Ptr;
    TrackingSystem(int _target_num);
    void update(const cv::Mat& img, std::vector<dlib::rectangle> detections);

private:
    int            target_num;
    int            frame_width;
    int            frame_height;
    cv::Mat        current_frame;
    std::vector<SingleTracker::Ptr> trackers;
    // TrackerManager manager;

};


#endif