#ifndef COLOR_DETECTION_H
#define COLOR_DETECTION_H

#include "hop_detection/armor_detection_base.h"
#include "common/timer.h"
#include "common/log.h"

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/CameraInfo.h>

#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <hop_detection/HopDetectionConfig.h>

#include <ros/ros.h>

namespace hop_detection
{
namespace armor_detectors
{
/**
 *  This class describes the armor information, including maximum bounding box, vertex, standard deviation.
 */
class ArmorInfo {
public:
    ArmorInfo(cv::RotatedRect armor_rect, std::vector<cv::Point2f> armor_vertex, float armor_stddev = 0.0) {
        rect = armor_rect;
        vertex = armor_vertex;
        stddev = armor_stddev;
    }
public:
    cv::RotatedRect rect;
    std::vector<cv::Point2f> vertex;
    float stddev;
};

enum EnemyColor
{
    BLUE = 0,
    RED = 1
};

struct ColorDetectionParams
{
	double light_max_aspect_ratio;
	double light_min_aspect_ratio;
	double light_min_area;
	int light_max_area;
	double light_max_angle;
	double light_max_angle_diff;
	double light_length_ratio;
	
	double armor_max_angle;
	int armor_min_area;
	double armor_max_aspect_ratio;
	double armor_max_pixel_val;
	double armor_max_stddev;
	
	double armor_blank_max_angle;
	int armor_blank_min_area;
	double blank_light_length_ratio;
	
};

class ColorDetection
{
public:
    typedef boost::shared_ptr<ColorDetection> Ptr;
    ColorDetection(ros::NodeHandle &nh, ros::NodeHandle &pnh, int cam_id);
    ~ColorDetection(){};
    void onInit();
    void calcTargetInfo(const ArmorInfo & armor, double & x_ratio, double& y_ratio, double &area);
    ErrorInfo detectTarget(double & x_ratio, double& y_ratio, double &area);
    void setDebug(bool debug)
    {
        debug_ = debug;
    }

    void setDisplay(bool display)
    {
        display_ = display;
    }

    void setCameraInfo(sensor_msgs::CameraInfo &cam_info)
    {
        camera_info_ = cam_info;
    }

    bool updateFrame(const cv::Mat& image_in);

    bool updateDepth(const cv::Mat& depth_in);
    
    ErrorInfo detectArmor(double &distance, double &pitch, double &yaw);
    
    
    cv::Mat extractRed(const cv::Mat &src);

    void detectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights, int light_color = EnemyColor::RED);
    /**
     * @brief Filtering the detected lights.
     * @param lights Filtered lights
     */
    void filterLights(std::vector<cv::RotatedRect> &lights);
    /**
     * @brief Finding possible armors.
     * @param lights Take lights information as input.
     * @param armors Possible armors
     */
    void possibleArmors(const std::vector<cv::RotatedRect> &lights, std::vector<ArmorInfo> &armors);
    /**
     * @brief Filtering Detected armors by standard deviation and non-maximum suppression(nms).
     * @param armors Result armors
     */
    void filterArmors(std::vector<ArmorInfo> &armors);
    /**
     * @brief Slecting final armor as the target armor which we will be shot.
     * @param Input armors
     */
    ArmorInfo selectFinalArmor(std::vector<ArmorInfo> &armors);
    void calcControlInfo(const ArmorInfo & armor,
                        double &distance,
                        double &pitch,
                        double &yaw,
                        double bullet_speed);
    /**
     * @brief Using two lights(left light and right light) to calculate four points of armor.
     * @param armor_points Out put
     * @param left_light Rotated rect of left light
     * @param right_light Rotated rectangles of right light
     */
    void calcArmorInfo(std::vector<cv::Point2f> &armor_points, cv::RotatedRect left_light, cv::RotatedRect right_light);
    /**
     * @brief Calculating the coordinates of the armor by its width and height.
     * @param width Armor width
     * @param height Armor height
     */
    void solveArmorCoordinate(const float width, const float height);

    void drawRotatedRect(const cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness) 
    {
        cv::Point2f vertex[4];

        rect.points(vertex);
        for (int i = 0; i < 4; i++)
        cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);
    }

private:
    int camera_id_;
    std::vector<float> camera_matrix_;
    std::vector<float> camera_distortion_;
    dynamic_reconfigure::Server<hop_detection::HopDetectionConfig> dyn_cfg_server_;
    dynamic_reconfigure::Server<hop_detection::HopDetectionConfig>::CallbackType dyn_cfg_f_;
    void dynCfgCallback(hop_detection::HopDetectionConfig& config, uint32_t level);

    //DetectionResult result_;
    ErrorCode error_code_;
    ErrorInfo error_info_;

    sensor_msgs::CameraInfo camera_info_;

    bool debug_, display_;
    //int enemy_color_;
    bool has_depth_;
    bool depth_updated_;

    cv::Mat src_img_;
    cv::Mat hsv_img_;
    cv::Mat gray_img_;
    cv::Mat depth_img_;
    unsigned int enemy_color_;

    cv::Mat show_lights_before_filter_;
    cv::Mat show_lights_after_filter_;
    cv::Mat show_armors_befor_filter_;
    cv::Mat show_armors_after_filter_;

    //armor info
    std::vector<cv::Point3f> armor_points_;

    ColorDetectionParams params_;

    std::string name_;
    ros::NodeHandlePtr nh_ptr_;
    ros::NodeHandlePtr pnh_ptr_;

    std::vector<cv::Scalar> red_range_;

};
} // namespace detector
} // namespace hop_detection


#endif
