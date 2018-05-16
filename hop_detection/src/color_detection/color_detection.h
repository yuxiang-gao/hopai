#ifdef COLOR_DETECTION_H
#define COLOR_DETECTION_H

#include <vector>

#include "opencv2/opencv.hpp"

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
#endif
