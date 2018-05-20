#ifndef UTILS_H
#define UTILS_H

#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/dir_nav.h>
#include <dlib/opencv.h>

#include <opencv2/opencv.hpp>

namespace hop_detection
{
namespace object_detectors
{
class Utils{
public:
	static double computeIOU(dlib::rectangle rect1, dlib::rectangle rect2)
	{
		return rect1.intersect(rect2).area() / (rect1.area() + rect2.area() - rect1.intersect(rect2).area());
	}

    static cv::Rect dRectToCvRect(dlib::rectangle r)
    {
        return cv::Rect(cv::Point2i(r.left(), r.top()), cv::Point2i(r.right() + 1, r.bottom() + 1));
    }
    /* --------------------------------------------
	Function : cvtRectToRect
	Convert cv::Rect to dlib::drectangle
	----------------------------------------------- */
	static dlib::drectangle cvRectToDrect(cv::Rect _rect)
	{
		return dlib::drectangle(_rect.tl().x, _rect.tl().y, _rect.br().x - 1, _rect.br().y - 1);
	}

	static dlib::rectangle cvRectToDlibRect(cv::Rect _rect)
	{
		return dlib::rectangle(_rect.tl().x, _rect.tl().y, _rect.br().x - 1, _rect.br().y - 1);
	}


	/* -------------------------------------------------
	Function : cvtMatToArray2d
	convert cv::Mat to dlib::array2d<unsigned char>
	------------------------------------------------- */
	static dlib::array2d<unsigned char> cvMatToArray2d(const cv::Mat& src_img) // cv::Mat, not cv::Mat&. Make sure use copy of image, not the original one when converting to grayscale
	{
		//Don't need to use color image in HOG-feature-based tracker
		//Convert color image to grayscale
        cv::Mat gray_img;
        if (src_img.channels() == 3)
        {
            cv::cvtColor(src_img, gray_img, cv::COLOR_RGB2GRAY);
        }
        else
            gray_img = src_img;
        //cv::Mat gray_img;
        //cv::cvtColor(src_img, gray_img, cv::COLOR_RGB2GRAY);

		//Convert opencv 'MAT' to dlib 'array2d<unsigned char>'
		dlib::array2d<unsigned char> dlib_img;
		dlib::assign_image(dlib_img, dlib::cv_image<unsigned char>(gray_img));

		return dlib_img;
	}


	/* -----------------------------------------------------------------
	Function : setRectToImage
	Put all tracking results(new rectangle) on the frame image
	Parameter _rects is stl container(such as vector..) filled with
	cv::Rect
	----------------------------------------------------------------- */
	template <typename Container>
	static void setRectToImage(cv::Mat& _mat_img, Container _rects)
	{
		std::for_each(_rects.begin(), _rects.end(), [&_mat_img](cv::Rect rect) {
			cv::rectangle(_mat_img, rect, cv::Scalar(0, 0, 255));
		});
	}
};
} // namespace detector
} // namespace hop_detection
#endif
