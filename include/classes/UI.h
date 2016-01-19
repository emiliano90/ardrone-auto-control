#ifndef UI_H_
#define UI_H_

//	# own classes includes
#include "include/classes/Video.h"
#include "include/classes/Robot.h"
#include "include/classes/Environment.h"
#include "include/util/Util.h"
#include "include/core/camera.h"

//	# opencv includes
#include <core/core.hpp>
#include <highgui/highgui.hpp>
//otros
#include "math.h"

namespace ar
{

class UI
{
	private:
		UI(){};
	public:
		static void drawRobot(cv::Mat &image, const Data &data);
		static void drawWindow(std::string window, cv::Mat &image);
		static void drawText(cv::Mat &image, string text, ar::Point pos, cv::Scalar color);
		static void drawDestinations(cv::Mat &image, std::vector<cv::Point2i > destinations, const Data &data);
		static void drawGraphics(cv::Mat &image, const Data &data);
		static void drawCircle(cv::Mat &image, ar::Point point, int radius, const cv::Scalar &color);
		static void drawNextDestination(cv::Mat &image, std::vector<cv::Point2i > destinations, int destination, const Data &data);
		static void drawMaxView(cv::Mat &image, const Data &data);
		static void destroy();

		virtual ~UI();
};

} /* namespace ar */
#endif /* UI_H_ */
