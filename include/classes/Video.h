#ifndef VIDEO_H_
#define VIDEO_H_

#include "include/util/structures.h"

#include <core/core.hpp>
#include <highgui/highgui.hpp>

namespace ar
{

class Video
{
	private:
		cv::VideoCapture cap;
		double fps;
		cv::Size imageSize;

	public:
		Video(int camera);
		void grabVideoAndData(string path, string ext, string buffer,
				cv::VideoWriter &writer, const cv::Mat &image);
		cv::Mat getFrame();
		bool isCircle(const cv::Mat &image, int x, int y, int width,
				int height);
		bool isSimilarColorHSV3(const cv::Scalar color, const HSVColor hsv);
		bool isSimilarColorHSV2(const cv::Scalar color, const HSVColor hsv);
		bool isSimilarColorHSV(const cv::Scalar color,
				const ar::HSVColor hsv);
		cv::Size getImageSize();
		void transformToBlackAndWhite2(cv::Mat &image, const HSVColor color);
		std::vector<ar::Point> transformToBlackAndWihte(cv::Mat &image, const HSVColor color);
		std::vector<ar::Point>* transformToBlackAndWihte(cv::Mat &image, const HSVColor color[], int lenght);


		virtual ~Video();
};
}
#endif /* VIDEO_H_ */
