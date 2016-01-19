#ifndef VIDEO_H_
#define VIDEO_H_

#include "include/util/structures.h"

#include <core/core.hpp>
#include <highgui/highgui.hpp>

#include <imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

namespace ar
{

class Video
{
	private:
		cv::VideoCapture cap;
		double fps;
		cv::Size imageSize;

		Mat img_2, frame_th,frame_th2,blank,currframe, dst, converted;
		cv::Mat morphKernel;

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
		ar::Point tracking(Mat img, const HSVColor hsv);

		virtual ~Video();
};
}
#endif /* VIDEO_H_ */
