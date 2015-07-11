#include "include/classes/Video.h"
#include <stdio.h>

using namespace std;
using namespace cv;

namespace ar
{

Video::Video(int camera) :
		cap(camera)
{
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	this->imageSize.height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	this->imageSize.width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	this->fps = cap.get(CV_CAP_PROP_FPS);
}

Video::~Video()
{
	this->cap.release();
}


void Video::grabVideoAndData(string path, string ext, string buffer,
		VideoWriter &writer, const Mat &image)
{
	if (!writer.isOpened())
	{
		string source = path;

		source.append(buffer);
		source.append("." + ext);
		// Open the output
		writer.open(source, CV_FOURCC('X', 'V', 'I', 'D'), 12,
				cv::Size(image.size().width, image.size().height), true);

		if (!writer.isOpened())
		{
			printf("Could not open the output video for write: %s", source.c_str());
		}
	}

	writer.write(image);
}
Mat Video::getFrame()
{
	Mat img;
	this->cap.read(img);
	return img;
}

bool Video::isCircle(const Mat &image, int x, int y, int width, int height)
{
	return false;
}

std::vector<ar::Point> Video::transformToBlackAndWihte(Mat &image, const HSVColor color)
{
	std::vector<ar::Point> puntos;
	// accept only char type matrices
	CV_Assert(image.depth() != sizeof(uchar));

	const int channels = image.channels();
	switch (channels)
	{
		case 3:
		{
			MatIterator_<Vec3b> it, end;
			for (it = image.begin<Vec3b>(), end = image.end<Vec3b>(); it != end;
					++it)
			{
				Scalar c((*it)[0], (*it)[1], (*it)[2]);
				uchar f = 255 * !this->isSimilarColorHSV(c, color); // va a ser 255 * 1 ó 255 * 0.

				(*it)[0] = f;
				(*it)[1] = f;
				(*it)[2] = f;
				if (f == 0)
					puntos.push_back(ar::Point(it.pos().x, it.pos().y));

			}
			break;
		}
	}
	return puntos;
}

bool Video::isSimilarColorHSV(const Scalar color, const HSVColor hsv)
{
	return (((color.val[0] < 9
			|| (color.val[0] > hsv.min.val[0] && color.val[0] < hsv.max.val[0]))
			&& color.val[1] > hsv.min.val[1] && color.val[1] < hsv.max.val[1]
			&& color.val[2] > hsv.min.val[2] && color.val[2] < hsv.max.val[2])
			|| ((color.val[0] < 9
					|| (color.val[0] > hsv.min.val[0]
							&& color.val[0] < hsv.max.val[0]))
					&& color.val[1] > hsv.min.val[1] + 100
					&& color.val[1] < hsv.max.val[1]
					&& color.val[2] > hsv.min.val[2] - 40
					&& color.val[2] < hsv.max.val[2]));
}

bool Video::isSimilarColorHSV2(const Scalar color, const HSVColor hsv)
{
	return ((color.val[0] >= hsv.min.val[0] && color.val[0] <= hsv.max.val[0]
			&& color.val[1] >= hsv.min.val[1] && color.val[1] <= hsv.max.val[1]
			&& color.val[2] >= hsv.min.val[2] && color.val[2] <= hsv.max.val[2])
			|| ( color.val[0] >= hsv.min.val[0] && color.val[0] <= hsv.max.val[0]
					&& color.val[1] >= hsv.min.val[1] + 100
					&& color.val[1] <= hsv.max.val[1]
					&& color.val[2] >= hsv.min.val[2] - 40
					&& color.val[2] <= hsv.max.val[2]) );
}
bool Video::isSimilarColorHSV3(const Scalar color, const HSVColor hsv)
{
	return ((color.val[0] >= hsv.min.val[0] && color.val[0] <= hsv.max.val[0]
			&& color.val[1] >= hsv.min.val[1] && color.val[1] <= hsv.max.val[1]
			&& color.val[2] >= hsv.min.val[2] && color.val[2] <= hsv.max.val[2]));
}

std::vector<ar::Point>* Video::transformToBlackAndWihte(Mat &image, const HSVColor color[], int lenght)
{
	std::vector<ar::Point> *puntos =  new std::vector<ar::Point>[lenght];
	// accept only char type matrices
	CV_Assert(image.depth() != sizeof(uchar));

	const int channels = image.channels();
	switch (channels)
	{
		case 3:
		{
			MatIterator_<Vec3b> it, end;
			for (it = image.begin<Vec3b>(), end = image.end<Vec3b>(); it != end;
					++it)
			{
				Scalar c((*it)[0], (*it)[1], (*it)[2]);
				char f = 255;
				for(int i = 0; i < lenght; i++)
				{
					if(this->isSimilarColorHSV3(c, color[i]))
					{
						ar::Point p (it.pos().x, it.pos().y);
						puntos[i].push_back(p);
						f = i * 100; // va a ser 255 * 1 ó 255 * 0.
					}
				}

				(*it)[0] = f;
				(*it)[1] = f;
				(*it)[2] = f;

			}
			break;
		}
	}
	return puntos;
}


void Video::transformToBlackAndWhite2(Mat &image, const HSVColor color)
{
	// accept only char type matrices
	CV_Assert(image.depth() != sizeof(uchar));

	const int channels = image.channels();
	switch (channels)
	{
		case 3:
		{
			MatIterator_<Vec3b> it, end;
			for (it = image.begin<Vec3b>(), end = image.end<Vec3b>(); it != end;
					++it)
			{
				Scalar c((*it)[0], (*it)[1], (*it)[2]);
				char f = 255;

				f = 255 * !this->isSimilarColorHSV3(c, color);


				(*it)[0] = f;
				(*it)[1] = f;
				(*it)[2] = f;

			}
			break;
		}
	}

}

cv::Size Video::getImageSize()
{
	return this->imageSize;
}

}
