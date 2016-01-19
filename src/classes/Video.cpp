#include "include/classes/Video.h"
#include <stdio.h>
#include <math.h>

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
bool start = true;
ar::Point Video::tracking(Mat img, const HSVColor hsv)
{
	ar::Point pos;

	if(start)
	{
		//inizializzazione
		//si potrebbe provare a dichiarare le variabili in video_stage.h
		//e a definirle quando vengono fatti partire i thread
		imageSize = cvSize(img.cols,img.rows);

		frame_th = cv::Mat(imageSize, CV_8UC1);
		frame_th2 = cv::Mat(imageSize, CV_8UC1);
		img_2 = cv::Mat(imageSize, CV_8UC3);
		blank = cv::Mat(imageSize, CV_8UC1);
		//  cvSet(blank, cvScalar(0,0,0,0),blank);
		converted= cv::Mat(imageSize, CV_8UC3);
		dst= cv::Mat(imageSize, CV_8UC3);
		currframe= cv::Mat(imageSize, CV_8UC3);

		/*creazione dell'elemento strutturante*/
		morphKernel = cv::getStructuringElement( cv::MORPH_RECT, cv::Size(5, 5), cv::Point(1,1));
		start = false;
	}


	//cambio dello spazio di colore, più semplice tarare l'algoritmo di visione
	//nello spazio HSV
	cv::cvtColor(img, img_2, CV_BGR2HSV);
	cv::inRange(img_2,hsv.min,hsv.max ,frame_th);
	//applicazione dell'elemento strutturante a frame_th, con operazione di apertura
	morphologyEx(frame_th, frame_th2, CV_MOP_OPEN,morphKernel);

	namedWindow("morphology", WINDOW_AUTOSIZE);
	imshow("morphology", frame_th2);

	Moments moments_1;
	Moments moments_2;

	double moment10;
	double moment01;
	double area,area2, r;

	moments_1 = moments(frame_th2, 1);

	moment10 = moments_1.m10;
	moment01 = moments_1.m01;
	area2 = moments_1.m00;


	r = sqrt(area2/M_PI);

	if(area2 > 3) //oggetto riconosciuto e abbastanza grande da non essere confuso con altri oggetti di colore simile
	{
		pos.x = moment10/area2;
		pos.y = moment01/area2;


	}
	else 	//OGGETTO NON RICONOSCIUTO
	{
		pos.x = -1;
		pos.y = -1;
	}
	return pos;
}



}
