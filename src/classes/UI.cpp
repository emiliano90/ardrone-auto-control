#include "include/classes/UI.h"

using namespace std;
using namespace cv;
using namespace ar;

namespace ar
{

void UI::drawWindow(string window, Mat &image)
{
	namedWindow(window, WINDOW_AUTOSIZE);
	imshow(window, image);
}

void UI::drawText(Mat &image, string text, ar::Point pos, cv::Scalar color)
{
	putText(image, text, cv::Point(pos.x, pos.y), FONT_HERSHEY_SIMPLEX, 0.4,
			color);
}

void UI::drawCircle(Mat &image, ar::Point point, int radius,
		const cv::Scalar &color)
{
	circle(image, cv::Point(point.x, point.y), radius, color, 2, 8, 0);
}

void UI::drawRobot(Mat &image, const Data &data)
{
	//paso la posicion de mt a px
	cv::Point posPx;
	posPx.x = (int)Util::mtToPx(data.tPos.x, data);
	posPx.y = (int)Util::mtToPx(data.tPos.y, data);

	cv::Point pt;
	cv::Point pt2;

	float ajusteSet = 200;
	float ajusteGet = 5;

	// ************ seteo los sets ************
	//pitch set
	//desde
	pt2.y = 0;
	pt2.x = 4;
	pt2 = Util::arToCv(Util::rotate(pt2, data.copterValues.yaw));

	pt2.x += posPx.x;
	pt2.y += posPx.y;

	//hasta
	pt.y = data.copterSets.pitch * ajusteSet;
	pt.x = 0;

	pt = Util::arToCv(Util::rotate(pt, data.copterValues.yaw));

	pt.y = posPx.y + pt.y;
	pt.x = posPx.x + 2 + pt.x;

	line(image, pt2, pt, CF_RED, 1, 8, 0);

	//roll set
	//desde
	pt2.y = 4;
	pt2.x = 0;

	pt2 = Util::arToCv(Util::rotate(pt2, data.copterValues.yaw));

	pt2.x += posPx.x;
	pt2.y += posPx.y;

	//hasta
	pt.x = data.copterSets.roll * ajusteSet;
	pt.y = 0;

	pt = Util::arToCv(Util::rotate(pt, data.copterValues.yaw));

	pt.x = posPx.x + pt.x;
	pt.y = posPx.y + 2 + pt.y;

	line(image, pt2, pt, CF_RED, 1, 8, 0);

	//********** seteo las inclinaciones *********
	//pitch value
	//desde
	pt2.y = 0;
	pt2.x = -4;

	pt2 = Util::arToCv(Util::rotate(pt2, data.copterValues.yaw));

	pt2.x += posPx.x;
	pt2.y += posPx.y;

	//hasta
	pt.y = -data.copterValues.vx * ajusteGet / 10;//pitch
	pt.x = 0;

	pt = Util::arToCv(Util::rotate(pt, data.copterValues.yaw));

	pt.y = pt2.y + pt.y;
	pt.x = pt2.x + pt.x;

	line(image, pt2, pt, CF_BLUE, 1, 8, 0);

	//roll value
	//desde
	pt2.y = -4;
	pt2.x = 0;

	pt2 = Util::arToCv(Util::rotate(pt2, data.copterValues.yaw));

	pt2.x += posPx.x;
	pt2.y += posPx.y;

	//hasta
	pt.x = data.copterValues.vy * ajusteGet / 10;//roll
	pt.y = 0;

	pt = Util::arToCv(Util::rotate(pt, data.copterValues.yaw));

	pt.x = pt2.x + pt.x;
	pt.y = pt2.y + pt.y;

	line(image, pt2, pt, CF_BLUE, 1, 8, 0);

	//dibujo el robot
	//linea 1
	//punto1
	pt.x = -7;
	pt.y = 0;

	pt = Util::arToCv(Util::rotate(pt, data.copterValues.yaw));

	pt.x = posPx.x + pt.x;
	pt.y = posPx.y + pt.y;

	//punto 2
	pt2.x = 7;
	pt2.y = 0;

	pt2 = Util::arToCv(Util::rotate(pt2, data.copterValues.yaw));

	pt2.x = posPx.x + pt2.x;
	pt2.y = posPx.y + pt2.y;
	line(image, pt2, pt, CF_BLACK, 2, 8, 0);

	//linea 2
	//punto1
	pt.x = 0;
	pt.y = -14;

	pt = Util::arToCv(Util::rotate(pt, data.copterValues.yaw));

	pt.x = posPx.x + pt.x;
	pt.y = posPx.y + pt.y;

	//punto 2
	pt2.x = 0;
	pt2.y = 7;

	pt2 = Util::arToCv(Util::rotate(pt2, data.copterValues.yaw));

	pt2.x = posPx.x + pt2.x;
	pt2.y = posPx.y + pt2.y;
	line(image, pt2, pt, CF_BLACK, 2, 8, 0);

//	circle(image, cv::Point(data.tPos.x, data.tPos.y), 4, CF_BLACK, 2, 8, 0);
}

void UI::drawMaxView(Mat &image, const Data &data)
{
	int wid = (data.imgSize.width - data.imgSize.width * (ALT_CAMERA - data.copterValues.altitude) / ALT_CAMERA) / 2;
	int hei = (data.imgSize.height - data.imgSize.height * (ALT_CAMERA - data.copterValues.altitude) / ALT_CAMERA) / 2;

	line(image, cv::Point(wid, hei), cv::Point(data.imgSize.width - wid, hei), CF_BLACK, 2, 8, 0);
	line(image, cv::Point(wid, hei), cv::Point(wid, data.imgSize.height - hei), CF_BLACK, 2, 8, 0);
	line(image, cv::Point(data.imgSize.width - wid, hei), cv::Point(data.imgSize.width - wid, data.imgSize.height - hei), CF_BLACK, 2, 8, 0);
	line(image, cv::Point(wid, data.imgSize.height - hei), cv::Point(data.imgSize.width - wid, data.imgSize.height - hei), CF_BLACK, 2, 8, 0);

}
void UI::drawDestinations(Mat &image, std::vector<cv::Point2i> destinations, const Data &data)
{
	for (size_t i = 0; i < destinations.size(); i++)
	{
		Point pt = Util::mtToPx(destinations[i], data);
		line(image, cv::Point(pt.x, pt.y - 5),
				cv::Point(pt.x, pt.y + 5), CF_BLACK,
				1, 8, 0);

		line(image, cv::Point(pt.x - 5, pt.y),
				cv::Point(pt.x + 5, pt.y), CF_BLACK,
				1, 8, 0);
	}
}
void UI::drawGraphics(Mat &image, const Data &data)
{
	//********************  PARA EL PITCH  ************

	//LINEA CENTRO PITCH
	line(image, cv::Point(0, CENTRO_Y_PITCH), cv::Point(image.size().width, CENTRO_Y_PITCH),
			CF_BLACK, 1, 8, 0);
	//LINEA ARRIBA PITCH
	line(image, cv::Point(0, CENTRO_Y_PITCH + 1 * AMPLIAR_Y2),
			cv::Point(image.size().width, CENTRO_Y_PITCH + 1 * AMPLIAR_Y2), CF_BLACK,
			1, 8, 0);
	//LINEA ABAJO PITCH
	line(image, cv::Point(0, CENTRO_Y_PITCH - 1 * AMPLIAR_Y2),
			cv::Point(image.size().width, CENTRO_Y_PITCH - 1 * AMPLIAR_Y2), CF_BLACK,
			1, 8, 0);
	//PUNTOS
	for (int i = 0; i < int(data.copterSets.pKp.size() - 1); i++)
	{
		line(image,
				cv::Point(i * AMPLIAR_X,
						CENTRO_Y_PITCH + data.copterSets.pKp[i] * AMPLIAR_Y),
				cv::Point((i + 1) * AMPLIAR_X,
						CENTRO_Y_PITCH + data.copterSets.pKp[i + 1] * AMPLIAR_Y),
				CF_RED, 1, 8, 0);
		line(image,
				cv::Point(i * AMPLIAR_X,
						CENTRO_Y_PITCH + data.copterSets.pKi[i] * AMPLIAR_Y),
				cv::Point((i + 1) * AMPLIAR_X,
						CENTRO_Y_PITCH + data.copterSets.pKi[i + 1] * AMPLIAR_Y),
				CV_RGB(0,180,0), 1, 8, 0);
		line(image,
				cv::Point(i * AMPLIAR_X,
						CENTRO_Y_PITCH + data.copterSets.pKd[i] * AMPLIAR_Y),
				cv::Point((i + 1) * AMPLIAR_X,
						CENTRO_Y_PITCH + data.copterSets.pKd[i + 1] * AMPLIAR_Y),
				CV_RGB(0,0,200), 1, 8, 0);

		line(image,
				cv::Point(i * AMPLIAR_X,
						CENTRO_Y_PITCH + data.copterSets.pitchs[i] * AMPLIAR_Y),//*30 grados
				cv::Point((i + 1) * AMPLIAR_X,
						CENTRO_Y_PITCH + data.copterSets.pitchs[i + 1] * AMPLIAR_Y),
				CV_RGB(200, 200, 0), 2, 8, 0);//amarillo

		line(image,
				cv::Point(i * AMPLIAR_X,
						CENTRO_Y_PITCH + data.copterValues.pitchs[i] * AMPLIAR_Y / 30),
				cv::Point((i + 1) * AMPLIAR_X,
						CENTRO_Y_PITCH + data.copterValues.pitchs[i + 1] * AMPLIAR_Y / 30),
				CV_RGB(0,255,255), 2, 8, 0);

	}

	//********************  PARA EL ROLL  ************

	//LINEA CENTRO ROLL
	line(image, cv::Point(0, CENTRO_Y_ROLL), cv::Point(image.size().width, CENTRO_Y_ROLL),
			CF_BLACK, 1, 8, 0);
	//LINEA ARRIBA ROLL
	line(image, cv::Point(0, CENTRO_Y_ROLL + 1 * AMPLIAR_Y2),
			cv::Point(image.size().width, CENTRO_Y_ROLL + 1 * AMPLIAR_Y2), CF_BLACK,
			1, 8, 0);
	//LINEA ABAJO ROLL
	line(image, cv::Point(0, CENTRO_Y_ROLL - 1 * AMPLIAR_Y2),
			cv::Point(image.size().width, CENTRO_Y_ROLL - 1 * AMPLIAR_Y2), CF_BLACK,
			1, 8, 0);

	for (int i = 0; i < int(data.copterSets.rKp.size() - 1); i++)
	{
		line(image,
				cv::Point(i * AMPLIAR_X,
						CENTRO_Y_ROLL + data.copterSets.rKp[i] * AMPLIAR_Y),
				cv::Point((i + 1) * AMPLIAR_X,
						CENTRO_Y_ROLL + data.copterSets.rKp[i + 1] * AMPLIAR_Y),
				CF_RED, 1, 8, 0);
		line(image,
				cv::Point(i * AMPLIAR_X,
						CENTRO_Y_ROLL + data.copterSets.rKi[i] * AMPLIAR_Y),
				cv::Point((i + 1) * AMPLIAR_X,
						CENTRO_Y_ROLL + data.copterSets.rKi[i + 1] * AMPLIAR_Y),
				CV_RGB(0,180,0), 1, 8, 0);
		line(image,
				cv::Point(i * AMPLIAR_X,
						CENTRO_Y_ROLL + data.copterSets.rKd[i] * AMPLIAR_Y),
				cv::Point((i + 1) * AMPLIAR_X,
						CENTRO_Y_ROLL + data.copterSets.rKd[i + 1] * AMPLIAR_Y),
				CV_RGB(0,0,200), 1, 8, 0);

		line(image,
				cv::Point(i * AMPLIAR_X,
						CENTRO_Y_ROLL + data.copterSets.rolls[i] * AMPLIAR_Y),
				cv::Point((i + 1) * AMPLIAR_X,
						CENTRO_Y_ROLL + data.copterSets.rolls[i + 1] * AMPLIAR_Y),
				CV_RGB(200, 200, 0), 2, 8, 0);//amarillo

		line(image,
				cv::Point(i * AMPLIAR_X,
						CENTRO_Y_ROLL + data.copterValues.rolls[i] * AMPLIAR_Y / 30),
				cv::Point((i + 1) * AMPLIAR_X,
						CENTRO_Y_ROLL + data.copterValues.rolls[i + 1] * AMPLIAR_Y / 30),
				CV_RGB(0,255,255), 2, 8, 0);

	}

	//********************  PARA EL YAW  ************
/*	//LINEA CENTRO YAW
	line(image, cv::Point(0, CENTRO_Y_YAW), cv::Point(image.size().width, CENTRO_Y_YAW),
			CF_BLACK, 1, 8, 0);
	//LINEA ARRIBA YAW
	line(image, cv::Point(0, CENTRO_Y_YAW + 1 * AMPLIAR_Y2),
			cv::Point(image.size().width, CENTRO_Y_YAW + 1 * AMPLIAR_Y2), CF_BLACK,
			1, 8, 0);
	//LINEA ABAJO YAW
	line(image, cv::Point(0, CENTRO_Y_YAW - 1 * AMPLIAR_Y2),
			cv::Point(image.size().width, CENTRO_Y_YAW - 1 * AMPLIAR_Y2), CF_BLACK,
			1, 8, 0);

	for (int i = 0; i < int(data.copterSets.yKp.size() - 1); i++)
	{
		line(image,
				cv::Point(i * AMPLIAR_X,
						CENTRO_Y_YAW + data.copterSets.yKp[i] * AMPLIAR_Y),
				cv::Point((i + 1) * AMPLIAR_X,
						CENTRO_Y_YAW + data.copterSets.yKp[i + 1] * AMPLIAR_Y),
				CF_RED, 1, 8, 0);
		line(image,
				cv::Point(i * AMPLIAR_X,
						CENTRO_Y_YAW + data.copterSets.yKi[i] * AMPLIAR_Y),
				cv::Point((i + 1) * AMPLIAR_X,
						CENTRO_Y_YAW + data.copterSets.yKi[i + 1] * AMPLIAR_Y),
				CV_RGB(0,180,0), 1, 8, 0);
		line(image,
				cv::Point(i * AMPLIAR_X,
						CENTRO_Y_YAW + data.copterSets.yKd[i] * AMPLIAR_Y),
				cv::Point((i + 1) * AMPLIAR_X,
						CENTRO_Y_YAW + data.copterSets.yKd[i + 1] * AMPLIAR_Y),
				CV_RGB(0,0,200), 1, 8, 0);

		line(image,
				cv::Point(i * AMPLIAR_X,
						CENTRO_Y_YAW + data.copterSets.yaws[i] * AMPLIAR_Y),
				cv::Point((i + 1) * AMPLIAR_X,
						CENTRO_Y_YAW + data.copterSets.yaws[i + 1] * AMPLIAR_Y),
				CV_RGB(200, 200, 0), 2, 8, 0);//amarillo
*/
	/*	line(image,
				cv::Point(i * AMPLIAR_X,
						CENTRO_Y_YAW + data.copterValues.yaws[i] * AMPLIAR_Y2),
				cv::Point((i + 1) * AMPLIAR_X,
						CENTRO_Y_YAW + data.copterValues.yaws[i + 1] * AMPLIAR_Y2),
				CV_RGB(0,255,255), 2, 8, 0);
				*/

//	}

	//********************  PARA LA ALTITUDE  ************
	//LINEA CENTRO YAW
	line(image, cv::Point(0, CENTRO_Y_YAW), cv::Point(image.size().width, CENTRO_Y_YAW),
			CF_BLACK, 1, 8, 0);
	//LINEA ARRIBA YAW
	line(image, cv::Point(0, CENTRO_Y_YAW + 1 * AMPLIAR_Y2),
			cv::Point(image.size().width, CENTRO_Y_YAW + 1 * AMPLIAR_Y2), CF_BLACK,
			1, 8, 0);
	//LINEA ABAJO YAW
	line(image, cv::Point(0, CENTRO_Y_YAW - 1 * AMPLIAR_Y2),
			cv::Point(image.size().width, CENTRO_Y_YAW - 1 * AMPLIAR_Y2), CF_BLACK,
			1, 8, 0);

	for (int i = 0; i < int(data.copterSets.zKp.size() - 1); i++)
	{
		line(image,
				cv::Point(i * AMPLIAR_X,
						CENTRO_Y_YAW + data.copterSets.zKp[i] * AMPLIAR_Y),
				cv::Point((i + 1) * AMPLIAR_X,
						CENTRO_Y_YAW + data.copterSets.zKp[i + 1] * AMPLIAR_Y),
				CF_RED, 1, 8, 0);
		line(image,
				cv::Point(i * AMPLIAR_X,
						CENTRO_Y_YAW + data.copterSets.zKi[i] * AMPLIAR_Y),
				cv::Point((i + 1) * AMPLIAR_X,
						CENTRO_Y_YAW + data.copterSets.zKi[i + 1] * AMPLIAR_Y),
				CV_RGB(0,180,0), 1, 8, 0);
		line(image,
				cv::Point(i * AMPLIAR_X,
						CENTRO_Y_YAW + data.copterSets.zKd[i] * AMPLIAR_Y),
				cv::Point((i + 1) * AMPLIAR_X,
						CENTRO_Y_YAW + data.copterSets.zKd[i + 1] * AMPLIAR_Y),
				CV_RGB(0,0,200), 1, 8, 0);

		line(image,
				cv::Point(i * AMPLIAR_X,
						CENTRO_Y_YAW + data.copterSets.altitudes[i] * AMPLIAR_Y),
				cv::Point((i + 1) * AMPLIAR_X,
						CENTRO_Y_YAW + data.copterSets.altitudes[i + 1] * AMPLIAR_Y),
				CV_RGB(200, 200, 0), 2, 8, 0);//amarillo
	/*	line(image,
				cv::Point(i * AMPLIAR_X,
						CENTRO_Y_YAW + data.copterValues.altitudes[i] * AMPLIAR_Y2),//Y2
				cv::Point((i + 1) * AMPLIAR_X,
						CENTRO_Y_YAW + data.copterValues.altitudes[i + 1] * AMPLIAR_Y2),//Y2
				CV_RGB(0,255,255), 2, 8, 0);*/

	}

	putText(image, "proportional", cv::Point(10, 10), CV_FONT_HERSHEY_SIMPLEX,
			0.4, CF_RED);
	putText(image, "integrate", cv::Point(10, 30), CV_FONT_HERSHEY_SIMPLEX, 0.4,
			CF_GREEN);
	putText(image, "derivate", cv::Point(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.4,
			CF_BLUE);
	putText(image, "total_pitch", cv::Point(10, 70), CV_FONT_HERSHEY_SIMPLEX, 0.4,
			CV_RGB(200, 200, 0));
	putText(image, "ROLL", cv::Point(10, CENTRO_Y_ROLL - AMPLIAR_Y2 + 20), CV_FONT_HERSHEY_SIMPLEX, 0.4,
				CV_RGB(200, 200, 0));
	putText(image, "YAW", cv::Point(10, CENTRO_Y_YAW - AMPLIAR_Y2 + 20), CV_FONT_HERSHEY_SIMPLEX, 0.4,
					CV_RGB(200, 200, 0));

	line(image, cv::Point(image.size().width - 100, 55),
			cv::Point(image.size().width - 10, 55), CF_BLACK, 2, 8, 0);

	line(image, cv::Point(image.size().width - 55, 10),
			cv::Point(image.size().width - 55, 100), CF_BLACK, 2, 8, 0);

	circle(image,
			cv::Point(image.size().width - 55 + data.copterValues.roll * 5, 55),
			4, CF_PURPLE, -1, 8, 0);

	circle(image,
			cv::Point(image.size().width - 55,
					55 - data.copterValues.pitch * 5), 4, CF_RED, -1, 8, 0);
}

void UI::drawNextDestination(cv::Mat &image,
		std::vector<cv::Point2i> destinations, int destination, const Data &data)
{
	Point pt = Util::mtToPx(destinations[destination], data);
	line(image, cv::Point(pt.x, pt.y - 5),
			cv::Point(pt.x, pt.y + 5), CF_BLACK, 2, 8,
			0);

	line(image, cv::Point(pt.x - 5, pt.y),
			cv::Point(pt.x + 5, pt.y), CF_BLACK, 2, 8,
			0);

}

void UI::destroy()
{
	destroyAllWindows();
}



UI::~UI()
{
	destroyAllWindows();
}

} /* namespace ar */
