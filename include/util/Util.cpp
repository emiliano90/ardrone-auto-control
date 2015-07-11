/*
 * Util.cpp
 *
 *  Created on: 01/06/2015
 *      Author: toshiba
 */

#include "Util.h"

namespace ar
{

Util::Util() {
	// TODO Auto-generated constructor stub

}

Util::~Util() {
	// TODO Auto-generated destructor stub
}


float Util::radAGrados(float rad)
{
	return rad * 180.0 / M_PI;
}
float Util::gradosARad(float grados)
{
	return grados * M_PI / 180.0;
}

ar::Point Util::rotate(ar::Point pt, float grados)
{
	ar::Point pt2;
	float rad = gradosARad(grados);

	pt2.x = pt.x * cos(rad) - pt.y * sin(rad);
	pt2.y = pt.x * sin(rad) + pt.y * cos(rad);

	return pt2;
}

Point Util::getDistXY(int dist, float grados)
{
	Point pt;
	float rad = gradosARad(grados);

	pt.y = dist * sin(rad);
	pt.x = dist * cos(rad);
	return pt;


}
float Util::distancia(Point pt, Point pt2)
{
	pt.x -= pt2.x;
	pt.y -= pt2.y;
	return sqrt(abs(pt.x * pt.x + pt.y * pt.y));
}
float Util::getAngle(Point pt, Point pt2, float yaw)
{
	pt.x -= pt2.x;
	pt.y -= pt2.y;
	float ang;
	if(pt.x != 0)
	{
		ang = atan(abs((float)pt.y/(float)pt.x));
		ang = radAGrados(ang);

		if(pt.x > 0)
			if(pt.y < 0)
				ang = -(ang + 90);
			else
				ang = -(90 -ang);
		else
			if(pt.y < 0)
				ang += 90;
			else
				ang = 90 - ang;

		/*
		ang -= yaw;
		if(ang > 180)
			ang -= 360;
		else if(ang < -180)
			ang += 360;*/

	}
	return ang;

}
cv::Point Util::arToCv(Point pt)
{
	return cv::Point(pt.x, pt.y);
}
float Util::pxToMt(int px, Data dt)
{
	return px * SIZE_X_MT / dt.imgSize.width;
}
int Util::mtToPx(float mt, Data dt)
{
	return mt * dt.imgSize.width / SIZE_X_MT;
}
int Util::angleMax(int dist)
{//-12
	float c = (float)(5.0 - 15.0) / (float)(230.0 - 20.0);
	float ang = c * dist + 15;
	if(ang < 5)
		ang = 5;
	return ang;
}
char *Util::itoa(long n)
{
    int len = n==0 ? 1 : floor(log10(abs(n)))+1;
    if (n<0) len++; // room for negative sign '-'

    char    *buf = (char*)calloc(sizeof(char), len+1); // +1 for null
    snprintf(buf, len+1, "%ld", n);
    return   buf;
}
}
