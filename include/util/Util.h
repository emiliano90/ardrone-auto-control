/*
 * Util.h
 *
 *  Created on: 01/06/2015
 *      Author: toshiba
 */

#ifndef UTIL_H_
#define UTIL_H_
#include "math.h"
#include "stdio.h"
#include "include/util/structures.h"

#include <core/core.hpp>
#include "math.h"

namespace ar {

class Util {
public:

	Util();
	virtual ~Util();
	static float radAGrados(float rad);
	static float gradosARad(float grados);
	static ar::Point rotate(ar::Point pt, float yaw);
	static float getAngle(Point pt, Point pt2, float yaw);
	static float distancia(Point pt, Point pt2);
	static cv::Point arToCv(Point pt);
	static Point getDistXY(int dist, float grados);
	static float pxToMt(int px, Data dt);
	static int mtToPx(float mt, Data dt);
	static int angleMax(int dist);
	static char *itoa(long n);

};

} /* namespace std */
#endif /* UTIL_H_ */
