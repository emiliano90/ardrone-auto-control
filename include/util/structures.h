#ifndef STRUCTURES_H_
#define STRUCTURES_H_

//	# system includes
#include <semaphore.h>
#include <VP_Os/vp_os_signal.h>

#include <core/core.hpp>

using namespace std;

// distance calculation
const int DIST_CERCA = 0; //cm
const int DIST_LEJOS = 110; //cm

const int DIAM_CERCA = 4; //px
const int DIAM_LEJOS = 6; //px

const float VEL_05 = 0.4044; // m/seg
const float VEL_10 = 0.6284; // m/seg
const float VEL_15 = 1.4427 / 1.4; // m/seg //con 1.5 andaba
const float VEL_20 = 1.7587; // m/seg
const float VEL_25 = 2.2094; // m/seg

const float SIZE_X_MT = 2.35;//2,9 en casa
const float SIZE_Y_MT = 1.74;//2,15 en casa

#ifndef CF_WHITE
#define CF_WHITE CV_RGB(255,255,255)
#endif
#ifndef CF_BLACK
#define CF_BLACK CV_RGB(0,0,0)
#endif
#ifndef CF_RED
#define CF_RED CV_RGB(255,0,0)
#endif
#ifndef CF_BLUE
#define CF_BLUE CV_RGB(0, 0, 255)
#endif
#ifndef CF_GREEN
#define CF_GREEN CV_RGB(0, 255, 0)
#endif
#ifndef CF_YELLOW
#define CF_YELLOW CV_RGB(255, 255, 0)
#endif
#ifndef CF_PURPLE
#define CF_PURPLE CV_RGB(255, 0, 255)
#endif



namespace ar{
struct Position
{
		int x;
		int y;
		int z;
		bool find;

		Position(int x, int y, int z) :
				x(x), y(y), z(z)
		{
		}
		Position()
		{
		}
};

struct Size
{
		int height;
		int width;

		Size(int h, int w) :
				height(h), width(w)
		{
		}
		;
		Size()
		{
		}
};


struct Point
{
		int x;
		int y;

		Point(cv::Point p){
			x = p.x;
			y = p.y;
		}
		Point(int x, int y) :
				x(x), y(y)
		{
		}
		Point()
		{
		}
};

struct Copter
{
	/*theta  pitch in milli-degrees
	  phi    roll  in milli-degrees
	  psi    yaw   in milli-degrees */
		vector<float> pKp;
		vector<float> pKi;
		vector<float> pKd;

		vector<float> rKp;
		vector<float> rKi;
		vector<float> rKd;

		vector<float> yKp;
		vector<float> yKi;
		vector<float> yKd;

		vector<float> rolls;
		vector<float> pitchs;
		vector<float> yaws;

		float   pitch;                  /*!< UAV's pitch in milli-degrees */
		float   roll;                    /*!< UAV's roll  in milli-degrees */
		float   yaw;                    /*!< UAV's yaw   in milli-degrees */

		int     altitude;               /*!< UAV's altitude in centimeters */

		float   last_vx;                     /*!< UAV's estimated linear velocity */
		float   last_vy;                     /*!< UAV's estimated linear velocity */


		float   vx;                     /*!< UAV's estimated linear velocity */
		float   vy;                     /*!< UAV's estimated linear velocity */
		float   vz;                     /*!< UAV's estimated linear velocity */

		int    ctrl_state;             /*!< Flying state (landed, flying, hovering, etc.) defined in CTRL_STATES enum. */
		string    ctrl_state_sz;             /*!< Flying state (landed, flying, hovering, etc.) defined in CTRL_STATES enum. */
		int    battery; /*!< battery voltage filtered (mV) */

};

struct Data
{
		std::vector<Position> tLastPos;
		Position tPos;
		long elapsedTime;
		Size imgSize;
		bool bReadead;
		char key;
		Copter copterSets;
		Copter copterValues;
		Point destino;
		float   desired_yaw;
		Position dist_destino;
		float yaw_correction;
};

struct ThreadAttr
{
		sem_t mutex;
		vp_os_mutex_t mutex2;
		Data data;
};

struct HSVColor
{
		cv::Scalar min;
		cv::Scalar max;

		HSVColor(cv::Scalar min, cv::Scalar max) :
				min(min), max(max)
		{
		}
		HSVColor()
		{
		}
};
}


#endif /* STRUCTURES_H_ */
