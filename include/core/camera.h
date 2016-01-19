#ifndef CAMERA_H_
#define CAMERA_H_

//	#system includes
#include <string>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <semaphore.h>
#include <fstream>
#include <sys/time.h>

//	# own includes
#include "include/util/structures.h"

//	# own classes
#include "include/classes/Environment.h"
#include "include/classes/Video.h"
#include "include/classes/UI.h"

//	# cvblob include
#include <cvblob.h>

//	# opencv includes
#include <core/core.hpp>
#include <highgui/highgui.hpp>
#include <imgproc/imgproc.hpp>
extern "C" {
#include <VP_Api/vp_api_thread_helper.h>
}

using namespace std;
using namespace cvb;
using namespace cv;
using namespace ar;

#ifndef START
#define START	90
#endif

#ifndef AMPLIAR_Y
#define AMPLIAR_Y	710
#endif

#ifndef AMPLIAR_Y2
#define AMPLIAR_Y2	90
#endif

#ifndef AMPLIAR_X
#define AMPLIAR_X	3
#endif

#ifndef CENTRO_Y_PITCH
#define CENTRO_Y_PITCH	(START + AMPLIAR_Y2)
#endif

#ifndef CENTRO_Y_ROLL
#define CENTRO_Y_ROLL	(START + AMPLIAR_Y2 * 3)
#endif

#ifndef CENTRO_Y_YAW
#define CENTRO_Y_YAW	(START + AMPLIAR_Y2 * 5)
#endif

#ifndef HEIGHT_GRAF
#define HEIGHT_GRAF	(AMPLIAR_Y2 * 6 + 90)
#endif
#ifndef PROYECT_PATH_
#define PROYECT_PATH_
const string PROYECT_PATH("/home/toshiba/tesis/Ardrone/");
#endif




const size_t FRAME = 7;

PROTO_THREAD_ROUTINE(camera_top, data);

void callbackMouseDestinations(int event, int x, int y, int flags,
		void* userdata);
void callbackMouseAddDestination(int event, int x, int y, int flags,
		void* userdata);

#endif /* CAMERA_H_ */
