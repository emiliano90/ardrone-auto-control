extern "C" {
#include <VP_Api/vp_api_thread_helper.h>
#include <ardrone_tool/Navdata/ardrone_navdata_client.h>
#include <ardrone_tool/Control/ardrone_control.h>
#include <ardrone_tool/UI/ardrone_input.h>
}

#include "include/util/structures.h"
#include "include/util/Util.h"
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include <include/classes/PIDY.h>
#include "include/classes/PIDRP.h"


PROTO_THREAD_ROUTINE(copter_control, data);
