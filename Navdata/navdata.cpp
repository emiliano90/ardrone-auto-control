
#include "navdata.h"
#include "include/util/structures.h"
#include <VP_Os/vp_os_signal.h>
#include <stdlib.h>
#include "include/util/Util.h"

using namespace ar;


ar::ThreadAttr *threadAttr = NULL;

/* Initialization local variables before event loop  */
inline C_RESULT demo_navdata_client_init( void* data )
{

	if (threadAttr == NULL)
	{
		threadAttr = new ThreadAttr;

		vp_os_mutex_init(&threadAttr->mutex2);
	}
	vp_os_mutex_lock(&threadAttr->mutex2);
	threadAttr->data.yaw_correction = 0;
	vp_os_mutex_unlock(&threadAttr->mutex2);

	/*
#ifndef aaa
#define aaa



	sem_init(&threadAttr->mutex, 0, 1);
#endif
*/
  return C_OK;
}

/* Receving navdata during the event loop */
inline C_RESULT demo_navdata_client_process( const navdata_unpacked_t* const navdata )
{
	const navdata_demo_t*nd = &navdata->navdata_demo;


	//	printf("=====================\nNavdata for flight demonstrations =====================\n\n");
	string state;
	switch(nd->ctrl_state){
	case CTRL_DEFAULT:
		state = "Default";
		break;
	case CTRL_INIT:
		state= "Init";
		break;
	case CTRL_LANDED:
		state= "Landed";
		break;
	case CTRL_FLYING:
		state= "Flying";
		break;
	case CTRL_HOVERING:
		state= "Hovering";
		break;
	case CTRL_TEST:
		state= "Test";
		break;
	case CTRL_TRANS_TAKEOFF:
		state= "TRANS_TAKEOFF";
		break;
	case CTRL_TRANS_GOTOFIX:
		state= "TRANS_GOTOFIX";
		break;
	case CTRL_TRANS_LANDING:
		state= "TRANS_LANDING";
		break;
	case CTRL_TRANS_LOOPING:
		state= "TRANS_LOOPING";
		break;
	case 131072:
		state= "Landed";
		break;
	case 393216:
		state= "Taking-off-Floor";
		break;
	case 393217:
		state= "Taking-off-Air";
		break;
	case 262144:
		state= "Hovering";
		break;
	case 524288:
		state= "Landing";
		break;
	case 458752:
		state= "Stabilizing";
		break;
	case 196608:
		state= "Moving";
		break;
	case 262153:
	case 196613:
		state= "Undefined";
		break;
	default:
		//char buffer [sizeof(long)*8+1];

		state= Util::itoa(nd->ctrl_state);
		break;

	}
	//458753 estabilizado
/*
	printf("Control state : %s --> %i\n",state.c_str(), nd->ctrl_state);
	printf("Battery level : %i mV\n",nd->vbat_flying_percentage);
	printf("Orientation   : [Theta] %4.3f  [Phi] %4.3f  [Psi] %4.3f\n",nd->theta,nd->phi,nd->psi);
	printf("Altitude      : %i\n",nd->altitude);
	printf("Speed         : [vX] %4.3f  [vY] %4.3f  [vZPsi] %4.3f\n",nd->vx,nd->vy,nd->vz);
*/

	sem_wait(&threadAttr->mutex);
	vp_os_mutex_lock(&threadAttr->mutex2);
	if(threadAttr->data.copterValues.ctrl_state  == 458752 && nd->ctrl_state == 458753)//stabilizing
		threadAttr->data.yaw_correction = nd->psi / 1000;

	threadAttr->data.copterValues.pitch = nd->theta / 1000;
	threadAttr->data.copterValues.roll = nd->phi / 1000;
	threadAttr->data.copterValues.yaw = nd->psi / 1000 - threadAttr->data.yaw_correction;
	if(threadAttr->data.copterValues.yaw > 180)
		threadAttr->data.copterValues.yaw -= 360;
	else if(threadAttr->data.copterValues.yaw < -180)
		threadAttr->data.copterValues.yaw += 360;

	float v = nd->vx;
	threadAttr->data.copterValues.vx = (threadAttr->data.copterValues.last_vx + nd->vx) / 2;
	threadAttr->data.copterValues.last_vx = v;

	v = nd->vy;
	threadAttr->data.copterValues.vy = (threadAttr->data.copterValues.last_vy + nd->vy) / 2;
	threadAttr->data.copterValues.last_vy = v;

	threadAttr->data.copterValues.vz = nd->vz;
	threadAttr->data.copterValues.altitude = nd->altitude;

	threadAttr->data.copterValues.battery = nd->vbat_flying_percentage;
	threadAttr->data.copterValues.ctrl_state = nd->ctrl_state;
	threadAttr->data.copterValues.ctrl_state_sz = state;


	vp_os_mutex_unlock(&threadAttr->mutex2);
	//sem_post(&threadAttr->mutex);

//	printf("\033[8A");

  return C_OK;
}

/* Relinquish the local resources after the event loop exit */
inline C_RESULT demo_navdata_client_release( void )
{
  return C_OK;
}

/* Registering to navdata client */
BEGIN_NAVDATA_HANDLER_TABLE
  NAVDATA_HANDLER_TABLE_ENTRY(demo_navdata_client_init, demo_navdata_client_process, demo_navdata_client_release, NULL)
END_NAVDATA_HANDLER_TABLE

