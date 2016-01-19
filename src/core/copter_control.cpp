#include "include/core/copter_control.h"
#include "Navdata/navdata.h"
#include <VP_Os/vp_os_signal.h>

const float ANGLE_ROLL = 0.10;
const float ANGLE_PITCH = 0.10;

using namespace ar;
DEFINE_THREAD_ROUTINE(copter_control, data)
{
	if (threadAttr == NULL)
	{
		threadAttr = new ThreadAttr;

		vp_os_mutex_init(&threadAttr->mutex2);
	}

	PID_Y* y_pid = new PID_Y(0.015, 0, 0.02, 0, 0, 1500, -1500, 1,
								0, 1); //D0.45, 0.0003,0.35
/*
	PID_RP* r_pid = new PID_RP(0.0004, 0, 0, 0, 0, 45000, 0.07, //D: 0.014
										0, 1); //D0.45, 0.0003,0.35

	PID_RP* p_pid = new PID_RP(0.0004, 0, 0, 0, 0, 45000, 0.07,
										0, 1); //D0.45, 0.0003,0.35
*/
//	0,367

	float derivate = 0.35; //80% de la proporcional
	float proporcional = 0.0025;//0.00065
	//0.0005 en pixeles

//0.00015 prop
	//derivate 80%
	//int 96
	PID_RP* r_pid = new PID_RP(proporcional, 0.0000001, derivate * proporcional, 0, 0, 1560000, 0.12,
											0, 1);
	PID_RP* p_pid = new PID_RP(proporcional, 0.0000001, derivate * proporcional, 0, 0, 1560000, 0.12,
											0, 1);
	PID_Z* z_pid = new PID_Z(0.0009, 0.0000000001, 0.24, 0, 0, 1560000, 0.35,
												0, 1);
//0.001    zxxx 26
	//para pid con dt multiple y dividi integral y derivador por dt (150) bastante bien
//I:0.00000005

/*	PID_RP* r_pid = new PID_RP(0.00015, 0.000000036, 0.27, 0, 0, 360000, 0.10,
										0, 1);
	PID_RP* p_pid = new PID_RP(0.00015, 0.000000036, 0.27, 0, 0, 360000, 0.10,
										0, 1);
*/
	//para pid con dt multiple y dividi integral y derivador por dt (150) anda bastante bien
/*
	PID_RP* r_pid = new PID_RP(0.0002, 0.00000006, 0.5, 0, 0, 180000, 0.10,
									0, 1);
	PID_RP* p_pid = new PID_RP(0.0002, 0.00000006, 0.5, 0, 0, 180000, 0.10,
									0, 1); */

/*anda bastante bien con el pid sin dt
	PID_RP* r_pid = new PID_RP(0.0005, 0.0002, 0.028, 0, 0, 150, 0.06, //D: 0.014
									0, 1); //D0.45, 0.0003,0.35

	PID_RP* p_pid = new PID_RP(0.0005, 0.0002, 0.028, 0, 0, 150, 0.06,
									0, 1); //D0.45, 0.0003,0.35
*/


	/*
	PID_RP* p_pid = new PID_RP(10, 0, 1, 0, 0, 1500, -1500, 5000,
										0, 1); //D0.45, 0.0003,0.35
	PID_RP* r_pid = new PID_RP(10, 0, 10, 0, 0, 1500, -1500, 3000,
											0, 1); //D0.45, 0.0003,0.35
*/

	//  ardrone_tool_set_progressive_cmd(control_flag, left_right, front_back, up_down, turn, 0.0, 0.0);


	bool bQuit = false;

	int safety = 80;

	Position destino = threadAttr->data.destino;
	z_pid->setPoint(destino.z);

	float set_point_pitch = 0;
	float set_point_yaw = 0;
	float roll_sp = 0;
	float pitch_sp = 0;
	float yaw_sp = 0;
	float gaz_sp = 0;
	bool bHover = false;
	bool bHoverFirst = false;

	bool bHovering = false;
	bool bAterrizar = false;

	struct timeval start, end;
	bool autoPilot = false;
	struct timeval pitch_start, pitch_end;
	struct timeval hover_start, hover_end;
	struct timeval roll_start, roll_end;
	float pitch_time = 0;
	float roll_time = 0;
	bool bPitch = false;
	bool bPitchWait = false;
	bool bRoll = false;
	bool bRollWait = false;
	bool bFlightHover = true;
	long seconds, useconds, mtime = 0;

	float lastDist = 0;
	gettimeofday(&start, NULL);

	ardrone_at_set_flat_trim();

	while (!bQuit && safety != 0)
	{

		gettimeofday(&end, NULL);

		seconds = end.tv_sec - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		mtime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;

		if (mtime > 35) //0.035 segundos
		{
			gettimeofday(&start, NULL);

			vp_os_mutex_lock(&threadAttr->mutex2);

			if (!bAterrizar) //&& threadAttr->data.tPos.find)
			{

				if(autoPilot && !threadAttr->data.bReadead && threadAttr->data.tPos.find)//si ya despego
				{

					safety = 10;

					ar::Point pos;
					pos.x = threadAttr->data.tPos.x;
					pos.y = threadAttr->data.tPos.y;

					set_point_yaw = Util::getAngle(pos, threadAttr->data.destino, threadAttr->data.copterValues.yaw);

				//	printf("set yaw: %.2f", set_point_yaw);

					y_pid->setPoint(set_point_yaw);

					if(destino.x != threadAttr->data.destino.x || destino.y != threadAttr->data.destino.y)
					{
						destino.x = threadAttr->data.destino.x;
						destino.y = threadAttr->data.destino.y;

						//	float dist = Util::distancia(pos, destino);
						p_pid->setPoint(destino.y);
						r_pid->setPoint(destino.x);

					}
					if(destino.z != threadAttr->data.destino.z)
					{
						destino.z = threadAttr->data.destino.z;

						//	float dist = Util::distancia(pos, destino);
						z_pid->setPoint(destino.z);

					}

					Point relative = pos;
					relative.x -= destino.x;
					relative.y -= destino.y;

					float ang = set_point_yaw;
					ang -= threadAttr->data.copterValues.yaw;
					if(ang > 180)
						ang -= 360;
					else if(ang < -180)
						ang += 360;

					float dist = Util::distancia(pos, destino);

					//calculo la distancia en x y en y segun la rotacion
					relative = Util::getDistXY(dist, ang);


					if(dist > 70)//lejos apunto
					{
						bFlightHover = true;
						yaw_sp = y_pid->update(threadAttr->data.copterValues.yaw);	//yaw
					}
					else
					{
						bFlightHover = true;
						yaw_sp = 0;

					}
				/*	if(dist < 60 && fabs(ang) > 90)//cerca y atras mio mantengo el mismo angulo.
					{
						bFlightHover = true;
						yaw_sp = 0;
					}
					else if (dist < 20)//si esta muy cerca no giro//iba 20
					{
						bFlightHover = true;
						yaw_sp = 0;
					}
					else //entre 20 y 60 de distancia
					{
						bFlightHover = true;
						yaw_sp = y_pid->update(threadAttr->data.copterValues.yaw);	//yaw
					}
					if(fabs(yaw_sp) > 0.5)
						bFlightHover = true;
					*/






					yaw_sp = 0;
					pitch_sp = p_pid->update(-relative.x, threadAttr->data.copterValues.vx, threadAttr->data.elapsedTime);
					roll_sp = r_pid->update(relative.y, -threadAttr->data.copterValues.vy, threadAttr->data.elapsedTime);
					gaz_sp = z_pid->update(threadAttr->data.copterValues.altitude, threadAttr->data.elapsedTime);

					float change = fabs(dist - lastDist);

					if(dist < 10)
					{
						if(!bHoverFirst)
							gettimeofday(&hover_start, NULL);

						bHoverFirst = true;
						bFlightHover = true;
						pitch_sp = 0;
						roll_sp = 0;
						p_pid->reset();
						r_pid->reset();

						//corroboro el tiempo de vuelo en hover
						gettimeofday(&hover_end, NULL);

						seconds = hover_end.tv_sec - hover_start.tv_sec;
						useconds = hover_end.tv_usec - hover_start.tv_usec;
						float mt = ((seconds) * 1000 + useconds / 1000.0) + 0.5;
						if (mt > 2000)
							threadAttr->data.changeDestino = true;

					}
					else
					{
						bHoverFirst = false;
						bFlightHover = false;
					}


					lastDist = dist;
					threadAttr->data.bReadead = true;

					threadAttr->data.dist_destino.z = dist;
					threadAttr->data.dist_destino.x = relative.y;//es al reves
					threadAttr->data.dist_destino.y = relative.x;
					threadAttr->data.copterSets.roll = roll_sp;
					threadAttr->data.copterSets.pitch = pitch_sp;
					threadAttr->data.copterSets.yaw = yaw_sp;
					threadAttr->data.copterSets.altitude = gaz_sp;
					threadAttr->data.desired_yaw = ang;

					//PITCH kp - ki - kd
					threadAttr->data.copterSets.pKp.push_back(p_pid->getKp());
					if (threadAttr->data.copterSets.pKp.size() == 200) threadAttr->data.copterSets.pKp.erase(
							threadAttr->data.copterSets.pKp.begin());

					threadAttr->data.copterSets.pKi.push_back(p_pid->getKi());
					if (threadAttr->data.copterSets.pKi.size() == 200) threadAttr->data.copterSets.pKi.erase(
							threadAttr->data.copterSets.pKi.begin());

					threadAttr->data.copterSets.pKd.push_back(p_pid->getKd());
					if (threadAttr->data.copterSets.pKd.size() == 200) threadAttr->data.copterSets.pKd.erase(
							threadAttr->data.copterSets.pKd.begin());

					//PITCH kp - ki - kd
					threadAttr->data.copterSets.rKp.push_back(r_pid->getKp());
					if (threadAttr->data.copterSets.rKp.size() == 200) threadAttr->data.copterSets.rKp.erase(
							threadAttr->data.copterSets.rKp.begin());

					threadAttr->data.copterSets.rKi.push_back(r_pid->getKi());
					if (threadAttr->data.copterSets.rKi.size() == 200) threadAttr->data.copterSets.rKi.erase(
							threadAttr->data.copterSets.rKi.begin());

					threadAttr->data.copterSets.rKd.push_back(r_pid->getKd());
					if (threadAttr->data.copterSets.rKd.size() == 200) threadAttr->data.copterSets.rKd.erase(
							threadAttr->data.copterSets.rKd.begin());

					//YAW kp - ki - kd
					threadAttr->data.copterSets.yKp.push_back(y_pid->getKp());
					if (threadAttr->data.copterSets.yKp.size() == 200) threadAttr->data.copterSets.yKp.erase(
							threadAttr->data.copterSets.yKp.begin());

					threadAttr->data.copterSets.yKi.push_back(y_pid->getKi());
					if (threadAttr->data.copterSets.yKi.size() == 200) threadAttr->data.copterSets.yKi.erase(
							threadAttr->data.copterSets.yKi.begin());

					threadAttr->data.copterSets.yKd.push_back(y_pid->getKd());
					if (threadAttr->data.copterSets.yKd.size() == 200) threadAttr->data.copterSets.yKd.erase(
							threadAttr->data.copterSets.yKd.begin());


					//set yaws_
					threadAttr->data.copterSets.yaws.push_back(
							threadAttr->data.copterSets.yaw);
					if (threadAttr->data.copterSets.yaws.size() == 200) threadAttr->data.copterSets.yaws.erase(
							threadAttr->data.copterSets.yaws.begin());

					//ALTITUDE kp - ki - kd
					threadAttr->data.copterSets.zKp.push_back(z_pid->getKp());
					if (threadAttr->data.copterSets.zKp.size() == 200) threadAttr->data.copterSets.zKp.erase(
							threadAttr->data.copterSets.zKp.begin());

					threadAttr->data.copterSets.zKi.push_back(z_pid->getKi());
					if (threadAttr->data.copterSets.zKi.size() == 200) threadAttr->data.copterSets.zKi.erase(
							threadAttr->data.copterSets.zKi.begin());

					threadAttr->data.copterSets.zKd.push_back(z_pid->getKd());
					if (threadAttr->data.copterSets.zKd.size() == 200) threadAttr->data.copterSets.zKd.erase(
							threadAttr->data.copterSets.zKd.begin());


					//set altitudes
					threadAttr->data.copterSets.altitudes.push_back(
							threadAttr->data.copterSets.altitude);
					if (threadAttr->data.copterSets.altitudes.size() == 200) threadAttr->data.copterSets.altitudes.erase(
							threadAttr->data.copterSets.altitudes.begin());

					//	set pitchs
					threadAttr->data.copterSets.pitchs.push_back(
							threadAttr->data.copterSets.pitch);
					if (threadAttr->data.copterSets.pitchs.size() == 200) threadAttr->data.copterSets.pitchs.erase(
							threadAttr->data.copterSets.pitchs.begin());

					//	set rolls
					threadAttr->data.copterSets.rolls.push_back(
							threadAttr->data.copterSets.roll);
					if (threadAttr->data.copterSets.rolls.size() == 200) threadAttr->data.copterSets.rolls.erase(
							threadAttr->data.copterSets.rolls.begin());

					//gets rolls - piths - yaws
					threadAttr->data.copterValues.rolls.push_back(
							threadAttr->data.copterValues.roll);
					if (threadAttr->data.copterValues.rolls.size() == 200) threadAttr->data.copterValues.rolls.erase(
							threadAttr->data.copterValues.rolls.begin());

					threadAttr->data.copterValues.pitchs.push_back(
							threadAttr->data.copterValues.pitch);
					if (threadAttr->data.copterValues.pitchs.size() == 200) threadAttr->data.copterValues.pitchs.erase(
							threadAttr->data.copterValues.pitchs.begin());

					threadAttr->data.copterValues.yaws.push_back(
							threadAttr->data.copterValues.yaw);
					if (threadAttr->data.copterValues.yaws.size() == 200) threadAttr->data.copterValues.yaws.erase(
							threadAttr->data.copterValues.yaws.begin());

					threadAttr->data.copterValues.altitudes.push_back(
							threadAttr->data.copterValues.altitude);
					if (threadAttr->data.copterValues.altitudes.size() == 200) threadAttr->data.copterValues.altitudes.erase(
							threadAttr->data.copterValues.altitudes.begin());


				}
				else if((!threadAttr->data.tPos.find || threadAttr->data.bReadead) && autoPilot)
				{
					safety--;
					if(safety < 0)
						bAterrizar = true;
				}
				else
					safety = 15;

				if (!bHover)
					if(pitch_sp == 0 && roll_sp == 0 && bFlightHover)
						ardrone_tool_set_progressive_cmd(false, 0, 0, gaz_sp, yaw_sp, 0, 0);
					else
						ardrone_tool_set_progressive_cmd(true, roll_sp, pitch_sp, gaz_sp, yaw_sp, 0, 0);

				else if(bHovering)
				{
					ardrone_tool_set_progressive_cmd(0, 0, 0, 0, 0, 0, 0);
					bHovering = false;
				}


			}
			//aterriza si no lo vio
			else
			{

			//	ardrone_tool_set_ui_pad_start(0);//aterriza 0 -- despega 1
				cout << endl << "Aterrizando..." << endl;
				ardrone_tool_set_ui_pad_start(0);
				threadAttr->data.copterSets.pitch = 0;
				threadAttr->data.copterSets.roll = 0;
				threadAttr->data.copterSets.altitude = 0;

			}

			switch (threadAttr->data.key)
			{
				case 'a':
				case 'A':
					yaw_sp = 0;
					pitch_sp = 0;
					roll_sp = 0;
					gaz_sp = 0;
					if(ardrone_tool_set_ui_pad_start(1) == C_OK)
						printf("TakeOff Ok");
					bAterrizar = false;
				break;
				case 'd':
				case 'D':
					gaz_sp = 0;
				break;
				case 's':
				case 'S':
					gaz_sp = 0.2;
				break;
				case 'x':
				case 'X':
					gaz_sp = -0.2;
				break;
				case 'c':
				case 'C':
					autoPilot = true;
				break;
				case 27:
				case 'q':
				case 'Q':
					bQuit = true;
				break;
				case 'v':
				case 'V':
					bAterrizar = false;
				break;
				case 'f':
				case 'F':
					ardrone_tool_set_ui_pad_select(0);
				break;
				case 'e':
				case 'E':
				case '\271':
					set_point_yaw = set_point_yaw + 90;
					if(set_point_yaw > 180)
						set_point_yaw = -180 - (180 - set_point_yaw	);

					y_pid->setPoint(set_point_yaw);
					bHover = false;
				break;
				case 'w':
				case 'W':
				case '\267':
					set_point_yaw = set_point_yaw - 90;
					if(set_point_yaw < -180)
						set_point_yaw = 180 + (180 + set_point_yaw);

					y_pid->setPoint(set_point_yaw);
					bHover = false;
				break;
				case 'g':
				case 'G':
					ardrone_tool_set_ui_pad_select(1);
				break;
				case 'z':
				case 'Z':
					autoPilot = false;
					if(ardrone_tool_set_ui_pad_start(0) == C_OK)
						printf("Landing Ok");
				break;
				case '\270': //8 adelante
					pitch_sp -= 0.2;
					bHover = false;
				break;
				case '\262': //2 atras
					pitch_sp += 0.2;
					bHover = false;
				break;
				case '\265': //5 hover
					if(!bHover)
					{
						yaw_sp = 0;
						pitch_sp = 0;
						roll_sp = 0;
						gaz_sp = 0;
						bHover = true;
						bHovering = true;
					}
					else
					{
						bHover = false;
						bHovering = false;
					}
				break;
				case '\264': //4  izquierda
					roll_sp -= 0.2;
					bHover = false;
				break;
				case '\266': //6  derecha
					roll_sp += 0.2;
					bHover = false;
				break;
				case '\261': //1  girar izquierda
					yaw_sp -= 0.1;
					bHover = false;
				break;
				case '\263': //3  girar derecha
					yaw_sp += 0.1;
					bHover = false;
				break;
			}
			threadAttr->data.key = 0;

			vp_os_mutex_unlock(&threadAttr->mutex2);
		}
		usleep(500); //0,035 segundos//0,010
	}

	cout << endl << "Aterrizando" << endl;
	ardrone_tool_set_ui_pad_start(0);

	threadAttr->data.copterSets.pitch = 0;
	threadAttr->data.copterSets.roll = 0;


	return NULL;
}

