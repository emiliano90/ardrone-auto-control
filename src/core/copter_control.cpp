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

	float derivate = 0.85; //80% de la proporcional
	float proporcional = 0.00050;//0.00065
//0.00015 prop
	//derivate 80%
	//int 96
	PID_RP* r_pid = new PID_RP(proporcional, 0.000000076, derivate * proporcional, 0, 0, 560000, 0.12,
											0, 1);
	PID_RP* p_pid = new PID_RP(proporcional, 0.000000076, derivate * proporcional, 0, 0, 560000, 0.12,
											0, 1);

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

	int safety = 7;

	Point destino = threadAttr->data.destino;
	float set_point_pitch = 0;
	float set_point_yaw = 0;
	float roll_sp = 0;
	float pitch_sp = 0;
	float yaw_sp = 0;
	float gaz_sp = 0;
	bool bHover = false;
	bool bHovering = false;
	bool bAterrizar = false;

	struct timeval start, end;
	bool autoPilot = false;
	struct timeval pitch_start, pitch_end;
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
	//sem_wait(&threadAttr->mutex);

			if (!bAterrizar) //&& threadAttr->data.tPos.find)
			{
				safety = 30;
				if(autoPilot && !threadAttr->data.bReadead && !threadAttr->data.tPos.find)//si ya despego
				{

					ar::Point pos;
					pos.x = threadAttr->data.tPos.x;
					pos.y = threadAttr->data.tPos.y;

					set_point_yaw = Util::getAngle(pos, threadAttr->data.destino, threadAttr->data.copterValues.yaw);

				//	printf("set yaw: %.2f", set_point_yaw);

					y_pid->setPoint(set_point_yaw);

					if(destino.x != threadAttr->data.destino.x || destino.y != threadAttr->data.destino.y)
					{
						destino = threadAttr->data.destino;

						//	float dist = Util::distancia(pos, destino);
						p_pid->setPoint(destino.y);
						r_pid->setPoint(destino.x);

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

/*

					if(pitch_time == 0 && dist > 40)
					{
						if (ang < Util::angleMax(dist) && ang > -Util::angleMax(dist) && fabs(yaw_sp) < 0.3)//&&  dist > 60
						{
					//	pitch_sp = p_pid->update(relative.x, threadAttr->data.copterValues.vx);
					//	roll_sp = r_pid->update(-relative.y, -threadAttr->data.copterValues.vy);

							gettimeofday(&pitch_start, NULL);
							pitch_time = abs((float)relative.x / (float)Util::mtToPx(VEL_15, threadAttr->data) * 1000.0) + 200;

							//lo acelero si esta muy cerca, sino no le afecta
							if(relative.x < 50 && threadAttr->data.copterValues.vx < 80)
								pitch_time *= 1.3;
							if(relative.x > 0)//adelante
								pitch_sp = -ANGLE_PITCH;
							else
								pitch_sp = ANGLE_PITCH;

							if(pitch_time < 100){
								pitch_sp = 0;
								pitch_time = 0;
							}

						}
						//ESTO NO SE VA A EJECUTAR
						else if (dist < 60 && fabs(ang) > 90 && fabs(yaw_sp) < 0.3)//&&  dist > 60
						{

					//	pitch_sp = p_pid->update(relative.x, threadAttr->data.copterValues.vx);
					//	roll_sp = r_pid->update(-relative.y, -threadAttr->data.copterValues.vy);

							gettimeofday(&pitch_start, NULL);
							pitch_time = abs((float)relative.x / (float)Util::mtToPx(VEL_15, threadAttr->data) * 1000.0) + 200;

							//lo acelero si esta muy cerca, sino no le afecta
							if(relative.x > -50 && threadAttr->data.copterValues.vx > -80)
								pitch_time *= 1.3;
							if(relative.x > 0)//adelante
								pitch_sp = -ANGLE_PITCH;
							else
								pitch_sp = ANGLE_PITCH;//atras // se deberia ejecutar este

							if(pitch_time < 100){
								pitch_sp = 0;
								pitch_time = 0;
							}
							printf("Atras: %d \n", relative.x);

						}
					}


					if(roll_time == 0 && fabs(yaw_sp) < 0.3 && dist < 0)// && (threadAttr->data.copterValues.vy > 90 || threadAttr->data.copterValues.vy < -90))
					{

						gettimeofday(&roll_start, NULL);
						roll_time = abs((float)relative.y / (float)Util::mtToPx(VEL_15, threadAttr->data) * 1000.0) + 200;

						//lo acelero si esta muy cerca, sino no le afecta
						if(relative.y > 0 && relative.y < 50 && threadAttr->data.copterValues.vy > 0)
							roll_time *= 2;
						else if(relative.y < 0 && relative.y > -50 && threadAttr->data.copterValues.vy < 0)
							roll_time *= 2;

						if(relative.y > 0)
							roll_sp = ANGLE_ROLL;
						else
							roll_sp = -ANGLE_ROLL;

						if(roll_time < 100)
						{
							roll_sp = 0;
							roll_time = 0;
						}
						printf("roll_time: %.2f \n", roll_time);


					}
					//corroboro el tiempo de vuelo en pitch
					gettimeofday(&pitch_end, NULL);

					seconds = pitch_end.tv_sec - pitch_start.tv_sec;
					useconds = pitch_end.tv_usec - pitch_start.tv_usec;
					float mt = ((seconds) * 1000 + useconds / 1000.0) + 0.5;

					if (mt > pitch_time && pitch_time != 0)
					{

						//bPitch = true;
						pitch_time = 0;
						pitch_sp = 0;
						//espero
						if (!bPitchWait)
						{
							gettimeofday(&pitch_start, NULL);
							pitch_time = 700;//0.5seg
							bPitchWait= true;
						}
						else
							bPitchWait = false;
					}
					if(fabs(yaw_sp) > 0.5)
					{
						pitch_sp = 0;
						roll_sp = 0;
						if(!bPitchWait)
							pitch_time= 0;
						if(!bRollWait)
							roll_time= 0;
					}

					//corroboro el tiempo de vuelo en roll
					gettimeofday(&roll_end, NULL);

					seconds = roll_end.tv_sec - roll_start.tv_sec;
					useconds = roll_end.tv_usec - roll_start.tv_usec;
					mt = ((seconds) * 1000 + useconds / 1000.0) + 0.5;

					if (mt > roll_time && roll_time  != 0)
					{

						//bRoll = true;
						roll_time = 0;
						roll_sp = 0;
						//espero
						if(!bRollWait)
						{
							gettimeofday(&roll_start, NULL);
							roll_time = 500;//0.5seg
							bRollWait = true;
						}
						else
							bRollWait = false;
					}


*/








					if (fabs(yaw_sp) < 0.5)
					{
						pitch_sp = p_pid->update(-relative.x, threadAttr->data.copterValues.vx, threadAttr->data.elapsedTime);
						roll_sp = r_pid->update(relative.y, -threadAttr->data.copterValues.vy, threadAttr->data.elapsedTime);
						float change = fabs(dist - lastDist);

						if(dist < 25)
						{
							bFlightHover = true;
							pitch_sp = 0;
							roll_sp = 0;
							p_pid->reset();
							r_pid->reset();
						}
						else
							bFlightHover = false;
					}
					/*else
					{
						p_pid->update(relative.x, threadAttr->data.copterValues.vy);
						r_pid->update(-relative.y, -threadAttr->data.copterValues.vx);
						pitch_sp = 0;
						roll_sp = 0;
					}*/
					/*
					if (dist > 50 && fabs(yaw_sp) < 0.5)
					{
						pitch_sp = p_pid->update(-relative.x, threadAttr->data.copterValues.vy);
					//	roll_sp = r_pid->update(-relative.y, -threadAttr->data.copterValues.vx);

					}
					else
					{
						p_pid->update(relative.x, threadAttr->data.copterValues.vy);
						r_pid->update(-relative.y, -threadAttr->data.copterValues.vx);

						pitch_sp = 0;
						roll_sp = 0;
					}

*/
					lastDist = dist;
					threadAttr->data.bReadead = true;

					threadAttr->data.dist_destino.z = dist;
					threadAttr->data.dist_destino.x = relative.y;//es al reves
					threadAttr->data.dist_destino.y = relative.x;
					threadAttr->data.copterSets.roll = roll_sp;
					threadAttr->data.copterSets.pitch = pitch_sp;
					threadAttr->data.copterSets.yaw = yaw_sp;
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


				}
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

				/*

				float roll = r_pid->update(threadAttr->data.tPos.x);	//roll
				float pitch = p_pid->update(threadAttr->data.tPos.y);	//pitch
//					float thrust = t_pid->update(120 - threadAttr->data.tPos.z);//thust

				float roll_sp = roll;
				float pitch_sp = pitch;

				if (roll_sp > CAP) roll_sp = CAP;
				else if (roll_sp < -CAP) roll_sp = -CAP;

				if (pitch_sp > CAP) pitch_sp = CAP;
				else if (pitch_sp < -CAP) pitch_sp = -CAP;

				if (thrust_sargp > TH_CAP) thrust_sp = TH_CAP;
				else if (thrust_sp < 0) thrust_sp = 0;

				threadAttr->data.copterSets.pitch = -pitch_sp;
				threadAttr->data.copterSets.roll = roll_sp;
				threadAttr->data.copterSets.yaw = 0;
*/
			/*	threadAttr->data.copterValues.rolls.push_back(
						threadAttr->data.copterValues.roll);
				if (threadAttr->data.copterValues.rolls.size() == 200) threadAttr->data.copterValues.rolls.erase(
						threadAttr->data.copterValues.rolls.begin());

				threadAttr->data.copterValues.pitchs.push_back(
						threadAttr->data.copterValues.pitch);
				if (threadAttr->data.copterValues.pitchs.size() == 200) threadAttr->data.copterValues.pitchs.erase(
						threadAttr->data.copterValues.pitchs.begin());
*//*
				threadAttr->data.copterSets.Kp.push_back(r_pid->getKp());
				if (threadAttr->data.copterSets.Kp.size() == 200) threadAttr->data.copterSets.Kp.erase(
						threadAttr->data.copterSets.Kp.begin());

				threadAttr->data.copterSets.Ki.push_back(r_pid->getKi());
				if (threadAttr->data.copterSets.Ki.size() == 200) threadAttr->data.copterSets.Ki.erase(
						threadAttr->data.copterSets.Ki.begin());

				threadAttr->data.copterSets.Kd.push_back(r_pid->getKd());
				if (threadAttr->data.copterSets.Kd.size() == 200) threadAttr->data.copterSets.Kd.erase(
						threadAttr->data.copterSets.Kd.begin());
*
				threadAttr->data.copterSets.rolls.push_back(roll_sp);
				if (threadAttr->data.copterSets.rolls.size() == 200) threadAttr->data.copterSets.rolls.erase(
						threadAttr->data.copterSets.rolls.begin());
*
				cflieCopter->setThrust(thrust_sp);

				cflieCopter->setPitch(pitch_sp);	//-pith
				cflieCopter->setRoll(-roll_sp);	//xx+roll  //decia-roll
				cflieCopter->setYaw(0);
*/
			}
			else
			{	//para que aterrice si no lo ve
				safety--;
			}

			if (bAterrizar)
			{

			//	ardrone_tool_set_ui_pad_start(0);//aterriza 0 -- despega 1
				cout << endl << "Aterrizando..." << endl;
				threadAttr->data.copterSets.pitch = 0;
				threadAttr->data.copterSets.roll = 0;


				usleep(800000); //0,5 segundo
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
//				case 'v':
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
	//sem_post(&threadAttr->mutex);
		}
		usleep(500); //0,035 segundos//0,010
	}

	cout << endl << "Aterrizandoooo..." << endl;
	ardrone_tool_set_ui_pad_start(0);

	threadAttr->data.copterSets.pitch = 0;
	threadAttr->data.copterSets.roll = 0;


	return NULL;
}

