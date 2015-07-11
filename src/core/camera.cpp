#include "include/core/camera.h"
#include <VP_Os/vp_os_signal.h>
#include "Navdata/navdata.h"

using namespace std;
using namespace cvb;
using namespace cv;
using namespace ar;

int nDestino = 0;
const int DESTINOS = 4;
const bool BUSCAR_DESTINOS = false;
HSVColor hsv;
HSVColor hsvDestinos[DESTINOS];

DEFINE_THREAD_ROUTINE(camera_top, data)
{
	if (threadAttr == NULL)
		{
			threadAttr = new ThreadAttr;

			vp_os_mutex_init(&threadAttr->mutex2);
		}
	/*
#ifndef aaa
#define aaa
	if (threadAttr == NULL)
			threadAttr = new ThreadAttr;

	vp_os_mutex_init(&threadAttr->mutex2);
	sem_init(&threadAttr->mutex, 0, 1);
#endif
*/
	string times;

	struct timeval start, end;

	long seconds, useconds, mtime = 0;
	long mtime2 = 0;

//	ThreadAttr *args = (ThreadAttr *) data;

	printf("\r\n----------------------------------------\r\n");
	printf("\t\tPID: %d", (unsigned int) pthread_self());
	printf("\r\n----------------------------------------\r\n");

	// color que busco -> umbral minimo y maximo
	hsv.max.val[0] = 179;
	hsv.max.val[1] = 255;
	hsv.max.val[2] = 255;

	hsv.min.val[0] = 135; // 0.75 135
	hsv.min.val[1] = 50; //76 bien  56  0.25 66   estaba en 80
	hsv.min.val[2] = 80; //125  //130 0.54 140  estaba en 130


	// amarillo
	hsvDestinos[0].max.val[0] = 81;
	hsvDestinos[0].max.val[1] = 127;
	hsvDestinos[0].max.val[2] = 195;

	hsvDestinos[0].min.val[0] = 0; // 0.75 135
	hsvDestinos[0].min.val[1] = 91; //76 bien  56  0.25 66
	hsvDestinos[0].min.val[2] = 185; //125  //130 0.54 140

	// verde
	hsvDestinos[1].max.val[0] = 101;
	hsvDestinos[1].max.val[1] = 149;
	hsvDestinos[1].max.val[2] = 108;

	hsvDestinos[1].min.val[0] = 83;
	hsvDestinos[1].min.val[1] = 101;
	hsvDestinos[1].min.val[2] = 101;

	// azul
	hsvDestinos[2].max.val[0] = 125;
	hsvDestinos[2].max.val[1] = 255;
	hsvDestinos[2].max.val[2] = 232;

	hsvDestinos[2].min.val[0] = 86;
	hsvDestinos[2].min.val[1] = 101;
	hsvDestinos[2].min.val[2] = 145;

	// naranja
	hsvDestinos[3].max.val[0] = 21;
	hsvDestinos[3].max.val[1] = 255;
	hsvDestinos[3].max.val[2] = 233;

	hsvDestinos[3].min.val[0] = 10;
	hsvDestinos[3].min.val[1] = 55;
	hsvDestinos[3].min.val[2] = 145;


	gettimeofday(&start, NULL);

	Environment* env = new Environment(1);

	Mat img = env->getVideoFrame();
	cv::Size imgSize = img.size();

	printf("width: %d, height: %d", imgSize.width, imgSize.height);

	threadAttr->data.imgSize.height = imgSize.height;
	threadAttr->data.imgSize.width = imgSize.width;

	//Defino los destinos
//	env->addDestination(Point2i(320, 240));

	namedWindow("Deteccion", WINDOW_AUTOSIZE);
	//Para detectar clicks en la ventana
	int destSize = DESTINOS; // = (int) env->getDestinations().size();
	setMouseCallback("Deteccion", callbackMouseDestinations, &destSize);

	/********************/
	//	Writers para guardar videos

	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, 80, "%d-%m-%Y %I:%M:%S", timeinfo);
	string str(buffer);

	VideoWriter outputVideo;
	VideoWriter outputVideo2;
	VideoWriter outputVideo3;

	/*******************/

	bool bContinue = true;

	int fps = 0;
	bool quit = false;
	while (!quit)
	{

		gettimeofday(&end, NULL);

		seconds = end.tv_sec - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		mtime2 = ((seconds) * 1000 + useconds / 1000.0) + 0.5;
		mtime += mtime2;
		fps++;
		if (mtime > 1000)
		{
			//printf("Red Object Elapsed time: %ld milliseconds, fps: %d \n",mtime, fps);
			fps = 0;
			mtime = 0;
		}

		gettimeofday(&start, NULL);

		Mat img = env->getVideoFrame();

		env->getVideo()->grabVideoAndData(PROYECT_PATH + "videos/", "avi", str,
				outputVideo, img);

		UI::drawWindow("Camara1", img);

		Mat segmentated(imgSize, CV_8UC3);

		// transformo a BN la matriz segmentated, para que el color HSV(hsv) se vea negro, y el resto blanco
		cv::cvtColor(img, segmentated, COLOR_BGR2HSV);

		if((int)env->getDestinations().size() != DESTINOS && BUSCAR_DESTINOS)
		{
			std::vector<ar::Point> *puntos_dest;
			puntos_dest = env->getVideo()->transformToBlackAndWihte(segmentated, hsvDestinos, DESTINOS);
			UI::drawWindow("colores", segmentated);
			ar::Point *destinos;
			destinos = env->getDestinoPositionInImage(puntos_dest, DESTINOS);

			env->clearDestinations();
			for(int i = 0; i < DESTINOS; i++)
			{
				if(destinos[i].x != -1)
					env->addDestination(cv::Point(destinos[i].x, destinos[i].y));
			}

			destSize = (int) env->getDestinations().size();
			cv::cvtColor(img, segmentated, COLOR_BGR2HSV);


		}
		//invento unos destinos:
		if((int)env->getDestinations().size() == 0 && !BUSCAR_DESTINOS)
		{
			env->addDestination(cv::Point(210, 160));
			env->addDestination(cv::Point(210, 320));
			env->addDestination(cv::Point(430, 320));
			env->addDestination(cv::Point(430, 160));
		}




		std::vector<ar::Point> puntos;
		puntos = env->getVideo()->transformToBlackAndWihte(segmentated, hsv);

		Mat grayImage(imgSize, CV_8UC3, cv::Scalar(255, 255, 255));

		vp_os_mutex_lock(&threadAttr->mutex2);
		ar::Point p = env->getRobotPositionInImage(puntos);

		//asigno un valor aleatorio  por q no tengo camara
	//	p.x = threadAttr->data.imgSize.width / 2 -50;
	//	p.y = threadAttr->data.imgSize.height / 2 -50;
		//modifico la posicion segun la altura del robot
		p.x = (threadAttr->data.imgSize.width / 2) + (p.x - (threadAttr->data.imgSize.width / 2)) * (ALT_CAMERA - threadAttr->data.copterValues.altitude) / ALT_CAMERA;
		p.y = (threadAttr->data.imgSize.height / 2) + (p.y - (threadAttr->data.imgSize.height / 2)) * (ALT_CAMERA - threadAttr->data.copterValues.altitude) / ALT_CAMERA;




		Position predecida = env->predictedPositionCalc();
		Position pos(p.x, p.y, 0);
		float dist = -1;

		threadAttr->data.bReadead = false;

		if(threadAttr->data.tPos.find == true)//si antes no encontro nada acumulo el tiempo
			threadAttr->data.elapsedTime = mtime2;

		else
			threadAttr->data.elapsedTime += mtime2;

		// meto datos en memoria compartida mientras tenga al robot visto
		if (threadAttr->data.tLastPos.size() != 0 && pos.x != -1) dist =
				Environment::distanceCalc(ar::Point(predecida.x, predecida.y),
						ar::Point(pos.x, pos.y));

		if (threadAttr->data.tLastPos.size() == 0 && pos.x != -1)
		{
			threadAttr->data.tPos = pos;
			threadAttr->data.tPos.find = true;
			env->addLastPosition(pos);
			threadAttr->data.tLastPos.push_back(pos);
		}
		else if (dist < 100 && dist != -1)
		{
			env->addLastPosition(pos);

			threadAttr->data.tLastPos.push_back(pos);
			threadAttr->data.tPos.find = true;
			threadAttr->data.tPos = pos;
			if (threadAttr->data.tLastPos.size() == FRAME)
			{
				env->deleteFirstPosition();
				threadAttr->data.tLastPos.erase(threadAttr->data.tLastPos.begin());
			}

		}
		else
		{
			// pierdo el robot
			printf("Robot Lost\n");
			Position p;
			p.x = -1;
			p.y = -1;
			threadAttr->data.tPos.find = false;
			threadAttr->data.tPos = p;

		}

		// dibujo un circulo donde esta el robot
		UI::drawRobot(grayImage, threadAttr->data);
		//UI::drawCircle(grayImage,
		//		ar::Point(threadAttr->data.tPos.x, threadAttr->data.tPos.y), 4, CF_BLACK);

		if(env->getDestinations().size() != 0)
		{
			// dibuja una cruz en los destinos
			UI::drawDestinations(grayImage, env->getDestinations());
			UI::drawNextDestination(grayImage, env->getDestinations(), nDestino);

			threadAttr->data.destino.x = env->getDestination(nDestino)->x;
			threadAttr->data.destino.y = env->getDestination(nDestino)->y;
		}


		// informacion en la pantalla.
		string line1 = format("SET-> Pitch: %.3f, Roll: %.3f, Yaw: %.3f, Dist: %d, D_x: %d, D_y: %d",
				threadAttr->data.copterSets.pitch, threadAttr->data.copterSets.roll,
				threadAttr->data.copterSets.yaw, threadAttr->data.dist_destino.z, threadAttr->data.dist_destino.x, threadAttr->data.dist_destino.y);

		string line2 = format(
				"GET--> Pitch: %.3f , Roll: %.3f, Yaw: %.3f, Altitude: %d cm, Desired Yaw: %.3f",
				threadAttr->data.copterValues.pitch, threadAttr->data.copterValues.roll,
				threadAttr->data.copterValues.yaw, threadAttr->data.copterValues.altitude, threadAttr->data.desired_yaw);
		string line3 = format(
				"Vel-Y: %f, Vel-X: %f, Vel-Z: %f cm/s",
				threadAttr->data.copterValues.vy, threadAttr->data.copterValues.vx,
				threadAttr->data.copterValues.vz);
		string line4 = format("Battery: %d %%, State: %s",
				threadAttr->data.copterValues.battery, threadAttr->data.copterValues.ctrl_state_sz.c_str());

		string line5 = format("POSICION--> X: %d, Y: %d, Z: %d",
				threadAttr->data.tPos.x, threadAttr->data.tPos.y, threadAttr->data.tPos.z);

		UI::drawText(grayImage, line1, ar::Point(10, 10), CF_RED);
		UI::drawText(grayImage, line2, ar::Point(10, 30), CF_BLUE);
		UI::drawText(grayImage, line3, ar::Point(10, 50), CF_BLUE);
		UI::drawText(grayImage, line4, ar::Point(10, 70), CF_BLUE);
		UI::drawText(grayImage, line5, ar::Point(10, 90), CF_BLACK);

		UI::drawMaxView(grayImage, threadAttr->data);

		UI::drawWindow("Deteccion", grayImage);

		//Genero los graficos sobre PID
		Mat graficos(cvSize(imgSize.width, HEIGHT_GRAF), CV_8UC3,
				cv::Scalar(255, 255, 255));

		UI::drawGraphics(graficos, threadAttr->data);

		vp_os_mutex_unlock(&threadAttr->mutex2);

		UI::drawWindow("Graficos", graficos);

		// compongo dos imagenes
		int height = imgSize.height;
		if(imgSize.height < HEIGHT_GRAF)
			height  = HEIGHT_GRAF;
		Mat grande(cvSize(imgSize.width * 2, height), CV_8UC3,
				cv::Scalar(255, 255, 255));

		grayImage.copyTo(
				grande.colRange(0, imgSize.width).rowRange(0, imgSize.height));
		graficos.copyTo(
				grande.colRange(imgSize.width, imgSize.width * 2).rowRange(0,
						HEIGHT_GRAF));
		// fin de composicion.

		env->getVideo()->grabVideoAndData(PROYECT_PATH + "videos/", "avi",
				str + "_2", outputVideo2, grande);

//		env->getVideo()->grabVideoAndData(PROYECT_PATH + "videos/", "avi",
//				str + "_3", outputVideo3, segmentated);

		UI::drawWindow("redMorphology", segmentated);

		char k = cvWaitKey(bContinue) & 0xff;

		vp_os_mutex_lock(&threadAttr->mutex2);
	//sem_wait(&threadAttr->mutex);
		threadAttr->data.key = k;
		vp_os_mutex_unlock(&threadAttr->mutex2);
	//sem_post(&threadAttr->mutex);

		switch (k)
		{
			case 27:
			case 'q':
			case 'Q':
				quit = true;
			break;
			case 's':
			case 'S':
				bContinue = false;
			break;
			case 'n':
			case 'N':
				bContinue = true;
			break;
		}
	}

	outputVideo.release();
	outputVideo2.release();
	outputVideo3.release();

	return 0;
}

void callbackMouseDestinations(int event, int x, int y, int flags, void* data)
{
	int size_ = *(int*) data;
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		int size = size_;
		nDestino++;
		if (nDestino == size) nDestino = 0;
	}
}
void callbackMouseAddDestination(int event, int x, int y, int flags,
		void* userdata)
{
}

