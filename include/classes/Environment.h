#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include "include/util/structures.h"
#include "include/util/Util.h"
#include "include/classes/Video.h"
#include <iostream>
#include <stdio.h>

#include <core/core.hpp>
namespace ar
{

class Environment
{
	private:
		Video* video;
		cv::Mat image;

		bool robotOnline;
		std::vector<ar::Position > lastPositions;
		std::vector<cv::Point2i > destinations;
		std::vector<int> destinationsZ;
	public:
		// constructor / destructor
		Environment(int cam);
		virtual ~Environment();

		// robot
		ar::Point getRobotPositionInImage(std::vector<ar::Point> puntos, Data dt);
		ar::Position predictedPositionCalc();
		void deleteFirstPosition();
		void addLastPosition(const ar::Position position);
		bool isRobotOnline();

		// destinations
		ar::Point* getDestinoPositionInImage(std::vector<ar::Point> puntos[], int lenght);
		void addDestination(cv::Point2i point);
		void deleteLastDestination();
		const cv::Point2i* getDestination(unsigned int i);
		const std::vector<cv::Point2i> getDestinations();
		void clearDestinations();

		//destinations in Z
		void addDestinationZ(int point);
		void deleteLastDestinationZ();
		const int getDestinationZ(unsigned int i);
		const std::vector<int> getDestinationsZ();
		void clearDestinationsZ();

		// image and video
		Video* getVideo();
		const cv::Mat& getVideoFrame();
		cv::Size getImageSize();

		// util
		static float distanceCalc(const ar::Point point_1, const ar::Point point_2);


};
}
#endif /* ENVIRONMENT_H_ */
