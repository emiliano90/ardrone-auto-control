#include "include/classes/Environment.h"

using namespace std;
using namespace cv;
using namespace ar;

namespace ar
{
Environment::Environment(int cam)
{
	this->video = new Video(cam);
	this->robotOnline = true;
}

Environment::~Environment()
{
	this->video->~Video();
	this->image.release();
}

Video* Environment::getVideo()
{
	return this->video;
}

const Mat& Environment::getVideoFrame()
{
	this->image = this->video->getFrame();
	return this->image;
}

ar::Point Environment::getRobotPositionInImage(std::vector<ar::Point> puntos)
{
	ar::Point p(0, 0);
	vector<ar::Point> centros;
	vector<vector<ar::Point> > grupos;

	Position predecida = this->predictedPositionCalc();
	ar::Point pred;
	pred.x = predecida.x;
	pred.y = predecida.y;
	int pos = -1;
	float dist = -1;
	float sizes = -1;
	int pos2 = -1;
	//armo grupos de puntos de acuerdo a la cercania de los mismos
	while (puntos.size() != 0)
	{
		vector<ar::Point> row;
		row.push_back(puntos[0]);
		puntos.erase(puntos.begin());
		for (int e = 0; e < (int)puntos.size(); e++)
		{
			if (Environment::distanceCalc(puntos[e], row[0]) < 39)
			{
				row.push_back(puntos[e]);
				puntos.erase(puntos.begin() + e);
				e--;
			}
		}
		grupos.push_back(row);
	}


	//calculo el centro de los grupos
	for (int i = 0; i < (int)grupos.size(); i++)
	{
		p.x = 0;
		p.y = 0;
		for (int e = 0; e < (int)grupos[i].size(); e++)
		{
			p.x += grupos[i][e].x;
			p.y += grupos[i][e].y;

		}
		if (grupos[i].size() != 0)
		{
			p.x /= grupos[i].size();
			p.y /= grupos[i].size();
			centros.push_back(p);
		}
	}

	//Elijo el grupo mas cercano a destino
	if (this->lastPositions.size() != 0)
	{
		for (int i = 0; i < (int)centros.size(); i++)
		{
			if (grupos[i].size() > sizes)
			{
				sizes = grupos[i].size();
				pos2 = i;
			}
			if (Environment::distanceCalc(centros[i], pred) < dist
					|| dist == -1)
			{
				dist = Environment::distanceCalc(centros[i], pred);
				pos = i;
			}

		}
	}
	else if (this->lastPositions.size() == 0)
	{
		if (grupos.size() == 0)
			pos = -1;
		else if (grupos.size() == 1)
			pos = 0;
		else
		{
			pos = 0;
			for (int i = 1; i < (int)grupos.size(); i++)
			{
				if (grupos[pos].size() < grupos[i].size()) pos = i;
			}
		}
	}

	if (pos != -1 && dist < 80)
	{
		this->robotOnline = true;
		return centros[pos];
	}
	else if (!this->isRobotOnline() && pos2 != -1)
	{
		this->robotOnline = true;
		return centros[pos2];
	}

	this->robotOnline = false;
	return ar::Point(-1, -1);

}

ar::Point* Environment::getDestinoPositionInImage(std::vector<ar::Point> puntos[], const int lenght)
{
	ar::Point p(0, 0);
	vector<ar::Point> centros[lenght];
	vector<vector<ar::Point> > grupos[lenght];
	//vector<ar::Point> puntos[lenght];
	ar::Point destinos[lenght];

	ar::Point pred;

	//armo grupos de puntos de acuerdo a la cercania de los mismos
	for(int i = 0; i < lenght; i++)
	{

		while (puntos[i].size() != 0)
		{
//			vector<ar::Point>  p =puntos[i];
			vector<ar::Point> row;
			row.push_back(puntos[i][0]);
			puntos[i].erase(puntos[i].begin());
			for (int e = 0; e < (int)puntos[i].size(); e++)
			{
				if (Environment::distanceCalc(puntos[i][e], row[0]) < 20)
				{
					row.push_back(puntos[i][e]);
					puntos[i].erase(puntos[i].begin() + e);
					e--;
				}
			}
			grupos[i].push_back(row);
		}
	}
	//calculo el centro de los grupos
	for(int j = 0; j < lenght; j++)
	{
		for (int i = 0; i < (int)grupos[j].size(); i++)
		{
			p.x = 0;
			p.y = 0;
			for (int e = 0; e < (int)grupos[j][i].size(); e++)
			{
				p.x += grupos[j][i][e].x;
				p.y += grupos[j][i][e].y;

			}
			if (grupos[j][i].size() != 0)
			{
				p.x /= grupos[j][i].size();
				p.y /= grupos[j][i].size();
				centros[j].push_back(p);
			}
		}
	}
	int pos[lenght];

	//elijo el grupo mas grande
	for (int j = 0; j < lenght; j++)
	{
		pos[j] = -1;
		if (grupos[j].size() == 0)
			pos[j] = -1;
		else if (grupos[j].size() == 1)
			pos[j] = 0;
		else
		{
			pos[j] = 0;
			for (int i = 1; i < (int)grupos[j].size(); i++)
			{
				if (grupos[j][pos[j]].size() < grupos[j][i].size())
					pos[j] = i;
			}
		}

		if (pos[j] != -1)
			destinos[j] = centros[j][pos[j]];
		else
			destinos[j] = ar::Point(-1, -1);

	}
	return destinos;
}

cv::Size Environment::getImageSize()
{
	return this->image.size();
}

void Environment::addLastPosition(const ar::Position position)
{
	this->lastPositions.push_back(position);
}
float Environment::distanceCalc(const ar::Point point_1,
		const ar::Point point_2)
{
	return sqrt(
			pow(fabs(point_1.x - point_2.x), 2)
					+ pow(fabs(point_1.y - point_2.y), 2));
}
Position Environment::predictedPositionCalc()
{
	Position desplazamiento;
	desplazamiento.x = 0;
	desplazamiento.y = 0;
	desplazamiento.z = 0;
	int e = 0;

	for (int n = 0; n < (int)(this->lastPositions.size()) - 1; n++)
	{
		//	centro = predecida->getCentro();
		desplazamiento.x += (this->lastPositions[n].x
				- this->lastPositions[n + 1].x) * (n + 1);
		desplazamiento.y += (this->lastPositions[n].y
				- this->lastPositions[n + 1].y) * (n + 1);
		desplazamiento.z += (this->lastPositions[n].z
				- this->lastPositions[n + 1].z) * (n + 1);

		e += n + 1;
	}
	if (this->lastPositions.size() > 1)
	{
		desplazamiento.x /= e;
		desplazamiento.y /= e;
		desplazamiento.z /= e;

		Position p;
		p.x = (this->lastPositions[this->lastPositions.size() - 1]).x
				- desplazamiento.x;
		p.y = this->lastPositions[this->lastPositions.size() - 1].y
				- desplazamiento.y;
		p.z =
				this->lastPositions[this->lastPositions.size() - 1].z
						- desplazamiento.z;
		return p;
	}
	else if (this->lastPositions.size() == 1) return this->lastPositions[0];
	else return desplazamiento;
}

void Environment::addDestination(Point2i point)
{
	this->destinations.push_back(point);
}
void Environment::clearDestinations()
{
	this->destinations.clear();
}

void Environment::deleteLastDestination()
{
	this->destinations.pop_back();
}
void Environment::deleteFirstPosition()
{
	this->lastPositions.erase(this->lastPositions.begin());
}
const cv::Point2i* Environment::getDestination(unsigned int i)
{
	if (i >= this->destinations.size())
	{
		return NULL;
	}
	return &(this->destinations[i]);
}

bool Environment::isRobotOnline()
{
	return this->robotOnline;
}

const std::vector<cv::Point2i> Environment::getDestinations()
{
	return this->destinations;
}

}
