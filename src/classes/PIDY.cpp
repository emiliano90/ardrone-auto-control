#include "include/classes/PIDY.h"

#include <iostream>

#include <stdio.h>
//using namespace std;
PID_Y::PID_Y(float P = 1.0, float I = 0.0, float D = 10.0, int Derivator = 0, float Integrator = 0,
        int Integrator_max = 250, int Integrator_min = -100, float P_limit = 12, float set_point = 0.0, float power = 1.0){

	this->Kp = P;
	this->Ki = I;
	this->Kd = D;
	this->Derivator = Derivator;
	this->power = power;
	this->Integrator = Integrator;
	this->Integrator_max = Integrator_max;
	this->Integrator_min = Integrator_min;
	this->last_error = 0.0;
	this->last_value = 0.0;
	this->set_point = set_point;
	this->error =0.0;
	this->P_limit = P_limit;

}

PID_Y::~PID_Y() {

}

float PID_Y::update(float current_value){
//Calculate PID output value for given reference input and feedback


	error = set_point - current_value;
	if(error > 180 || error < -180)
	{
		if(current_value > 0)
			current_value = -180 - (180 - current_value);
		else
			current_value = 180 + (180 + current_value);
	}
	error = set_point - current_value;
//	printf("error:  %.2f \n",  error);

	float change = error - last_error;
	float last_change = last_error - last_last_error;

	//aumento Kp si me alejo
		P_value = Kp * error;

	if (P_value > P_limit)
		P_value = P_limit;
	else if (P_value < -P_limit)
		P_value = -P_limit;

	float changes = 0;
	if (error != last_error)
		Integrators.push_back(change);
	else
		Integrators.push_back(last_change);

	size_t COUNT = 3;
	if (Integrators.size() > COUNT)
		Integrators.erase(Integrators.begin());
	for (size_t i = 0; i < Integrators.size();i++)
		changes += Integrators[i];
	changes = changes / Integrators.size();

	//aumento Kd   si me alejo
	D_value = Kd * changes;


	Derivator = error;
	Integrator += error;

	if (Integrator > Integrator_max)
		Integrator = Integrator_max;
	else if(Integrator < Integrator_min)
		Integrator = Integrator_min;

	I_value = Integrator * Ki;

	last_last_error = last_error;
	last_error = error;
	last_value = current_value;

	float total = P_value + I_value + D_value;
	if (total > P_limit)
	total = P_limit;
	else if (total < -P_limit)
		total = -P_limit;

	return total;
}

float PID_Y::update2(float current_value){
//Calculate PID output value for given reference input and feedback

	error = set_point - current_value;
	if(error > 180 || error < -180)
	{
		if(current_value > 0)
			current_value = -180 - (180 - current_value);
		else
			current_value = 180 + (180 + current_value);
	}
	error = set_point - current_value;

	P_value = Kp * error;

	float change = error - last_error;
	float last_change = last_error - last_last_error;

	float changes = 0;
	if (error != last_error)
		Integrators.push_back(change);
	else
		Integrators.push_back(last_change);

	size_t COUNT = 8;
	if (Integrators.size() > COUNT)
		Integrators.erase(Integrators.begin());

	for (size_t i = 0; i < Integrators.size();i++)
		changes += Integrators[i] / COUNT * (i + 1);
	changes = changes / Integrators.size();
	I_value = changes * Ki;

//	D_value = Kd * ( error - Derivator);
	D_value = Kd * change;
	Derivator = error;

	//Integrator = Integrator + error * Ki;// / 200.0;

	if (Integrator > Integrator_max)
		Integrator = Integrator_max;
	else if (Integrator < Integrator_min)
		Integrator = Integrator_min;

	last_last_error = last_error;
	last_error = error;
	last_value = current_value;

	return P_value + I_value + D_value;
}
void PID_Y::setPoint(float set_point){
	//Initilize the setpoint of PID
	this->set_point = set_point;
	Integrator = 0;
	Derivator = 0;
	Integrators.clear();

	last_last_error = 0;
	last_error = 0;
	last_value = 0;
	error = 0;
}
void PID_Y::reset()
{
	Integrator = 0;
	Derivator = 0;
}
float PID_Y::getKp()
{
	return P_value;
}
float PID_Y::getKi()
{
	return I_value;
}
float PID_Y::getKd()
{
	return D_value;
}
