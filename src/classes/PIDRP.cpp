#include "include/classes/PIDRP.h"

#include <iostream>

#include <stdio.h>
//using namespace std;
PID_RP::PID_RP(float P = 1.0, float I = 0.0, float D = 10.0, int Derivator = 0, float Integrator = 0,
        int Integrator_max = 250, float P_limit = 12, float set_point = 0.0, float power = 1.0){

	this->Kp = P;
	this->Ki = I;
	this->Kd = D;
	this->Derivator = Derivator;
	this->power = power;
	this->Integrator = Integrator;
	this->Integrator_max = Integrator_max;
	this->last_error = 0.0;
	this->last_value = 0.0;
	this->set_point = set_point;
	this->error =0.0;
	this->P_limit = P_limit;

	last_change = 0.0;
}

PID_RP::~PID_RP() {

}

float PID_RP::update(float current_value, float change2, long dt){
//Calculate PID output value for given reference input and feedback
	error = current_value;
	float change = error - last_error; //si es positivo me alejo, si es negativo me acerco
	//error = set_point - current_value;


//	float last_change = last_error - last_last_error;

	//aumento Kp si me alejo
	P_value = Kp * error;
/*
	if (P_value > P_limit)
		P_value = P_limit;
	else if (P_value < -P_limit)
		P_value = -P_limit;
*/
	/*
	float changes = 0;
	if (error != last_error)
		Integrators.push_back(change);
	else
		Integrators.push_back(last_change);

	size_t COUNT = 1;
	if (Integrators.size() > COUNT)
		Integrators.erase(Integrators.begin());
	for (size_t i = 0; i < Integrators.size();i++)
		changes += Integrators[i];
	changes = changes / Integrators.size();
*/

	float changes = 0;
	Integrators.push_back(change / dt);

	/*if (error != last_error)
		Integrators.push_back(change);
	else
		Integrators.push_back(last_change);
*/
	size_t COUNT = 2;
	if (Integrators.size() > COUNT)
		Integrators.erase(Integrators.begin());
	for (size_t i = 0; i < Integrators.size();i++)
		changes += Integrators[i];
	changes = changes / Integrators.size();

	D_value = Kd * change2;//usp el del robot

//	D_value = Kd * changes;//uso el de la camara
	//if(error < 50)
	//	D_value *= 1.2;
	integratorIncrease(error * dt, Integrator_max, changes);
//	D_value = Kd * (float)changes;


/*	Derivator = error;
	Integrator += error;

	if (Integrator > Integrator_max)
		Integrator = Integrator_max;
	else if(Integrator < Integrator_min)
		Integrator = Integrator_min;
*/
	I_value = (float)Integrator * (float)Ki;

	last_last_error = last_error;
	last_error = error;
	last_value = current_value;
	last_change = change;

	float total = P_value + I_value + D_value;

	if (total > P_limit)
		total = P_limit;
	else if (total < -P_limit)
		total = -P_limit;

	return total;
}

float PID_RP::update2(float current_value){
//Calculate PID output value for given reference input and feedback
	this->error = this->set_point - current_value;

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

	D_value = Kd * ( error - Derivator);
	D_value = Kd * change;
	Derivator = error;

	//Integrator = Integrator + error * Ki;// / 200.0;

	if (Integrator > Integrator_max)
		Integrator = Integrator_max;
	else if (Integrator < -Integrator_max)
		Integrator = -Integrator_max;

	last_last_error = last_error;
	last_error = error;
	last_value = current_value;

	return P_value + I_value + D_value;
}

float PID_RP::update3(float current_value, float change){
//Calculate PID output value for given reference input and feedback
	error = current_value;
	printf("error:  %.2f \n",  error);

	//aumento Kp si me alejo
	P_value = Kp * error;

	if (P_value > P_limit)
		P_value = P_limit;
	else if (P_value < -P_limit)
		P_value = -P_limit;

	D_value = Kd * change;

	float total = P_value + D_value;
	if (total > P_limit)
	total = P_limit;
	else if (total < -P_limit)
		total = -P_limit;

	return total;
}

void PID_RP::setPoint(float set_point){
	//Initilize the setpoint of PID
	this->set_point = 0;
	Integrator = 0;
	Derivator = 0;
	Integrators.clear();

	last_last_error = set_point;
	last_error = set_point;
	last_value = 0;
	error = 0;
}
void PID_RP::integratorIncrease(float newError, float max, float change)
{

	if(newError < 0 && Integrator > 0)
		Integrator = std::max(0.0, Integrator + 14.0 * newError);
	else if(newError > 0 && Integrator < 0)
		Integrator = std::min(0.0, Integrator + 14.0 * newError);

	else if(change < 0 && Integrator > 0 && newError > 0)//si me estoy acercando
		Integrator = std::max(0.0f, float(Integrator - newError));
	else if(change > 0 && Integrator < 0 && newError < 0)//si me estoy acercando
		Integrator = std::min(0.0f, float(Integrator - newError));
	else
		Integrator += newError;

	if(Integrator > max)
		Integrator  = max;
	else if (Integrator < -max)
		Integrator  = -max;
}
void PID_RP::reset()
{
	Integrator = 0;
	Derivator = 0;
}
float PID_RP::getKp()
{
	return P_value;
}
float PID_RP::getKi()
{
	return I_value;
}
float PID_RP::getKd()
{
	return D_value;
}
