#include "include/classes/PIDZ.h"

#include <iostream>

#include <stdio.h>
//using namespace std;
PID_Z::PID_Z(float P = 1.0, float I = 0.0, float D = 10.0, int Derivator = 0, float Integrator = 0,
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

PID_Z::~PID_Z() {

}

float PID_Z::update(float current_value, long dt){
//Calculate PID output value for given reference input and feedback
	error = set_point - current_value;
	printf("error:  %.2f \n",  error);

	float change = error - last_error; //si es positivo me alejo, si es negativo me acerco
	//error = set_point - current_value;

	P_value = Kp * error;
	D_value = Kd * change / dt;//usp el del robot

//	integratorIncrease(error * dt, Integrator_max, change);

//	I_value = (float)Integrator * (float)Ki;

	last_last_error = last_error;
	last_error = error;
	last_value = current_value;
	last_change = change;

	float total = P_value + D_value;

	if (total > P_limit)
		total = P_limit;
	else if (total < -P_limit)
		total = -P_limit;

	return total;
}




void PID_Z::setPoint(float set_point){
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
void PID_Z::integratorIncrease(float newError, float max, float change)
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
void PID_Z::reset()
{
	Integrator = 0;
	Derivator = 0;
}
float PID_Z::getKp()
{
	return P_value;
}
float PID_Z::getKi()
{
	return I_value;
}
float PID_Z::getKd()
{
	return D_value;
}
