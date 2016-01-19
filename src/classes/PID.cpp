#include "include/classes/PID.h"

PID::PID(float P = 1.0, float I = 0.0, float D = 10.0, int Derivator = 0, int Integrator = 0,
        int Integrator_max = 200, int Integrator_min = -200, float set_point = 0.0, float power = 1.0){

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

}

PID::~PID() {

}

float PID::update(float current_value){
//Calculate PID output value for given reference input and feedback
	this->error = this->set_point - current_value;

	P_value = Kp * error;
	float change;
	if (last_value >= current_value)
		change = error - last_error;
	else
		change = 0;


	if(error > 0)
		I_value = Integrator * Ki;
	else
		I_value = (Integrator * Ki)*0.5;


	D_value = Kd * ( error - Derivator);
	D_value = Kd * change;
	Derivator = error;

	Integrator = Integrator + error / 200;

	if (Integrator > Integrator_max)
		Integrator = Integrator_max;
	else if (Integrator < Integrator_min)
		Integrator = Integrator_min;

	last_error = error;
	last_value = current_value;

	return P_value + I_value + D_value;
}
void PID::setPoint(float set_point){
	//Initilize the setpoint of PID
	this->set_point = set_point;
	Integrator = 0;
	Derivator = 0;
}
