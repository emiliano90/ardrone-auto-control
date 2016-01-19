#ifndef PIDY_H_
#define PIDY_H_

#include <vector>

class PID_Y {
	float Kp;
	float Ki;
	float Kd;
	int Derivator;
	std::vector<float> Integrators;
	float Integrator;
	int Integrator_max;
	int Integrator_min;
	float P_limit;
	float set_point;
	float power;

	float last_last_error;
	float last_error;
	float last_value;
	float error;

	float P_value;
	float I_value;
	float D_value;
public:
	PID_Y(float P, float I, float D, int Derivator, float Integrator,
	            int Integrator_max, int Integrator_min, float P_limit, float set_point, float power);

	float update(float current_value);
	float update2(float current_value);
	void setPoint(float set_point);
	void reset();
	float getKp();
	float getKi();
	float getKd();

	virtual ~PID_Y();
};

#endif /* PIDRP_H_ */
