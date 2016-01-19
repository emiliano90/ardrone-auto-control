#ifndef PIDZ_H_
#define PIDZ_H_

#include <vector>

class PID_Z {
	float Kp;
	float Ki;
	float Kd;
	int Derivator;
	std::vector<float> Integrators;
	float Integrator;
	int Integrator_max;
	float P_limit;
	float set_point;
	float power;

	float last_last_error;
	float last_error;
	float last_value;
	float last_change;
	float error;

	float P_value;
	float I_value;
	float D_value;
public:
	PID_Z(float P, float I, float D, int Derivator, float Integrator,
		            int Integrator_max, float P_limit, float set_point, float power);

	float update(float current_value, long dt);
	void setPoint(float set_point);
	void integratorIncrease(float newError, float max, float change);
	void reset();
	float getKp();
	float getKi();
	float getKd();

	virtual ~PID_Z();
};

#endif /* PIDRP_H_ */
