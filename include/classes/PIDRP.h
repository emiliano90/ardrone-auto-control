#ifndef PIDRP_H_
#define PIDRP_H_

#include <vector>

class PID_RP {
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
	PID_RP(float P, float I, float D, int Derivator, float Integrator,
		            int Integrator_max, float P_limit, float set_point, float power);

	float update(float current_value, float change, long dt);
	float update2(float current_value);
	float update3(float current_value, float change);
	void setPoint(float set_point);
	void integratorIncrease(float newError, float max, float change);
	void reset();
	float getKp();
	float getKi();
	float getKd();

	virtual ~PID_RP();
};

#endif /* PIDRP_H_ */
