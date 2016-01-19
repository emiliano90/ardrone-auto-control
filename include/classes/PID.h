#ifndef PID_H_
#define PID_H_

class PID
{
		float Kp;
		float Ki;
		float Kd;
		int Derivator;
		int Integrator;
		int Integrator_max;
		int Integrator_min;
		float set_point;
		float power;

		float last_error;
		float last_value;
		float error;

		float P_value;
		float I_value;
		float D_value;

	public:
		PID(float P, float I, float D, int Derivator, int Integrator,
				int Integrator_max, int Integrator_min, float set_point,
				float power);

		float update(float current_value);
		void setPoint(float set_point);
		virtual ~PID();
};

#endif /* PID_H_ */
