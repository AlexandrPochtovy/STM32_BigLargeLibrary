#include "PID_Moto.h"

void PID_MotoInit(float kp, float ki, float kd, uint32_t dT, PID_M_t *pid) {
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->alpha = 0.5;
	pid->intgErr = 0.0f;
	pid->errFilter = 0.0f;
	pid->oldErrFilter = 0.0f;
}

size_t PID_MotoProcessing(float sp, float act, size_t min, size_t max, uint32_t dT, PID_M_t *pid) {
	float error = sp - act;  // e[k] = r[k] - y[k], error between setpoint and true position
	pid->errFilter = MovingAverageFilter(error, pid->oldErrFilter, pid->alpha);
	// e_f[k] = α e[k] + (1-α) e_f[k-1], filtered error
	float derivative;
	if (dT) {
		derivative = (pid->errFilter - pid->oldErrFilter) * 1000.0f / dT;
		// e_d[k] = (e_f[k] - e_f[k-1]) / Tₛ, filtered derivative
	}
	else {
		derivative = 0.0f;
	}
	float actInt = pid->intgErr + pid->errFilter * dT / 1000.0f;
	// e_i[k+1] = e_i[k] + Tₛ e[k], integral
	pid->out = pid->kp * pid->errFilter + pid->ki * pid->intgErr + pid->kd * derivative;
	// PID formula: u[k] = Kp e[k] + Ki e_i[k] + Kd e_d[k], control signal
	if ((pid->out <= min) || (sp < 0.001f)) {
		pid->out = min;
	}
	else if (pid->out >= max) {
		pid->out = max;
	}
	else {  // Anti-windup
		pid->intgErr = actInt;
	}
	pid->oldErrFilter = pid->errFilter;  // store the state for the next iteration
	return (size_t)pid->out;
}

void PID_MotoFilteredInit(float kp, float ki, float kd, uint8_t N, uint32_t dT, PID_MF_t *pid) {
	float tmp = N * dT / 1000.0f;
	pid->a[0] = (1 + tmp);
	pid->a[1] = -(2 + tmp);
	float b0 = kp * pid->a[0] + ki * dT * pid->a[0] / 1000.0f + kd * N;
	float b1 = -(kp * (-pid->a[1]) + ki * dT / 1000.0f + 2 * kd * N);
	float b2 = kp + kd * N;
	pid->ku1 = pid->a[1] / pid->a[0];
	pid->ku2 = 1.0f / pid->a[0];
	pid->ke[0] = b0 / pid->a[0];
	pid->ke[1] = b1 / pid->a[0];
	pid->ke[2] = b2 / pid->a[0];
	pid->kp_mem = pid->kp = kp;
	pid->ki_mem = pid->ki = ki;
	pid->kd_mem = pid->kd = kd;
	pid->N = N;
	pid->dT = dT;
}

size_t PID_MotoFilteredProcessing(float sp, float act, size_t min, size_t max, PID_MF_t *pid) {
	if ((pid->kp != pid->kp_mem) || (pid->ki != pid->ki_mem) || (pid->kd != pid->kd_mem)
	    || (pid->N != pid->N_mem)) {
		float tmp = pid->N * pid->dT / 1000.0f;
		pid->a[0] = (1 + tmp);
		pid->a[1] = -(2 + tmp);
		float b0 = pid->kp * pid->a[0] + pid->ki * pid->dT * pid->a[0] / 1000.0f + pid->kd * pid->N;
		float b1 = -(pid->kp * (-pid->a[1]) + pid->ki * pid->dT / 1000.0f + 2 * pid->kd * pid->N);
		float b2 = pid->kp + pid->kd * pid->N;
		pid->ku1 = pid->a[1] / pid->a[0];
		pid->ku2 = 1.0f / pid->a[0];
		pid->ke[0] = b0 / pid->a[0];
		pid->ke[1] = b1 / pid->a[0];
		pid->ke[2] = b2 / pid->a[0];
		pid->kp_mem = pid->kp;
		pid->ki_mem = pid->ki;
		pid->kd_mem = pid->kd;
		pid->N = pid->N_mem;
	}
	else {
		pid->e[2] = pid->e[1];
		pid->e[1] = pid->e[0];
		pid->u[2] = pid->u[1];
		pid->u[1] = pid->u[0];  // update variables
		pid->e[0] = sp - act;  // compute new error
		pid->u[0] = pid->ke[0] * pid->e[0] + pid->ke[1] * pid->e[1] + pid->ke[2] * pid->e[2]
		    - pid->ku1 * pid->u[1] - pid->ku2 * pid->u[2];  // eq (12)
		if (pid->u[0] > max) {
			pid->u[0] = max;
		}  // limit to DAC or PWM range
		else if ((pid->u[0] < min) || (sp <= 0.001f)) {
			pid->u[0] = min;
		}
	}
	return (size_t)pid->u[0];  // sent    to output
}

float MovingAverageFilter(float act, float last, float k) {
	return last + (act - last) * k;
	}