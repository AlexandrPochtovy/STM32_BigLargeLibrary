#include "PID_Moto.h"

void PID_MotoInit(float kp, float ki, float kd, float fc, uint32_t dT, PID_M_t *pid) {
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->dT = dT;
	float fn = fc * dT / 1000;
	// Compute the weight factor α for an exponential moving average filter
    // with a given normalized cutoff frequency `fn`.
    if (fn <= 0) {
        pid->alpha = 1;
	} else {
		// α(fₙ) = cos(2πfₙ) - 1 + √( cos(2πfₙ)² - 4 cos(2πfₙ) + 3 )
    	float c = cosf(2 * M_PI * fn);
    	pid->alpha = c - 1 + sqrt(c * c - 4 * c + 3);
	}
	pid->intgErr = 0;
	pid->oldEf = 0;
}

uint16_t PID_MotoCalc(float sp, float act, int32_t min, int32_t max, PID_M_t *pid) {
	float error = sp - act;// e[k] = r[k] - y[k], error between setpoint and true position
	float ef = pid->alpha * error + (1 - pid->alpha) * pid->oldEf;// e_f[k] = α e[k] + (1-α) e_f[k-1], filtered error
	float derivative = (ef - pid->oldEf) / pid->dT;// e_d[k] = (e_f[k] - e_f[k-1]) / Tₛ, filtered derivative
	float actInt = pid->intgErr + error * pid->dT;// e_i[k+1] = e_i[k] + Tₛ e[k], integral
	pid->out = pid->kp * error + pid->ki * pid->intgErr + pid->kd * derivative;// PID formula: u[k] = Kp e[k] + Ki e_i[k] + Kd e_d[k], control signal
    if (pid->out < min) { 
		pid->out = min; 
		}
    else if (pid->out > max) { 
		pid->out = max; 
		}
	//else {// Anti-windup
		pid->intgErr = actInt;
	//}
	pid->oldEf = ef;// store the state for the next iteration
    return (int32_t)pid->out;
}

void PID_MotoFilteredInit(float kp, float ki, float kd, uint8_t N, uint32_t dT, PID_MF_t *pid) {
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->dT = dT;
	float tmp = (float)(N * dT) / 1000.0f;
	pid->a[0] = 1 + tmp;
	pid->a[1] = -(2.0f + tmp);
	pid->a[2] = 1.0f;
	float b0 = kp * pid->a[0] + ki * dT * pid->a[0] / 1000 + kd * N;
	float b1 = -(kp * (2 + tmp) + ki * dT / 1000 + 2 * kd * N);
	float b2 = kp + kd * N;
	pid->ku[0] = pid->a[1] / pid->a[0];
	pid->ku[1] = 1.0f / pid->a[0];
	pid->ke[0] = b0 / pid->a[0];
	pid->ke[1] = b1 / pid->a[0];
	pid->ke[2] = b2 / pid->a[0];
}

int32_t PID_MotoFilteredCalc(float sp, float act, int32_t min, int32_t max, PID_MF_t *pid) {
	pid->e[2] = pid->e[1];
	pid->e[1] = pid->e[0];
	pid->u[2] = pid->u[1];
	pid->u[1] = pid->u[0]; // update variables
	pid->e[0] = sp - act;           // compute new error
	pid->u[0] = -pid->ku[0] * pid->u[1] - pid->ku[1] * pid->u[2] + pid->ke[0] * pid->e[0] + pid->ke[1] * pid->e[1] + pid->ke[2] * pid->e[2]; // eq (12)
	if (pid->u[0] > max) {// limit to DAC or PWM range
		pid->u[0] = max;
	} else if (pid->u[0] < min) {
		pid->u[0] = min;
	}
	return (int32_t)pid->u[0];
}

