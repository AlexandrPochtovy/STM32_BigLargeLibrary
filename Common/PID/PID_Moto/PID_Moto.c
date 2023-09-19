#include "PID_Moto.h"

void PID_Init(float kp, float ki, float kd, float fc, float Ts, PID_t *pid) {
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->Ts = Ts;
	float fn = fc * Ts;
	// Compute the weight factor α for an exponential moving average filter
    // with a given normalized cutoff frequency `fn`.
    if (fn <= 0) {
        pid->alpha = 1;
	} else {
		// α(fₙ) = cos(2πfₙ) - 1 + √( cos(2πfₙ)² - 4 cos(2πfₙ) + 3 )
    	float c = cosf(2 * M_PI * fn);
    	pid->alpha = c - 1 + sqrt(c * c - 4 * c + 3);
	}
	pid->integral = 0;
	pid->old_ef = 0;
}

uint16_t PID_Calculate(float sp, float act, uint16_t min, uint16_t max, PID_t *pid) {
	// e[k] = r[k] - y[k], error between setpoint and true position
	float error = sp - act;
	// e_f[k] = α e[k] + (1-α) e_f[k-1], filtered error
	float ef = pid->alpha * error + (1 - pid->alpha) * pid->old_ef;
	// e_d[k] = (e_f[k] - e_f[k-1]) / Tₛ, filtered derivative
	float derivative = (ef - pid->old_ef) / pid->Ts;
	// e_i[k+1] = e_i[k] + Tₛ e[k], integral
	float new_integral = pid->integral + error * pid->Ts;
	// PID formula: u[k] = Kp e[k] + Ki e_i[k] + Kd e_d[k], control signal
	float control_u = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
	uint16_t out = (uint16_t)control_u;
    if (out >= max) {// Clamp the output
		out = max;
	}
	else if (out <= min) {
    	out = min;
	}
	else {// Anti-windup
		pid->integral = new_integral;
	} 
	pid->old_ef = ef;// store the state for the next iteration
    return out;// return the control signal
}

uint16_t PID_Alt(float act, float sp, float min, float max, PID_alt_t *pid) {
//                    -- the following coefficients must be recomputed if
//                    --- any parameter above is changed by user
float a0 = (1+pid->N * pid->Ts);
float a1 = -(2 + pid->N * pid->Ts);
float b0 = pid->Kp * (1+ pid->N * pid->Ts) + pid->Ki * pid->Ts * (1 + pid->N * pid->Ts) + pid->Kd * pid->N;
float b1 = -(pid->Kp * (2 + pid->N * pid->Ts) + pid->Ki * pid->Ts + 2 * pid->Kd * pid->N);
float b2 = pid->Kp + pid->Kd * pid->N;
float ku1 = a1/a0;
float ku2 = 1.0f/a0;
float ke0 = b0/a0;
float ke1 = b1/a0;
float ke2 = b2/a0;
pid->e[2] = pid->e[1];
pid->e[1] = pid->e[0];
pid->u[2] = pid->u[1];
pid->u[1] = pid->u[0]; // update variables
pid->e[0] = sp - act;           // compute new error
pid->u[0] = -ku1 * pid->u[1] - ku2 * pid->u[2] + ke0 * pid->e[0] + ke1 * pid->e[1] + ke2 * pid->e[2]; // eq (12)
if (pid->u[0] >= max) pid->u[0] = max;  // limit to DAC or PWM range
if (pid->u[0] <= min) pid->u[0] = min;
return (uint16_t)pid->u[0];
}

