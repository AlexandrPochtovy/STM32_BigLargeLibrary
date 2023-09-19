#include <stdint.h>
#include <math.h>

#ifndef _PID_MOTO_H_
#define _PID_MOTO_H_
/// Very basic, mostly educational PID controller with derivative filter.
/// @param  kp  Proportional gain   @f$ K_p @f$
/// @param  ki  Integral gain       @f$ K_i @f$
/// @param  kd  Derivative gain     @f$ K_d @f$
/// @param  fc  Cutoff frequency    @f$ f_c @f$ of derivative filter in Hz
/// @param  Ts  Controller sampling time    @f$ T_s @f$ in seconds
/// The derivative filter can be disabled by setting `fc` to zero.
typedef struct PID {
    float kp;		
	float ki;		
	float kd;		
	float alpha;	
	float Ts;		
    float integral;	
    float old_ef;	
} PID_t;

typedef struct PID_alt {
	//                    -- these parameters should be user-adjustable 
  float Kp;		// proportional gain
	float Ki;	//          integral gain	
	float Kd; //          derivative          gain
	uint8_t N;		//          filter coefficients
	float e[3];		// variables used in PID computation
	float u[3];
	float Ts;		// This     must match      actual sampling time PID	
    } PID_alt_t;

void PID_Init(float kp, float ki, float kd, float fc, float Ts, PID_t *pid);
    /// Update the controller with the given position measurement `meas_y` and 
    /// return the new control signal.
uint16_t PID_Calculate(float sp, float act, uint16_t min, uint16_t max, PID_t *pid);

uint16_t PID_Alt(float act, float sp, float min, float max, PID_alt_t *pid);

#endif
