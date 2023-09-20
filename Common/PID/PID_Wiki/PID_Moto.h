#include <stdint.h>
#include <math.h>

#ifndef _PID_MOTO_H_
#define _PID_MOTO_H_
// Very basic, mostly educational PID controller with derivative filter.
// @param  Kp  Proportional gain   @f$ K_p @f$
// @param  Ki  Integral gain       @f$ K_i @f$
// @param  Kd  Derivative gain     @f$ K_d @f$
// @param  fc  Cutoff frequency    @f$ f_c @f$ of derivative filter in Hz
// @param  dT  Controller sampling time    @f$ T_s @f$ in seconds
// The derivative filter can be disabled by setting `fc` to zero.
typedef struct PID_M {
    float kp;		
	float ki;		
	float kd;		
	float alpha;	
	uint32_t dT;		
    float intgErr;	
    float oldEf;	
	float out;      // output value
} PID_M_t;

typedef struct PID_MF {
	float kp;	// proportional gain
	float ki;	// integral gain	
	float kd;	// derivative gain
	float a[3];
	float ku[2];
	float ke[3];
	float e[3];	// errors variables used in PID computation
	float u[3];	// filtered value
	uint32_t dT;	// sampling time PID, msec	
    } PID_MF_t;

void PID_MotoInit(float kp, float ki, float kd, float fc, uint32_t dT, PID_M_t *pid);

uint16_t PID_MotoCalc(float sp, float act, int32_t min, int32_t max, PID_M_t *pid);

void PID_MotoFilteredInit(float kp, float ki, float kd, uint8_t N, uint32_t dT, PID_MF_t *pid);

int32_t PID_MotoFilteredCalc(float sp, float act, int32_t min, int32_t max, PID_MF_t *pid);

#endif
