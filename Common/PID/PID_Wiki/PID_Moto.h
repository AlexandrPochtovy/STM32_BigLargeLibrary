#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "Function/Function.h"

#ifndef _PID_MOTO_H_
#define _PID_MOTO_H_

/**********************************************************************
*                       TYPEDEF's PID                                 * 
***********************************************************************/
/*****************************************************************
  * @brief structure for PID controller with simple filtration:
  * @param Kp 		- the proportional tuning constant, multiplied with SCALING_FACTOR
    @param Ki 		- the Integral tuning constant, multiplied with SCALING_FACTOR
    @param Kd 		- the Derivative tuning constant, multiplied with SCALING_FACTOR
	@param alpfa	- factor α for an exponential moving average filter
    @param intgErr 	- inegral of error between set-point and actual values
    @param oldErrFilter 	- previous error between set-point and actual values
  * @retval out 	- output value  
  */
typedef struct PID_M {
    float kp;
	float ki;
	float kd;
	float alpha;
    float intgErr;
    float oldErrFilter;
	float out;
} PID_M_t;

/*****************************************************************
  * @brief structure for PID controller with good derivative filtering:
  * @param Kp 	 - the proportional tuning constant, multiplied with SCALING_FACTOR
    @param Ki 	 - the Integral tuning constant, multiplied with SCALING_FACTOR
    @param Kd 	 - the Derivative tuning constant, multiplied with SCALING_FACTOR
	@param a[3]	 - factor α for filter
	@param ku[2] - factor u for filter
	@param ke[3] - factor e for filter
	@param e[3]	 - error's value history
	@param u[3]	 - output's value history
  */
typedef struct PID_MF {
	float kp;
	float ki;	
	float kd;
  uint8_t N;
	float a[2];
	float ku[2];
	float ke[3];
	float e[3];
	float u[3];
	float kp_mem;
	float ki_mem;
	float kd_mem;
    } PID_MF_t;

/*****************************************************************
  * @brief init the PID controller: calculate coefficients and constants
  * @param kp - the proportional tuning constant
    @param ki - the Integral tuning constant
    @param kd - the Derivative tuning constant
	@param fc  Cutoff frequency of derivative filter in Hz
    @param dT - time between PID calcilation in msec
    @param *pid - pointer for pidS_t pid structure
  * @retval none
  */
void PID_MotoInit(float kp, float ki, float kd, float fc, uint32_t dT, PID_M_t *pid);

/*****************************************************************
  * @brief calculate the PID controller
  * @param sp - set-point value
    @param act - actual value, i.e. feedback
    @param min - minimum output value
    @param max - maximum output value
    @param dT - time between pid calculation
    @param *pid - pointer for pidS_t pid structure
  * @retval control value from pid
  */
size_t PID_MotoCalc(float sp, float act, size_t min, size_t max, uint32_t dT, PID_M_t *pid);

/*****************************************************************
  * @brief init the PID controller: calculate coefficients and constants
  * @param kp - the proportional tuning constant
    @param ki - the Integral tuning constant
    @param kd - the Derivative tuning constant
    @param N - number of samples for IIR filter
    @param dT - time between PID calcilation in msec
    @param *pid - pointer for pidS_t pid structure
  * @retval none
  */
void PID_MotoFilteredInit(float kp, float ki, float kd, uint8_t N, uint32_t dT, PID_MF_t *pid);

/*****************************************************************
  * @brief calculate the PID controller
  * @param sp - set-point value
    @param act - actual value, i.e. feedback
    @param min - minimum output value
    @param max - maximum output value
    @param *pid - pointer for pidS_t pid structure
  * @retval control value from pid
  */
size_t PID_MotoFilteredCalc(float sp, float act, uint32_t dT, size_t min, size_t max, PID_MF_t *pid);

#endif
