/*********************************************************************************
  @author:  https://www.scilab.org/discrete-time-pid-controller-implementation
  
  @details: Implementation for STM32 Aliaksandr Pachtovy<alex.mail.prime@gmail.com>
                    https://github.com/AlexandrPochtovy

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

 * @headerfile PID_W.h
 * Created on: Sep 14, 2023
*********************************************************************************/

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
  @brief structure for PID controller with simple filtration:
  @param Kp 		- the proportional tuning constant, multiplied with SCALING_FACTOR
  @param Ki 		- the Integral tuning constant, multiplied with SCALING_FACTOR
  @param Kd 		- the Derivative tuning constant, multiplied with SCALING_FACTOR
  @param alpfa	- factor α for an exponential moving average filter
  @param intgErr 	- inegral of error between set-point and actual values
  @param oldErrFilter 	- previous error between set-point and actual values
  @retval out 	- output value
  */
typedef struct PID_M {
  float kp;
  float ki;
  float kd;
  float alpha;
  float errFilter;
  float intgErr;
  float oldErrFilter;
  float out;
  } PID_M_t;

/*****************************************************************
  @brief structure for PID controller with good derivative filtering:
  @param Kp 	 - the proportional tuning constant, multiplied with SCALING_FACTOR
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
  uint32_t dT;
  uint8_t N;
  float a[2];
  float ku1;
  float ku2;
  float ke[3];
  float e[3];
  float u[3];
  float kp_mem;
  float ki_mem;
  float kd_mem;
  uint8_t N_mem;
  } PID_MF_t;

/*****************************************************************
  @brief Filter for actual value
  @param act - current value of the input parameter
  @param last - previous filtered input parameter value 
  @param k - the smoothing factor
  @retval smoothing value

  @example float filtered = MovingAverageFilter(sensorValue, filtered, 0.75);
  */
float MovingAverageFilter(float act, float last, float k);


/*****************************************************************
  @brief init the PID controller: calculate coefficients and constants
  @param kp - the proportional tuning constant
  @param ki - the Integral tuning constant
  @param kd - the Derivative tuning constant
  @param fc  Cutoff frequency of derivative filter in Hz
  @param dT - time between PID calcilation in msec
  @param *pid - pointer for pidS_t pid structure
  @retval none
  */
void PID_MotoInit(float kp, float ki, float kd, uint32_t dT, PID_M_t* pid);

/*****************************************************************
  @brief calculate the PID controller
  @param sp - set-point value
  @param act - actual value, i.e. feedback
  @param min - minimum output value
  @param max - maximum output value
  @param dT - time between pid calculation
  @param *pid - pointer for pidS_t pid structure
  @retval control value from pid
  */
size_t PID_MotoProcessing(float sp, float act, size_t min, size_t max, uint32_t dT, PID_M_t* pid);

/*****************************************************************
  @brief init the PID controller: calculate coefficients and constants
  @param kp - the proportional tuning constant
  @param ki - the Integral tuning constant
  @param kd - the Derivative tuning constant
  @param N - number of samples for IIR filter
  @param dT - time between PID calcilation in msec
  @param *pid - pointer for pidS_t pid structure
  @retval none
  */
void PID_MotoFilteredInit(float kp, float ki, float kd, uint8_t N, uint32_t dT, PID_MF_t* pid);

/*****************************************************************
  @brief calculate the PID controller
  @param sp - set-point value
  @param act - actual value, i.e. feedback
  @param min - minimum output value
  @param max - maximum output value
  @param *pid - pointer for pidS_t pid structure
  @retval control value from pid
  */
size_t PID_MotoFilteredProcessing(float sp, float act, size_t min, size_t max, PID_MF_t* pid);

#endif
