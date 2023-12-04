/*********************************************************************************
  @author:  Aliaksandr Pachtovy<alex.mail.prime@gmail.com>
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

   https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Discrete_implementation

 * @headerfile PID_W.h
 * Created on: Sep 14, 2023
*********************************************************************************/
#ifndef _PID_W_H
#define _PID_W_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "Function/Function.h"

  /**********************************************************************
  *                       TYPEDEF's PID                                 *
  ***********************************************************************/
  /*****************************************************************
    * @brief structure for simple PID controller:
    * @param Kp - the proportional tuning constant, multiplied with SCALING_FACTOR
      @param Ki - the Integral tuning constant, multiplied with SCALING_FACTOR
      @param Kd - the Derivative tuning constant, multiplied with SCALING_FACTOR
      @param a[3] - coefficients for calculate pid
      @param e[3] - the data storage error values for calculating
    * @retval out - output value
    */
  typedef struct pidSimple {
    float Kp;
    float Ki;
    float Kd;
    size_t dT;
    float a[3];
    float e[3];
    float out;
    float Kp_mem;
    float Ki_mem;
    float Kd_mem;
    } pidS_t;

  /*****************************************************************
    * @brief structure for simple PID controller:
    * @param Kp - the proportional tuning constant, multiplied with SCALING_FACTOR
      @param Ki - the Integral tuning constant, multiplied with SCALING_FACTOR
      @param Kd - the Derivative tuning constant, multiplied with SCALING_FACTOR
      @param a - coefficients for calculate pid
      @param ad[2] - derivative calculation coefficient
      @param alpha - IIR filter coefficient
      @param e[3] - the data storage error values for calculating
      @param d[2] - derivative value act & prev
      @param fd[2] - filtered derivative act & prev
    * @retval out - output value
    */
  typedef struct pidFiltered {
    float kp;
    float ki;
    float kd;
    size_t dT;
    uint8_t N;
    float kp_mem;
    float ki_mem;
    float kd_mem;
    float a;
    float ad[2];
    float alpha;
    float e[3];
    float d[2];
    float fd[2];
    float out;
    } pidF_t;

  /*****************************************************************
    * @brief init the PID controller: calculate coefficients and constants
    * @param Kp - the proportional tuning constant
      @param Ki - the Integral tuning constant
      @param Kd - the Derivative tuning constant
      @param dT - time between PID calcilation in msec
      @param *pid - pointer for pidS_t pid structure
    * @retval none
    */
  void PidSimpleInit(float kp, float ki, float kd, size_t dT, pidS_t* pid);

  /*****************************************************************
    * @brief calculate the PID controller
    * @param sp - set-point value
      @param act - actual value, i.e. feedback
      @param min - minimum output value
      @param max - maximum output value
      @param *pid - pointer for pidS_t pid structure
    * @retval control value from pid
    */
  size_t PidSimpleProcessing(float sp, float act, size_t min, size_t max, pidS_t* pid);

  /*****************************************************************
    * @brief init the PID controller: calculate coefficients and constants
    * @param Kp - the proportional tuning constant
      @param Ki - the Integral tuning constant
      @param Kd - the Derivative tuning constant
      @param N - number of samples for IIR filter
      @param dT - time between PID calcilation in msec
      @param *pid - pointer for pidS_t pid structure
    * @retval none
    */
  void PidFilteredInit(float kp, float ki, float kd, uint8_t N, size_t dT, pidF_t* pid);

  /*****************************************************************
    * @brief calculate the PID controller
    * @param sp - set-point value
      @param act - actual value, i.e. feedback
      @param min - minimum output value
      @param max - maximum output value
      @param *pid - pointer for pidS_t pid structure
    * @retval control value from pid
    */
  size_t PidFilteredProcessing(float sp, float act, size_t dT, size_t min, size_t max, pidF_t* pid);


#ifdef __cplusplus
}
#endif

#endif
