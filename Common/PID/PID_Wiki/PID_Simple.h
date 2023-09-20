/*********************************************************************************
  Original author:  Aliaksandr Pachtovy<alex.mail.prime@gmail.com>
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

 * PID_W.h
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
typedef struct pidSimple {
    float Kp;       // the Proportional tuning constant, multiplied with SCALING_FACTOR
    float Ki;	    // the Integral tuning constant, multiplied with SCALING_FACTOR
    float Kd;	    // the Derivative tuning constant, multiplied with SCALING_FACTOR
    float a[3];     // coefficients for calculate pid
    float e[3];     // the data storage values for recurrent calculating
    float out;      // output value
} pidS_t;

typedef struct pidFiltered {
    float kp;       // the Proportional tuning constant, multiplied with SCALING_FACTOR
    float ki;	    // the Integral tuning constant, multiplied with SCALING_FACTOR
    float kd;	    // the Derivative tuning constant, multiplied with SCALING_FACTOR
    float a;        // coefficients for calculate pid
    float ad[2];    // coefficients for calculate pid
    float alpha;
    float e[3];     // the data storage values for recurrent calculating
    float d[2];     // derivative value act & prev
    float fd[2];    // filtered derivative act & prev
    float out;      // output value
} pidF_t;

void PidSimple_Init(float kp, float ki, float kd, uint8_t N, int32_t dT, pidS_t *pid);

int32_t PidSimple_Processing(float sp, float actual, int32_t min, int32_t max, pidS_t *pid);

void PidFiltered_Init(float kp, float ki, float kd, uint8_t N, int32_t dT, pidF_t *pid);

int32_t PidFiltered_Processing(float sp, float actual, int32_t min, int32_t max, pidF_t *pid);


#ifdef __cplusplus
}
#endif

#endif
