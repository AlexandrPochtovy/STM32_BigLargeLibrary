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

#include <stdint.h>
#include <stddef.h>
#include "Function.h"

typedef struct pid
{
    int32_t Kp;	//the Proportional tuning constant, multiplied with SCALING_FACTOR
    int32_t Ki;	//the Integral tuning constant, multiplied with SCALING_FACTOR
    int32_t Kd;	//the Derivative tuning constant, multiplied with SCALING_FACTOR
    int32_t e[3];       //the data storage values for recurrent calculating
    uint8_t N;          
    float d0;
    float d1;
    float fd0;
    float fd1;
    int32_t out;        //output value
} pid_t;

void Pid_Init(pid_t *pid, int32_t kp, int32_t ki, int32_t kd, int32_t dt);

int32_t PidProcessingSimple(pid_t *pid, int32_t measured_value, int32_t setpoint, int32_t min, int32_t max, uint32_t dt);

int32_t PidProcessingFiltered(pid_t *pid, int32_t measured_value, int32_t setpoint, int32_t min, int32_t max, uint32_t dt);