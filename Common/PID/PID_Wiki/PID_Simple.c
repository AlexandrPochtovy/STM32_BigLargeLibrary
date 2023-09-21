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

#include "PID_Simple.h"

void PidSimple_Init(float kp, float ki, float kd, int32_t dT, pidS_t *pid) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->a[2] = pid->Kd * 1000 / dT;
    pid->a[1] = -pid->Kp - 2 * pid->a[2];
    pid->a[0] = pid->Kp + pid->Ki * dT + pid->a[2];
    pid->e[2] = 0.0;
    pid->e[1] = 0.0;
    pid->e[0] = 0.0;
    pid->out = 0.0;
}

int32_t PidSimple_Processing(float sp, float actual, int32_t min, int32_t max, pidS_t *pid) {
    pid->e[2] = pid->e[1];
    pid->e[1] = pid->e[0];
    pid->e[0] = sp - actual;
    pid->out = pid->out + pid->a[0] * pid->e[0] + pid->a[1] * pid->e[1] + pid->a[2] * pid->e[2];
    //out limit
    if (pid->out < min) { pid->out = min; }
    else if (pid->out > max) { pid->out = max; }
    return (int32_t)pid->out;
}

void PidFiltered_Init(float kp, float ki, float kd, uint8_t N, int32_t dT, pidF_t *pid) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->a = kp + ki * dT / 1000;
    pid->e[2] = 0;
    pid->e[1] = 0;
    pid->e[0] = 0;
    pid->out = 0;  //the current value of the actuator
    pid->ad[0] = kd *1000 / dT;
    pid->ad[1] = -2.0 * pid->ad[0];
    float tau = kd / (kp * N); // IIR filter time constant
    pid->alpha = dT / (2 * 1000 * tau);
    pid->d[0] = 0;
    pid->d[1] = 0;
    pid->fd[0] = 0;
    pid->fd[1] = 0;
    pid->out = 0;
}

int32_t PidFiltered_Processing(float sp, float act, int32_t min, int32_t max, pidF_t *pid) {
    pid->e[2] = pid->e[1];
    pid->e[1] = pid->e[0];
    pid->e[0] = sp - act;
    pid->out = pid->out + pid->a * pid->e[0] - pid->kp * pid->e[1];// PI
    pid->d[1] = pid->d[0];// Filtered D
    pid->d[0] = pid->ad[0] * pid->e[0] + pid->ad[1] * pid->e[1] + pid->ad[0] * pid->e[2];
    pid->fd[1] = pid->fd[0];
    pid->fd[0] = ((pid->alpha) / (pid->alpha + 1)) * (pid->d[0] + pid->d[1]) - ((pid->alpha - 1) / (pid->alpha + 1)) * pid->fd[1];
    pid->out = pid->out + pid->fd[0];
    if (pid->out < min) {
    	pid->out = min;
    }
    else if (pid->out > max) {
    	pid->out = max;
    }
    return (int32_t)pid->out;
}
