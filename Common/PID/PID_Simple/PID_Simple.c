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

void PidSimpleInit(float kp, float ki, float kd, size_t dT, pidS_t* pid) {
    pid->Kp_mem = pid->Kp = kp;
    pid->Ki_mem = pid->Ki = ki;
    pid->Kd_mem = pid->Kd = kd;
    pid->dT = dT;
    if (pid->dT) {
        pid->a[2] = pid->Kd * 1000 / pid->dT;
        }
    else {
        pid->a[2] = 0;
        }
    pid->a[1] = -pid->Kp - 2 * pid->a[2];
    pid->a[0] = kp + ki * dT + pid->a[2];
    pid->e[2] = 0.0;
    pid->e[1] = 0.0;
    pid->e[0] = 0.0;
    pid->out = 0.0;
    }

size_t PidSimpleProcessing(float sp, float actual, size_t min, size_t max, pidS_t* pid) {
    if ((pid->Kp != pid->Kp_mem) || (pid->Ki != pid->Ki_mem) || (pid->Kd != pid->Kd_mem)) {
        if (pid->dT) {
            pid->a[2] = pid->Kd * 1000 / pid->dT;
            }
        else {
            pid->a[2] = 0;
            }
        pid->a[1] = -pid->Kp - 2 * pid->a[2];
        pid->a[0] = pid->Kp + pid->Ki * pid->dT + pid->a[2];
        pid->Kp_mem = pid->Kp;
        pid->Ki_mem = pid->Ki;
        pid->Kd_mem = pid->Kd;
        }
    pid->e[2] = pid->e[1];
    pid->e[1] = pid->e[0];
    pid->e[0] = sp - actual;
    pid->out = pid->out + pid->a[0] * pid->e[0] + pid->a[1] * pid->e[1] + pid->a[2] * pid->e[2];
    //out limit
    if ((pid->out < min) || (sp <= 0.001f)) { pid->out = min; }
    else if (pid->out > max) { pid->out = max; }
    return ( size_t )pid->out;
    }

void PidFilteredInit(float kp, float ki, float kd, uint8_t N, size_t dT, pidF_t* pid) {
    pid->kp_mem = pid->kp = kp;
    pid->ki_mem = pid->ki = ki;
    pid->kd_mem = pid->kd = kd;
    pid->dT = dT;
    pid->N = N;
    pid->a = kp + ki * dT / 1000;
    pid->e[2] = 0;
    pid->e[1] = 0;
    pid->e[0] = 0;
    if (dT) {
        pid->ad[0] = kd * 1000 / dT;
        pid->ad[1] = -2.0 * pid->ad[0];
        }
    else {
        pid->ad[0] = 0.0f;
        pid->ad[1] = 0.0f;
        }
    if (N) {
        float tau = kd / (kp * N); // IIR filter time constant
        pid->alpha = dT / (2 * 1000 * tau);
        }
    else {
        pid->alpha = 0.0f;
        }
    pid->d[0] = 0;
    pid->d[1] = 0;
    pid->fd[0] = 0;
    pid->fd[1] = 0;
    pid->out = 0;//the current value of the actuator
    }

size_t PidFilteredProcessing(float sp, float act, size_t dT, size_t min, size_t max, pidF_t* pid) {
    if ((pid->kp != pid->kp_mem) || (pid->ki != pid->ki_mem) ||
            (pid->kd != pid->kd_mem) || (pid->dT != dT) || (pid->N != pid->N_mem)) {
        pid->dT = dT;
        pid->kp_mem = pid->kp;
        pid->ki_mem = pid->ki;
        pid->kd_mem = pid->kd;
        pid->N_mem = pid->N;
        pid->a = pid->kp + pid->ki * pid->dT / 1000;
        if (pid->dT) {
            pid->ad[0] = pid->kd * 1000 / pid->dT;
            pid->ad[1] = -2.0 * pid->ad[0];
            }
        else {
            pid->ad[0] = 0.0f;
            pid->ad[1] = 0.0f;
            }
        if (pid->N) {
            float tau = pid->kd / (pid->kp * pid->N); // IIR filter time constant
            pid->alpha = pid->dT / (2 * 1000 * tau);
            }
        else {
            pid->alpha = 0.0f;
            }
        pid->d[0] = 0.0f;
        pid->d[1] = 0.0f;
        pid->fd[0] = 0.0f;
        pid->fd[1] = 0.0f;
        }
    pid->e[2] = pid->e[1];
    pid->e[1] = pid->e[0];
    pid->e[0] = sp - act;
    pid->out = pid->out + pid->a * pid->e[0] - pid->kp * pid->e[1];// PI
    pid->d[1] = pid->d[0];// Filtered D
    pid->d[0] = pid->ad[0] * pid->e[0] + pid->ad[1] * pid->e[1] + pid->ad[0] * pid->e[2];
    pid->fd[1] = pid->fd[0];
    pid->fd[0] = ((pid->alpha) / (pid->alpha + 1)) * (pid->d[0] + pid->d[1]) - ((pid->alpha - 1) / (pid->alpha + 1)) * pid->fd[1];
    if (pid->fd[0] < 0.001f) {
        pid->fd[0] = 0.0f;
        }
    pid->out = pid->out + pid->fd[0];
    if (pid->out < min) {
        pid->out = min;
        }
    else if (pid->out > max) {
        pid->out = max;
        }
    return ( size_t )pid->out;
    }
