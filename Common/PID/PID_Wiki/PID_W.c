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

#include "PID_W.h"

void Pid_Init(pid_t *pid, int32_t kp, int32_t ki, int32_t kd, int32_t dt) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->N = 5;
    pid->e[0] = 0;
    pid->e[1] = 0;
    pid->e[2] = 0;
    pid->d0 = 0;
    pid->d1 = 0;
    pid->fd0 = 0;
    pid->fd1 = 0;
    pid->out = 0;
}

int32_t PidProcessingSimple(pid_t *pid, int32_t actual, int32_t sp, int32_t min, int32_t max, uint32_t dt) {
    float A0 = pid->Kp + pid->Ki*dt + pid->Kd/dt;
    float A1 = -pid->Kp - 2*pid->Kd/dt;
    float A2 = pid->Kd/dt;

    pid->e[2] = pid->e[1];
    pid->e[1] = pid->e[0];
    pid->e[0] = sp - actual;
    pid->out = pid->out + A0 * pid->e[0] + A1 * pid->e[1] + A2 * pid->e[2];
    return Max(Min(pid->out, 0), 1000);
}

int32_t PidProcessingFiltered(pid_t *pid, int32_t actual, int32_t sp, int32_t min, int32_t max, uint32_t dt) {
    float A0 = pid->Kp + pid->Ki*dt;
    float A1 = -pid->Kp;
    float A0d = pid->Kd/dt;
    float A1d = - 2.0 * pid->Kd/dt;
    float A2d = pid->Kd / dt;
    float tau = pid->Kd / (pid->Kp * N); // IIR filter time constant
    float alpha = dt / (2 * tau);

    pid->e[2] = pid->e[1];
    pid->e[1] = pid->e[0];
    pid->e[0] = sp - actual;
    // PI
    pid->out = pid->out + A0 * pid->e[0] + A1 * pid->e[1];
    // Filtered D
    pid->d1 = pid->d0;
    pid->d0 = A0d * pid->e[0] + A1d * pid->e[1] + A2d * pid->e[2];
    pid->fd1 = pid->fd0;
    pid->fd0 = ((alpha) / (alpha + 1)) * (pid->d0 + pid->d1) - ((alpha - 1) / (alpha + 1)) * pid->fd1;
    pid->out = pid->out + pid->fd0;    
    return Max(Min(pid->out, 0), 1000); 
}