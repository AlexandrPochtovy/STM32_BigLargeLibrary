/*********************************************************************************
 Original author: Atmel Corporation: http://www.atmel.com
 *                       Support email: avr@atmel.com
 * AppNote:              AVR221 - Discrete PID controller
 * $Name$
 * $Revision: 456 $
 * $RCSfile$
 * $Date: 2006-02-16 12:46:13 +0100 (to, 16 feb 2006) $

Modification for STM32: Aliaksandr Pachtovy<alex.mail.prime@gmail.com>
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

*****************************************************************************/
#include <stdint.h>
#include <stddef.h>

#ifndef _PID_H_
#define _PID_H_

#define MY_SCALING_FACTOR 1024

/*! brief PID Status
 *
 * Setpoints and data used by the PID control algorithm
 */
typedef struct PID_DATA{
  int32_t lastProcessValue; //Last process value, used to find derivative of process value.
  int32_t integral;					//Summation of errors, used for integrate calculations
  int32_t P_Factor;					//The Proportional tuning constant, multiplied with SCALING_FACTOR
  int32_t I_Factor;					//The Integral tuning constant, multiplied with SCALING_FACTOR
  int32_t D_Factor;					//The Derivative tuning constant, multiplied with SCALING_FACTOR
} pidData_t;

void pid_Init(int16_t p_factor, int16_t i_factor, int16_t d_factor, pidData_t *pid);
int16_t pid_Controller(int32_t setPoint, int32_t processValue, pidData_t *pid, int32_t cycle);
void pid_Reset_Integrator(pidData_t *pid);

#endif
