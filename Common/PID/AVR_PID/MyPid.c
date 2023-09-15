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

#include "MyPid.h"

/*! brief Initialisation of PID controller parameters.
 *
 *  Initialise the variables used by the PID algorithm.
 *
 *  \param p_factor  Proportional term.
 *  \param i_factor  Integral term.
 *  \param d_factor  Derivate term.
 *  \param pid  Struct with PID status.
 */
void pid_Init(int16_t p_factor, int16_t i_factor, int16_t d_factor, pidData_t *pid) {
  pid->lastProcessValue = 0;
  pid->integral = 0;
  // Tuning constants for PID loop
  pid->P_Factor = p_factor;
  pid->I_Factor = i_factor;
  pid->D_Factor = d_factor;
}

/*! \brief PID control algorithm.
 *
 *  Calculates output from setpoint, process value and PID status.
 *
 *  \param setPoint  Desired value.
 *  \param processValue  Measured value.
 *  \param pid_st  PID status struct.
 */
int16_t pid_Controller(int32_t setPoint, int32_t processValue, pidData_t *pid, int32_t cycle) {
  int32_t error, p_term, d_term, i_term;
  // Calculate Pterm and limit error overflow
  error = setPoint - processValue;
  p_term = pid->P_Factor * error;

  // Calculate Iterm and limit integral runaway
  pid->integral += error * cycle / 1000;
  if (pid->integral > pid->maxInt) {
  	pid->integral = pid->maxInt;
  } else if (pid->integral < -pid->maxInt) {
  	pid->integral = -pid->maxInt;
  }
  i_term = pid->I_Factor * pid->integral;

  // Calculate Dterm
  d_term = pid->D_Factor * (error - pid->lastProcessValue) / cycle;
  pid->lastProcessValue = error;

  int32_t tempOut = (p_term + i_term + d_term) / MY_SCALING_FACTOR;
  if (tempOut > 1000) {
  	return 1000;
  } else if (tempOut < 0) {
  	return 0;
  } else {
  	return tempOut;
  }
}

/*! brief Resets the integrator.
 *
 *  Calling this function will reset the integrator in the PID regulator.
 */
void pid_Reset_Integrator(pidData_t *pid)
{
  pid->integral = 0;
}
