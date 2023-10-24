/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief Header file for pid.c.
 *
 * - File:               pid.h
 * - Compiler:           IAR EWAAVR 4.11A
 * - Supported devices:  All AVR devices can be used.
 * - AppNote:            AVR221 - Discrete PID controller
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support email: avr@atmel.com
 *
 * $Name$
 * $Revision: 456 $
 * $RCSfile$
 * $Date: 2006-02-16 12:46:13 +0100 (to, 16 feb 2006) $
 *****************************************************************************/

#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stddef.h>

#define SCALING_FACTOR  128

/*! \brief PID Status
 *
 * Setpoints and data used by the PID control algorithm
 */
typedef struct PID_DATA{
  int16_t lastProcessValue;//! Last process value, used to find derivative of process value.
  size_t sumError; //! Summation of errors, used for integrate calculations
  int16_t P_Factor;//! The Proportional tuning constant, multiplied with SCALING_FACTOR
  int16_t I_Factor;//! The Integral tuning constant, multiplied with SCALING_FACTOR
  int16_t D_Factor;//! The Derivative tuning constant, multiplied with SCALING_FACTOR
  int16_t maxError;//! Maximum allowed error, avoid overflow
  size_t maxSumError;//! Maximum allowed sumerror, avoid overflow
} pidData_t;

/*! \brief Maximum values
 *
 * Needed to avoid sign/overflow problems
 */
// Maximum value of variables
#define MAX_INT         INT16_MAX
#define MAX_LONG        INT32_MAX
#define MAX_I_TERM      (MAX_LONG / 2)

void pid_Init(int16_t p_factor, int16_t i_factor, int16_t d_factor, struct PID_DATA *pid);
int16_t pid_Controller(int16_t setPoint, int16_t processValue, struct PID_DATA *pid_st);
void pid_Reset_Integrator(pidData_t *pid_st);

#endif
