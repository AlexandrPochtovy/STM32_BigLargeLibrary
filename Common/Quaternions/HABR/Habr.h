/*
 * Habr.h
 *
 *  Created on: Nov 24, 2023
 *      Author: alexm
 */

#include <math.h>
#include "Quaternions/QuaternionUDT.h"
#include "Function/Function.h"

// System constants

#define gyroMeasError (M_PI * 2.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 2 deg/s)
#define gyroMeasDrift (M_PI * 0.2f / 180.0f) // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
#define zeta sqrt(3.0f / 4.0f) * gyroMeasDrift // compute zeta

Quaternion_t AHRSUpdate6(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float deltat);
Quaternion_t AHRSUpdate9(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z, float deltat);

