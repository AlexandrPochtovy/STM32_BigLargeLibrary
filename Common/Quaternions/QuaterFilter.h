/*********************************************************************************
Original author: x-io Technologies 
http://www.x-io.co.uk/
https://github.com/xioTechnologies

The MIT License (MIT)

Copyright (c) 2021 x-io Technologies

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

https://habr.com/ru/articles/255661/

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

  QuaterFilter.h
	Created on: 11.02.2021

Add fix 
https://diydrones.com/forum/topics/madgwick-imu-ahrs-and-fast-inverse-square-root?id=705844%3ATopic%3A1018435&page=4#comments
*********************************************************************************/

#ifndef _QUATERFILTER_H_
#define _QUATERFILTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <math.h>   // Math library required for ‘sqrt’ and PI
#include <string.h>
#include "Function/Function.h"

typedef struct Axis {
	float x;// X- axis measurements
	float y;// Y- axis measurements
	float z;// Z- axis measurements
} vector_t;

//Quaternion
typedef struct Quaternion {
    float w;
    float x;
    float y;
    float z;
} Quaternion_t;

typedef struct EulerAngles {
    float roll;
    float pitch;
    float yaw;
} EulerAngles_t;

// System constants
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define gyroMeasError (M_PI * (5.0f / 180.0f))      // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasDrift (M_PI * (0.2f / 180.0f))      // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define betaVal (sqrtf(3.0f / 4.0f) * gyroMeasError)   // compute beta
#define zetaVal (sqrtf(3.0f / 4.0f) * gyroMeasDrift)// compute zeta
#define betaDef		0.1f					                    // 2 * proportional gain

static const float beta = betaVal;
static const float zeta = zetaVal;

/*****************************************************************************************
*                   HABR.COM                                                             *
*****************************************************************************************/
uint8_t QuaternionCalcHabr_6Axis(vector_t accel, vector_t gyro, uint32_t deltat, Quaternion_t *SIQ);
uint8_t QuaternionCalcHabr_6(float a_x, float a_y, float a_z, float g_x, float g_y, float g_z, uint32_t deltat, Quaternion_t *SIQ);
uint8_t QuaternionCalcHabr_9Axis(vector_t accel, vector_t gyro, vector_t mag, uint32_t deltat, Quaternion_t *SIQ);
uint8_t QuaternionCalcHabr_9(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z, uint32_t deltat, Quaternion_t *SIQ);

/*****************************************************************************************
*                   MAHONY                                                               *
*****************************************************************************************/
uint8_t MahonyAHRS_9(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, uint32_t deltat, Quaternion_t *SIQ);
uint8_t MahonyAHRS_6(float gx, float gy, float gz, float ax, float ay, float az, float twoKp, float twoKi, uint32_t deltat, Quaternion_t *SIQ);
/*****************************************************************************************
*                   MADGWICK_AHRS                                                        *
*****************************************************************************************/
uint8_t MadgwickAHRS_9(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, uint32_t deltat, Quaternion_t *SIQ);
uint8_t MadgwickAHRS_6(float gx, float gy, float gz, float ax, float ay, float az, uint32_t deltat, Quaternion_t *SIQ);
/*****************************************************************************************
*                   CONVERSION                                                           *
*****************************************************************************************/
Quaternion_t EulerAngleToQuaternion(double yaw, double pitch, double roll);
EulerAngles_t QuaternionToEulerAngles(Quaternion_t q, uint8_t deg);
#ifdef __cplusplus
}
#endif

#endif // _QUATERFILTER_H_
