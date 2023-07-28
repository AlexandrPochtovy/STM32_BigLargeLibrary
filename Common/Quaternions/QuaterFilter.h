/*********************************************************************************
   Original author: Alexandr Pochtovy<alex.mail.prime@gmail.com>

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

Based on iIplementation of Madgwick's IMU and AHRS algorithms.
See:https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
	https://github.com/xioTechnologies/Fusion

	Date	Author			Notes
29/09/2011	SOH Madgwick    Initial release
02/10/2011	SOH Madgwick	Optimised for reduced CPU load
19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
The MIT License (MIT)
Copyright (c) 2021 x-io Technologies
*********************************************************************************/

#ifndef _QUATERFILTER_H_
#define _QUATERFILTER_H_

#ifdef __cplusplus
extern "C" {
#endif

// Math library required for ‘sqrt’
#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include "Function/Function.h"

typedef struct AccelAxis_t {
	float a_x;// accelerometer X- axis measurements
	float a_y;// accelerometer Y- axis measurements
	float a_z;// accelerometer Z- axis measurements
} AccelAxis;

typedef struct GyroAxis_t {
	float w_x;// gyroscope X- axis measurements
	float w_y;// gyroscope Y- axis measurements
	float w_z;// gyroscope Z- axis measurements
} GyroAxis;

typedef struct MagAxis_t {
	float m_x;// gyroscope X- axis measurements
	float m_y;// gyroscope Y- axis measurements
	float m_z;// gyroscope Z- axis measurements
} MagAxis;

typedef struct Quat_DOF_t {
  volatile float SEq_1;
  volatile float SEq_2;
  volatile float SEq_3;
  volatile float SEq_4;
  //volatile float beta;
  //volatile float freq;
} Quat_DOF;

//Quaternion
typedef struct Quaternion_t {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

typedef struct EulerAngles_t {
    float roll;
    float pitch;
    float yaw;
} EulerAngles;


void FilterUpdate_6DOF(AccelAxis *accel, GyroAxis *gyro);
void FilterUpdate6(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z);
void FilterUpdate_9DOF(AccelAxis *accel, GyroAxis *gyro, MagAxis *mag);
void FilterUpdate9(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
EulerAngles ToEulerAngles(Quaternion q, uint8_t deg);
#ifdef __cplusplus
}
#endif

#endif // _QUATERFILTER_H_
