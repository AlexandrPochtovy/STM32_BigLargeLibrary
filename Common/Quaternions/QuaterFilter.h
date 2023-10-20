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
#include <math.h>
#include <string.h>
#include "Function/Function.h"

/**********************************************************************
*                       TYPEDEF's                                     * 
***********************************************************************/

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

/*****************************************************************************************
*                   HABR.COM                                                             *
*****************************************************************************************/
/*****************************************************************
  * @brief calculate quaternion vector for 6 value axis: accelerometer and gyroscope without magnetometer
  * @param accel - accelerometer axis structure value
  * @param gyro - gyroscope axis structure value
  * @param dT - time between calcilation in msec
  * @param *SIQ - pointer for Quaternion_t quaternion structure
  * @retval 1 when end
  */
uint8_t QuaternionCalcHabr_6Axis(vector_t accel, vector_t gyro, uint32_t dT, Quaternion_t *SIQ);

/*****************************************************************
  * @brief calculate quaternion vector for 6 value axis: accelerometer and gyroscope without magnetometer
  * @param ax - accelerometer X-axis value
  * @param ay - accelerometer Y-axis value
  * @param az - accelerometer Z-axis value
  * @param gx - gyroscope X-axis value
  * @param gy - gyroscope Y-axis value
  * @param gz - gyroscope Z-axis value
  * @param dT - time between calcilation in msec
  * @param *SIQ - pointer for Quaternion_t quaternion structure
  * @retval 1 when end
  */
uint8_t QuaternionCalcHabr_6(float ax, float ay, float az, float gx, float gy, float gz, uint32_t dT, Quaternion_t *SIQ);

/*****************************************************************
  * @brief calculate quaternion vector for 9 value axis: accelerometer, gyroscope and magnetometer
  * @param accel - accelerometer axis structure value
  * @param gyro - gyroscope axis structure value
  * @param mag - magnetometer axis structure value
  * @param dT - time between calcilation in msec
  * @param *SIQ - pointer for Quaternion_t quaternion structure
  * @retval 1 when end
  */
uint8_t QuaternionCalcHabr_9Axis(vector_t accel, vector_t gyro, vector_t mag, uint32_t dT, Quaternion_t *SIQ);

/*****************************************************************
  * @brief calculate quaternion vector for 9 value axis: accelerometer, gyroscope and magnetometer
  * @param ax - accelerometer X-axis value
  * @param ay - accelerometer Y-axis value
  * @param az - accelerometer Z-axis value
  * @param gx - gyroscope X-axis value
  * @param gy - gyroscope Y-axis value
  * @param gz - gyroscope Z-axis value
  * @param mx - magnetometer X-axis value
  * @param my - magnetometer Y-axis value
  * @param mz - magnetometer Z-axis value
  * @param dT - time between calcilation in msec
  * @param *SIQ - pointer for Quaternion_t quaternion structure
  * @retval 1 when end
  */
uint8_t QuaternionCalcHabr_9(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, uint32_t dT, Quaternion_t *SIQ);

/*****************************************************************************************
*                   MAHONY                                                               *
Mahony's algorythm for quaternion calculating
*****************************************************************************************/
/*****************************************************************
  * @brief calculate quaternion vector for 9 value axis: accelerometer, gyroscope and magnetometer
  * @param ax - accelerometer X-axis value
  * @param ay - accelerometer Y-axis value
  * @param az - accelerometer Z-axis value
  * @param gx - gyroscope X-axis value
  * @param gy - gyroscope Y-axis value
  * @param gz - gyroscope Z-axis value
  * @param mx - magnetometer X-axis value
  * @param my - magnetometer Y-axis value
  * @param mz - magnetometer Z-axis value
  * @param twoKp - proportional gain factor
  * @param twoKi - integral gain factor
  * @param dT - time between calcilation in msec
  * @param *SIQ - pointer for Quaternion_t quaternion structure
  * @retval 1 when end
  */
uint8_t MahonyAHRS_9(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float twoKp, float twoKi, uint32_t dT, Quaternion_t *SIQ);

/*****************************************************************
  * @brief calculate quaternion vector for 6 value axis: accelerometer and gyroscope without magnetometer
  * @param ax - accelerometer X-axis value
  * @param ay - accelerometer Y-axis value
  * @param az - accelerometer Z-axis value
  * @param gx - gyroscope X-axis value
  * @param gy - gyroscope Y-axis value
  * @param gz - gyroscope Z-axis value
  * @param twoKp - proportional gain factor
  * @param twoKi - integral gain factor
  * @param dT - time between calcilation in msec
  * @param *SIQ - pointer for Quaternion_t quaternion structure
  * @retval 1 when end
  */
uint8_t MahonyAHRS_6(float ax, float ay, float az, float gx, float gy, float gz, float twoKp, float twoKi, uint32_t dT, Quaternion_t *SIQ);

/*****************************************************************************************
*                   MADGWICK_AHRS                                                        *
Madgwick's algorythm for quaternion calculating
*****************************************************************************************/
/*****************************************************************
  * @brief calculate quaternion vector for 9 value axis: accelerometer, gyroscope and magnetometer
  * @param ax - accelerometer X-axis value
  * @param ay - accelerometer Y-axis value
  * @param az - accelerometer Z-axis value
  * @param gx - gyroscope X-axis value
  * @param gy - gyroscope Y-axis value
  * @param gz - gyroscope Z-axis value
  * @param mx - magnetometer X-axis value
  * @param my - magnetometer Y-axis value
  * @param mz - magnetometer Z-axis value
  * @param dT - time between calcilation in msec
  * @param *SIQ - pointer for Quaternion_t quaternion structure
  * @retval 1 when end
  */
uint8_t MadgwickAHRS_9(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, uint32_t dT, Quaternion_t *SIQ);

/*****************************************************************
  * @brief calculate quaternion vector for 6 value axis: accelerometer and gyroscope without magnetometer
  * @param ax - accelerometer X-axis value
  * @param ay - accelerometer Y-axis value
  * @param az - accelerometer Z-axis value
  * @param gx - gyroscope X-axis value
  * @param gy - gyroscope Y-axis value
  * @param gz - gyroscope Z-axis value
  * @param twoKp - proportional gain factor
  * @param twoKi - integral gain factor
  * @param dT - time between calcilation in msec
  * @param *SIQ - pointer for Quaternion_t quaternion structure
  * @retval 1 when end
  */
uint8_t MadgwickAHRS_6(float ax, float ay, float az, float gx, float gy, float gz, uint32_t deltat, Quaternion_t *SIQ);
/*****************************************************************************************
*                   CONVERSION                                                           *
*****************************************************************************************/

/*****************************************************************
  * @brief converter from Euler angle to quaternion
  * @param yaw - yaw rotation
  * @param pitch - pitch rotation
  * @param rollaz - roll rotation
  * @retval Quaternion_t quaternion vector
  */
Quaternion_t EulerAngleToQuaternion(double yaw, double pitch, double roll);

/*****************************************************************
  * @brief converter from quaternion to Euler angle
  * @param q - Quaternion_t quaternion vector
  * @param deg - select Euler output: 0 - rad, 1 - degree
  * @retval EulerAngles_t angle rotation vector
  */
EulerAngles_t QuaternionToEulerAngles(Quaternion_t q, uint8_t deg);
#ifdef __cplusplus
}
#endif

#endif // _QUATERFILTER_H_
