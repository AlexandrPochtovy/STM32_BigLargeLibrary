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

  QuaterFilter.c
	Created on: 11.02.2021

Add fix 
https://diydrones.com/forum/topics/madgwick-imu-ahrs-and-fast-inverse-square-root?id=705844%3ATopic%3A1018435&page=4#comments
*********************************************************************************/

#include "QuaterFilter.h"

#define gyroMeasError (M_PI * (5.0f / 180.0f))      // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasDrift (M_PI * (0.2f / 180.0f))      // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define betaVal (sqrt(3.0f / 4.0f) * gyroMeasError)// compute beta
#define zetaVal (sqrt(3.0f / 4.0f) * gyroMeasDrift)// compute zeta
#define betaDef		0.1f					        // 2 * proportional gain
//ADD calculate coeff as parameters
static float beta = betaVal;
static float zeta = zetaVal;
static float b_x = 1, b_z = 0;                          // reference direction of flux in earth frame
static float w_bx = 0, w_by = 0, w_bz = 0;              // estimate gyroscope biases error
static vector_t integralFB = {0.0f,  0.0f, 0.0f};	    // integral error terms scaled by Ki
Quaternion_t Q = {1.0, 0.0, 0.0, 0.0};	                // quaternion of sensor frame relative to auxiliary frame

/*****************************************************************************************
*                   HABR.COM                                                             *
*****************************************************************************************/
uint8_t QuaternionCalcHabr_6Axis(vector_t accel, vector_t gyro, uint32_t dT, Quaternion_t *SIQ) {
    // Local system variables
    float norm;             // vector norm
    float SEqDot_omega[4];  // quaternion derrivative from gyroscopes elements
    float f[3];             // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float SEqHatDot[4]; // estimated direction of the gyroscope error
    // Axulirary variables to avoid reapeated calcualtions
    float halfSEq[4];
    halfSEq[0] = 0.5f * SIQ->w; halfSEq[1] = 0.5f * SIQ->x;
    halfSEq[2] = 0.5f * SIQ->y; halfSEq[3] = 0.5f * SIQ->z;
    float twow = 2.0f * SIQ->w;
    float twox = 2.0f * SIQ->x;
    float twoy = 2.0f * SIQ->y;
    // Normalise the accelerometer measurement
    norm = sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    accel.x /= norm;
    accel.y /= norm;
    accel.z /= norm;
    // Compute the objective function and Jacobian
    f[0] = twox * SIQ->z - twow * SIQ->y - accel.x;
    f[1] = twow * SIQ->x + twoy * SIQ->z - accel.y;
    f[2] = 1.0f - twox * SIQ->x - twoy * SIQ->y - accel.z;
    J_11or24 = twoy; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SIQ->z;
    J_13or22 = twow; // J_12 negated in matrix multiplication
    J_14or21 = twox;
    J_32 = 2.0f * J_14or21; // negated in matrix multiplication
    J_33 = 2.0f * J_11or24; // negated in matrix multiplication
    // Compute the gradient (matrix multiplication)
    SEqHatDot[0] = J_14or21 * f[1] - J_11or24 * f[0];
    SEqHatDot[1] = J_12or23 * f[0] + J_13or22 * f[1] - J_32 * f[2];
    SEqHatDot[2] = J_12or23 * f[1] - J_33 * f[2] - J_13or22 * f[0];
    SEqHatDot[3] = J_14or21 * f[0] + J_11or24 * f[1];
    // Normalise the gradient
    norm = sqrt(SEqHatDot[0] * SEqHatDot[0] + SEqHatDot[1] * SEqHatDot[1] + SEqHatDot[2] * SEqHatDot[2] + SEqHatDot[3] * SEqHatDot[3]);
    SEqHatDot[0] /= norm;
    SEqHatDot[1] /= norm;
    SEqHatDot[2] /= norm;
    SEqHatDot[3] /= norm;
    // Compute the quaternion derrivative measured by gyroscopes
    SEqDot_omega[0] = -halfSEq[1] * gyro.x - halfSEq[2] * gyro.y - halfSEq[3] * gyro.z;
    SEqDot_omega[1] = halfSEq[0] * gyro.x + halfSEq[2] * gyro.z - halfSEq[3] * gyro.y;
    SEqDot_omega[2] = halfSEq[0] * gyro.y - halfSEq[1] * gyro.z + halfSEq[3] * gyro.x;
    SEqDot_omega[3] = halfSEq[0] * gyro.z + halfSEq[1] * gyro.y - halfSEq[2] * gyro.x;
    // Compute then integrate the estimated quaternion derrivative
    SIQ->w += (SEqDot_omega[0] - (beta * SEqHatDot[0])) * dT / 1000;
    SIQ->x += (SEqDot_omega[1] - (beta * SEqHatDot[1])) * dT / 1000;
    SIQ->y += (SEqDot_omega[2] - (beta * SEqHatDot[2])) * dT / 1000;
    SIQ->z += (SEqDot_omega[3] - (beta * SEqHatDot[3])) * dT / 1000;
    // Normalise quaternion
    norm = sqrt(SIQ->w * SIQ->w + SIQ->x * SIQ->x + SIQ->y *SIQ->y + SIQ->z * SIQ->z);
    SIQ->w /= norm;
    SIQ->x /= norm;
    SIQ->y /= norm;
    SIQ->z /= norm;
    return 1;
}

uint8_t QuaternionCalcHabr_6(float ax, float ay, float az, float gx, float gy, float gz, uint32_t dT, Quaternion_t *SIQ) {
    // Local system variables
    float norm;             // vector norm
    float SEqDot_omega[4];  // quaternion derrivative from gyroscopes elements
    float f[3];             // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float SEqHatDot[4]; // estimated direction of the gyroscope error
    // Axulirary variables to avoid reapeated calcualtions
    float halfSEq[4];
    halfSEq[0] = 0.5f * SIQ->w; halfSEq[1] = 0.5f * SIQ->x;
    halfSEq[2] = 0.5f * SIQ->y; halfSEq[3] = 0.5f * SIQ->z;
    float twow = 2.0f * SIQ->w;
    float twox = 2.0f * SIQ->x;
    float twoy = 2.0f * SIQ->y;
    // Normalise the accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    ax /= norm;
    ay /= norm;
    az /= norm;
    // Compute the objective function and Jacobian
    f[0] = twox * SIQ->z - twow * SIQ->y - ax;
    f[1] = twow * SIQ->x + twoy * SIQ->z - ay;
    f[2] = 1.0f - twox * SIQ->x - twoy * SIQ->y - az;
    J_11or24 = twoy; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SIQ->z;
    J_13or22 = twow; // J_12 negated in matrix multiplication
    J_14or21 = twox;
    J_32 = 2.0f * J_14or21; // negated in matrix multiplication
    J_33 = 2.0f * J_11or24; // negated in matrix multiplication
    // Compute the gradient (matrix multiplication)
    SEqHatDot[0] = J_14or21 * f[1] - J_11or24 * f[0];
    SEqHatDot[1] = J_12or23 * f[0] + J_13or22 * f[1] - J_32 * f[2];
    SEqHatDot[2] = J_12or23 * f[1] - J_33 * f[2] - J_13or22 * f[0];
    SEqHatDot[3] = J_14or21 * f[0] + J_11or24 * f[1];
    // Normalise the gradient
    norm = sqrt(SEqHatDot[0] * SEqHatDot[0] + SEqHatDot[1] * SEqHatDot[1] + SEqHatDot[2] * SEqHatDot[2] + SEqHatDot[3] * SEqHatDot[3]);
    SEqHatDot[0] /= norm;
    SEqHatDot[1] /= norm;
    SEqHatDot[2] /= norm;
    SEqHatDot[3] /= norm;
    // Compute the quaternion derrivative measured by gyroscopes
    SEqDot_omega[0] = -halfSEq[1] * gx - halfSEq[2] * gy - halfSEq[3] * gz;
    SEqDot_omega[1] = halfSEq[0] * gx + halfSEq[2] * gz - halfSEq[3] * gy;
    SEqDot_omega[2] = halfSEq[0] * gy - halfSEq[1] * gz + halfSEq[3] * gx;
    SEqDot_omega[3] = halfSEq[0] * gz + halfSEq[1] * gy - halfSEq[2] * gx;
    // Compute then integrate the estimated quaternion derrivative
    SIQ->w += (SEqDot_omega[0] - (beta * SEqHatDot[0])) * dT / 1000;
    SIQ->x += (SEqDot_omega[1] - (beta * SEqHatDot[1])) * dT / 1000;
    SIQ->y += (SEqDot_omega[2] - (beta * SEqHatDot[2])) * dT / 1000;
    SIQ->z += (SEqDot_omega[3] - (beta * SEqHatDot[3])) * dT / 1000;
    // Normalise quaternion
    norm = sqrt(SIQ->w * SIQ->w + SIQ->x * SIQ->x + SIQ->y *SIQ->y + SIQ->z * SIQ->z);
    SIQ->w /= norm;
    SIQ->x /= norm;
    SIQ->y /= norm;
    SIQ->z /= norm;
    return 1;
}

uint8_t QuaternionCalcHabr_9Axis(vector_t accel, vector_t gyro, vector_t mag, uint32_t dT, Quaternion_t *SIQ) {
    // local system variables
    float norm; // vector norm
    float SEqDot_omega[4];  // quaternion derrivative from gyroscopes elements
    float f[6];             // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, // objective function Jacobian elements
    J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
    float w_err_x, w_err_y, w_err_z; // estimated direction of the gyroscope error (angular)
    float h_x, h_y, h_z; // computed flux in the earth frame
    // axulirary variables to avoid reapeated calcualtions
    float halfw = 0.5f * SIQ->w;
    float halfx = 0.5f * SIQ->x;
    float halfy = 0.5f * SIQ->y;
    float halfz = 0.5f * SIQ->z;
    float twow = 2.0f * SIQ->w;
    float twox = 2.0f * SIQ->x;
    float twoy = 2.0f * SIQ->y;
    float twoz = 2.0f * SIQ->z;
    float twob_x = 2.0f * b_x;
    float twob_z = 2.0f * b_z;
    float twob_xw = 2.0f * b_x * SIQ->w;
    float twob_xx = 2.0f * b_x * SIQ->x;
    float twob_xy = 2.0f * b_x * SIQ->y;
    float twob_xz = 2.0f * b_x * SIQ->z;
    float twob_zw = 2.0f * b_z * SIQ->w;
    float twob_zx = 2.0f * b_z * SIQ->x;
    float twob_zy = 2.0f * b_z * SIQ->y;
    float twob_zz = 2.0f * b_z * SIQ->z;
    float wx;
    float wy = SIQ->w * SIQ->y;
    float wz;
    float xy;
    float xz = SIQ->x * SIQ->z;
    float yz;
    float twom_x = 2.0f * mag.x;
    float twom_y = 2.0f * mag.y;
    float twom_z = 2.0f * mag.z;
    // normalise the accelerometer measurement
    norm = sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    accel.x /= norm;
    accel.y /= norm;
    accel.z /= norm;
    // normalise the magnetometer measurement
    norm = sqrt(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
    mag.x /= norm;
    mag.y /= norm;
    mag.z /= norm;
    // compute the objective function and Jacobian
    f[0] = twox * SIQ->z - twow * SIQ->y - accel.x;
    f[1] = twow * SIQ->x + twoy * SIQ->z - accel.y;
    f[2] = 1.0f - twox * SIQ->x - twoy * SIQ->y - accel.z;
    f[3] = twob_x * (0.5f - SIQ->y * SIQ->y - SIQ->z * SIQ->z) + twob_z * (xz - wy) - mag.x;
    f[4] = twob_x * (SIQ->x * SIQ->y - SIQ->w * SIQ->z) + twob_z * (SIQ->w * SIQ->x + SIQ->y * SIQ->z) - mag.y;
    f[5] = twob_x * (wy + xz) + twob_z * (0.5f - SIQ->x * SIQ->x - SIQ->y * SIQ->y) - mag.z;
    J_11or24 = twoy; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SIQ->z;
    J_13or22 = twow; // J_12 negated in matrix multiplication
    J_14or21 = twox;
    J_32 = 2.0f * J_14or21; // negated in matrix multiplication
    J_33 = 2.0f * J_11or24; // negated in matrix multiplication
    J_41 = twob_zy; // negated in matrix multiplication
    J_42 = twob_zz;
    J_43 = 2.0f * twob_xy + twob_zw; // negated in matrix multiplication
    J_44 = 2.0f * twob_xz - twob_zx; // negated in matrix multiplication
    J_51 = twob_xz - twob_zx; // negated in matrix multiplication
    J_52 = twob_xy + twob_zw;
    J_53 = twob_xx + twob_zz;
    J_54 = twob_xw - twob_zy; // negated in matrix multiplication
    J_61 = twob_xy;
    J_62 = twob_xz - 2.0f * twob_zx;
    J_63 = twob_xw - 2.0f * twob_zy;
    J_64 = twob_xx;
    // compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f[1] - J_11or24 * f[0] - J_41 * f[3] - J_51 * f[4] + J_61 * f[5];
    SEqHatDot_2 = J_12or23 * f[0] + J_13or22 * f[1] - J_32 * f[2] + J_42 * f[3] + J_52 * f[4] + J_62 * f[5];
    SEqHatDot_3 = J_12or23 * f[1] - J_33 * f[2] - J_13or22 * f[0] - J_43 * f[3] + J_53 * f[4] + J_63 * f[5];
    SEqHatDot_4 = J_14or21 * f[0] + J_11or24 * f[1] - J_44 * f[3] - J_54 * f[4] + J_64 * f[5];
    // normalise the gradient to estimate direction of the gyroscope error
    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 = SEqHatDot_1 / norm;
    SEqHatDot_2 = SEqHatDot_2 / norm;
    SEqHatDot_3 = SEqHatDot_3 / norm;
    SEqHatDot_4 = SEqHatDot_4 / norm;
    // compute angular estimated direction of the gyroscope error
    w_err_x = twow * SEqHatDot_2 - twox * SEqHatDot_1 - twoy * SEqHatDot_4 + twoz * SEqHatDot_3;
    w_err_y = twow * SEqHatDot_3 + twox * SEqHatDot_4 - twoy * SEqHatDot_1 - twoz * SEqHatDot_2;
    w_err_z = twow * SEqHatDot_4 - twox * SEqHatDot_3 + twoy * SEqHatDot_2 - twoz * SEqHatDot_1;
    // compute and remove the gyroscope baises
    w_bx += w_err_x * dT * zeta;
    w_by += w_err_y * dT * zeta;
    w_bz += w_err_z * dT * zeta;
    gyro.x -= w_bx;
    gyro.y -= w_by;
    gyro.z -= w_bz;
    // compute the quaternion rate measured by gyroscopes
    SEqDot_omega[0] = -halfx * gyro.x - halfy * gyro.y - halfz * gyro.z;
    SEqDot_omega[1] = halfw * gyro.x + halfy * gyro.z - halfz * gyro.y;
    SEqDot_omega[2] = halfw * gyro.y - halfx * gyro.z + halfz * gyro.x;
    SEqDot_omega[3] = halfw * gyro.z + halfx * gyro.y - halfy * gyro.x;
    // compute then integrate the estimated quaternion rate
    SIQ->w += (SEqDot_omega[0] - (beta * SEqHatDot_1)) * dT / 1000;
    SIQ->x += (SEqDot_omega[1] - (beta * SEqHatDot_2)) * dT / 1000;
    SIQ->y += (SEqDot_omega[2] - (beta * SEqHatDot_3)) * dT / 1000;
    SIQ->z += (SEqDot_omega[3] - (beta * SEqHatDot_4)) * dT / 1000;
    // normalise quaternion
    norm = sqrt(SIQ->w * SIQ->w + SIQ->x * SIQ->x + SIQ->y * SIQ->y + SIQ->z * SIQ->z);
    SIQ->w /= norm;
    SIQ->x /= norm;
    SIQ->y /= norm;
    SIQ->z /= norm;
    // compute flux in the earth frame
    wx = SIQ->w * SIQ->x; // recompute axulirary variables
    wy = SIQ->w * SIQ->y;
    wz = SIQ->w * SIQ->z;
    yz = SIQ->y * SIQ->z;
    xy = SIQ->x * SIQ->y;
    xz = SIQ->x * SIQ->z;
    h_x = twom_x * (0.5f - SIQ->y * SIQ->y - SIQ->z * SIQ->z) + twom_y * (xy - wz) + twom_z * (xz + wy);
    h_y = twom_x * (xy + wz) + twom_y * (0.5f - SIQ->x * SIQ->x - SIQ->z * SIQ->z) + twom_z * (yz - wx);
    h_z = twom_x * (xz - wy) + twom_y * (yz + wx) + twom_z * (0.5f - SIQ->x * SIQ->x - SIQ->y * SIQ->y);
    // normalise the flux vector to have only components in the x and z
    b_x = sqrt((h_x * h_x) + (h_y * h_y));
    b_z = h_z;
    return 1;
}

uint8_t QuaternionCalcHabr_9(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, uint32_t dT, Quaternion_t *SIQ) {
    // local system variables
    float norm; // vector norm
    float SEqDot_omega[4];  // quaternion derrivative from gyroscopes elements
    float f[6];// objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, // objective function Jacobian elements
    J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
    float w_err_x, w_err_y, w_err_z; // estimated direction of the gyroscope error (angular)
    float h_x, h_y, h_z; // computed flux in the earth frame
    // axulirary variables to avoid reapeated calcualtions
    float halfw = 0.5f * SIQ->w;
    float halfx = 0.5f * SIQ->x;
    float halfy = 0.5f * SIQ->y;
    float halfz = 0.5f * SIQ->z;
    float twow = 2.0f * SIQ->w;
    float twox = 2.0f * SIQ->x;
    float twoy = 2.0f * SIQ->y;
    float twoz = 2.0f * SIQ->z;
    float twob_x = 2.0f * b_x;
    float twob_z = 2.0f * b_z;
    float twob_xw = 2.0f * b_x * SIQ->w;
    float twob_xx = 2.0f * b_x * SIQ->x;
    float twob_xy = 2.0f * b_x * SIQ->y;
    float twob_xz = 2.0f * b_x * SIQ->z;
    float twob_zw = 2.0f * b_z * SIQ->w;
    float twob_zx = 2.0f * b_z * SIQ->x;
    float twob_zy = 2.0f * b_z * SIQ->y;
    float twob_zz = 2.0f * b_z * SIQ->z;
    float wx;
    float wy = SIQ->w * SIQ->y;
    float wz;
    float xy;
    float xz = SIQ->x * SIQ->z;
    float yz;
    float twom_x = 2.0f * mx;
    float twom_y = 2.0f * my;
    float twom_z = 2.0f * mz;
    // normalise the accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    ax /= norm;
    ay /= norm;
    az /= norm;
    // normalise the magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    mx /= norm;
    my /= norm;
    mz /= norm;
    // compute the objective function and Jacobian
    f[0] = twox * SIQ->z - twow * SIQ->y - ax;
    f[1] = twow * SIQ->x + twoy * SIQ->z - ay;
    f[2] = 1.0f - twox * SIQ->x - twoy * SIQ->y - az;
    f[3] = twob_x * (0.5f - SIQ->y * SIQ->y - SIQ->z * SIQ->z) + twob_z * (xz - wy) - mx;
    f[4] = twob_x * (SIQ->x * SIQ->y - SIQ->w * SIQ->z) + twob_z * (SIQ->w * SIQ->x + SIQ->y * SIQ->z) - my;
    f[5] = twob_x * (wy + xz) + twob_z * (0.5f - SIQ->x * SIQ->x - SIQ->y * SIQ->y) - mz;
    J_11or24 = twoy; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SIQ->z;
    J_13or22 = twow; // J_12 negated in matrix multiplication
    J_14or21 = twox;
    J_32 = 2.0f * J_14or21; // negated in matrix multiplication
    J_33 = 2.0f * J_11or24; // negated in matrix multiplication
    J_41 = twob_zy; // negated in matrix multiplication
    J_42 = twob_zz;
    J_43 = 2.0f * twob_xy + twob_zw; // negated in matrix multiplication
    J_44 = 2.0f * twob_xz - twob_zx; // negated in matrix multiplication
    J_51 = twob_xz - twob_zx; // negated in matrix multiplication
    J_52 = twob_xy + twob_zw;
    J_53 = twob_xx + twob_zz;
    J_54 = twob_xw - twob_zy; // negated in matrix multiplication
    J_61 = twob_xy;
    J_62 = twob_xz - 2.0f * twob_zx;
    J_63 = twob_xw - 2.0f * twob_zy;
    J_64 = twob_xx;
    // compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f[1] - J_11or24 * f[0] - J_41 * f[3] - J_51 * f[4] + J_61 * f[5];
    SEqHatDot_2 = J_12or23 * f[0] + J_13or22 * f[1] - J_32 * f[2] + J_42 * f[3] + J_52 * f[4] + J_62 * f[5];
    SEqHatDot_3 = J_12or23 * f[1] - J_33 * f[2] - J_13or22 * f[0] - J_43 * f[3] + J_53 * f[4] + J_63 * f[5];
    SEqHatDot_4 = J_14or21 * f[0] + J_11or24 * f[1] - J_44 * f[3] - J_54 * f[4] + J_64 * f[5];
    // normalise the gradient to estimate direction of the gyroscope error
    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 = SEqHatDot_1 / norm;
    SEqHatDot_2 = SEqHatDot_2 / norm;
    SEqHatDot_3 = SEqHatDot_3 / norm;
    SEqHatDot_4 = SEqHatDot_4 / norm;
    // compute angular estimated direction of the gyroscope error
    w_err_x = twow * SEqHatDot_2 - twox * SEqHatDot_1 - twoy * SEqHatDot_4 + twoz * SEqHatDot_3;
    w_err_y = twow * SEqHatDot_3 + twox * SEqHatDot_4 - twoy * SEqHatDot_1 - twoz * SEqHatDot_2;
    w_err_z = twow * SEqHatDot_4 - twox * SEqHatDot_3 + twoy * SEqHatDot_2 - twoz * SEqHatDot_1;
    // compute and remove the gyroscope baises
    w_bx += w_err_x * dT * zeta;
    w_by += w_err_y * dT * zeta;
    w_bz += w_err_z * dT * zeta;
    gx -= w_bx;
    gy -= w_by;
    gz -= w_bz;
    // compute the quaternion rate measured by gyroscopes
    SEqDot_omega[0] = -halfx * gx - halfy * gy - halfz * gz;
    SEqDot_omega[1] = halfw * gx + halfy * gz - halfz * gy;
    SEqDot_omega[2] = halfw * gy - halfx * gz + halfz * gx;
    SEqDot_omega[3] = halfw * gz + halfx * gy - halfy * gx;
    // compute then integrate the estimated quaternion rate
    SIQ->w += (SEqDot_omega[0] - (beta * SEqHatDot_1)) * dT / 1000;
    SIQ->x += (SEqDot_omega[1] - (beta * SEqHatDot_2)) * dT / 1000;
    SIQ->y += (SEqDot_omega[2] - (beta * SEqHatDot_3)) * dT / 1000;
    SIQ->z += (SEqDot_omega[3] - (beta * SEqHatDot_4)) * dT / 1000;
    // normalise quaternion
    norm = sqrt(SIQ->w * SIQ->w + SIQ->x * SIQ->x + SIQ->y * SIQ->y + SIQ->z * SIQ->z);
    SIQ->w /= norm;
    SIQ->x /= norm;
    SIQ->y /= norm;
    SIQ->z /= norm;
    // compute flux in the earth frame
    wx = SIQ->w * SIQ->x; // recompute axulirary variables
    wy = SIQ->w * SIQ->y;
    wz = SIQ->w * SIQ->z;
    yz = SIQ->y * SIQ->z;
    xy = SIQ->x * SIQ->y;
    xz = SIQ->x * SIQ->z;
    h_x = twom_x * (0.5f - SIQ->y * SIQ->y - SIQ->z * SIQ->z) + twom_y * (xy - wz) + twom_z * (xz + wy);
    h_y = twom_x * (xy + wz) + twom_y * (0.5f - SIQ->x * SIQ->x - SIQ->z * SIQ->z) + twom_z * (yz - wx);
    h_z = twom_x * (xz - wy) + twom_y * (yz + wx) + twom_z * (0.5f - SIQ->x * SIQ->x - SIQ->y * SIQ->y);
    // normalise the flux vector to have only components in the x and z
    b_x = sqrt((h_x * h_x) + (h_y * h_y));
    b_z = h_z;
    return 1;
}

/*****************************************************************************************
*                   MAHONY                                                               *
*****************************************************************************************/
uint8_t MahonyAHRS_9(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float twoKp, float twoKi, uint32_t dT, Quaternion_t *SIQ) {
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		return MahonyAHRS_6(gx, gy, gz, ax, ay, az, twoKp, twoKi, dT, SIQ);
	}
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     
		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   
        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = SIQ->w * SIQ->w;
        q0q1 = SIQ->w * SIQ->x;
        q0q2 = SIQ->w * SIQ->y;
        q0q3 = SIQ->w * SIQ->z;
        q1q1 = SIQ->x * SIQ->x;
        q1q2 = SIQ->x * SIQ->y;
        q1q3 = SIQ->x * SIQ->z;
        q2q2 = SIQ->y * SIQ->y;
        q2q3 = SIQ->y * SIQ->z;
        q3q3 = SIQ->z * SIQ->z;   
        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFB.x += twoKi * halfex * dT / 1000.0f;	// integral error scaled by Ki
			integralFB.y += twoKi * halfey * dT / 1000.0f;
			integralFB.z += twoKi * halfez * dT / 1000.0f;
			gx += integralFB.x;	// apply integral feedback
			gy += integralFB.y;
			gz += integralFB.z;
		}
		else {
			integralFB.x = 0.0f;	// prevent integral windup
			integralFB.y = 0.0f;
			integralFB.z = 0.0f;
		}
		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	// Integrate rate of change of quaternion
	gx *= (0.5f * dT / 1000.0f);		// pre-multiply common factors
	gy *= (0.5f * dT / 1000.0f);
	gz *= (0.5f * dT / 1000.0f);
	qa = SIQ->w;
	qb = SIQ->x;
	qc = SIQ->y;
	SIQ->w += (-qb * gx - qc * gy - SIQ->z * gz);
	SIQ->x += (qa * gx + qc * gz - SIQ->z * gy);
	SIQ->y += (qa * gy - qb * gz + SIQ->z * gx);
	SIQ->z += (qa * gz + qb * gy - qc * gx); 
	// Normalise quaternion
	recipNorm = invSqrt(SIQ->w * SIQ->w + SIQ->x * SIQ->x + SIQ->y * SIQ->y + SIQ->z * SIQ->z);
	SIQ->w *= recipNorm;
	SIQ->x *= recipNorm;
	SIQ->y *= recipNorm;
	SIQ->z *= recipNorm;
	return 1;
}

uint8_t MahonyAHRS_6(float ax, float ay, float az, float gx, float gy, float gz, float twoKp, float twoKi, uint32_t dT, Quaternion_t *SIQ) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        
		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = SIQ->x * SIQ->z - SIQ->w * SIQ->y;
		halfvy = SIQ->w * SIQ->x + SIQ->y * SIQ->z;
		halfvz = SIQ->w * SIQ->w - 0.5f + SIQ->z * SIQ->z;
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFB.x += twoKi * halfex * dT / 1000.0f;	// integral error scaled by Ki
			integralFB.y += twoKi * halfey * dT / 1000.0f;
			integralFB.z += twoKi * halfez * dT / 1000.0f;
			gx += integralFB.x;	// apply integral feedback
			gy += integralFB.y;
			gz += integralFB.z;
		}
		else {
			integralFB.x = 0.0f;	// prevent integral windup
			integralFB.y = 0.0f;
			integralFB.z = 0.0f;
		}
		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	// Integrate rate of change of quaternion
	gx *= (0.5f * dT / 1000.0f);		// pre-multiply common factors
	gy *= (0.5f * dT / 1000.0f);
	gz *= (0.5f * dT / 1000.0f);
	qa = SIQ->w;
	qb = SIQ->x;
	qc = SIQ->y;
	SIQ->w += (-qb * gx - qc * gy - SIQ->z * gz);
	SIQ->x += (qa * gx + qc * gz - SIQ->z * gy);
	SIQ->y += (qa * gy - qb * gz + SIQ->z * gx);
	SIQ->z += (qa * gz + qb * gy - qc * gx); 
	// Normalise quaternion
	recipNorm = invSqrt(SIQ->w * SIQ->w + SIQ->x * SIQ->x + SIQ->y * SIQ->y + SIQ->z * SIQ->z);
	SIQ->w *= recipNorm;
	SIQ->x *= recipNorm;
	SIQ->y *= recipNorm;
	SIQ->z *= recipNorm;
	return 1;
}

/*****************************************************************************************
*                   MADGWICK_AHRS                                                        *
*****************************************************************************************/
uint8_t MadgwickAHRS_9(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, uint32_t dT, Quaternion_t *SIQ) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	// old variables
	//float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, 				_2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	// new fix
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _8bx, _8bz, _2q0, _2q1, _2q2, _2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		return MadgwickAHRS_6(ax, ay, az, gx, gy, gz, dT, SIQ);
	}
	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-Q.x * gx - Q.y * gy - Q.z * gz);
	qDot2 = 0.5f * (Q.w * gx + Q.y * gz - Q.z * gy);
	qDot3 = 0.5f * (Q.w * gy - Q.x * gz + Q.z * gx);
	qDot4 = 0.5f * (Q.w * gz + Q.x * gy - Q.y * gx);
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   
		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;
		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * Q.w * mx;
		_2q0my = 2.0f * Q.w * my;
		_2q0mz = 2.0f * Q.w * mz;
		_2q1mx = 2.0f * Q.x * mx;
		_2q0 = 2.0f * Q.w;
		_2q1 = 2.0f * Q.x;
		_2q2 = 2.0f * Q.y;
		_2q3 = 2.0f * Q.z;
		//_2q0q2 = 2.0f * Q.w * Q.y; // new fix
		//_2q2q3 = 2.0f * Q.y * Q.z; // new fix
		q0q0 = Q.w * Q.w;
		q0q1 = Q.w * Q.x;
		q0q2 = Q.w * Q.y;
		q0q3 = Q.w * Q.z;
		q1q1 = Q.x * Q.x;
		q1q2 = Q.x * Q.y;
		q1q3 = Q.x * Q.z;
		q2q2 = Q.y * Q.y;
		q2q3 = Q.y * Q.z;
		q3q3 = Q.z * Q.z;
		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * Q.z + _2q0mz * Q.y + mx * q1q1 + _2q1 * my * Q.y + _2q1 * mz * Q.z - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * Q.z + my * q0q0 - _2q0mz * Q.x + _2q1mx * Q.y - my * q1q1 + my * q2q2 + _2q2 * mz * Q.z - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * Q.y + _2q0my * Q.x + mz * q0q0 + _2q1mx * Q.z - mz * q1q1 + _2q2 * my * Q.z - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;
		_8bx = 2.0f * _4bx;//new fix
		_8bz = 2.0f * _4bz;//new fix
		// Gradient decent algorithm corrective step
		//old
		//s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * Q.y * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * Q.z + _2bz * Q.x) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * Q.y * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		//s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * Q.x * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * Q.z * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * Q.y + _2bz * Q.w) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * Q.z - _4bz * Q.x) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		//s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * Q.y * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * Q.y - _2bz * Q.w) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * Q.x + _2bz * Q.z) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * Q.w - _4bz * Q.y) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		//s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * Q.z + _2bz * Q.x) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * Q.w + _2bz * Q.y) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * Q.x * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		//new fix
		s0 = -_2q2 * (2.0f * (q1q3 - q0q2) - ax) + _2q1 * (2 * (q0q1 + q2q3) - ay) + -_4bz * Q.y * (_4bx * (0.5 - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (-_4bx * Q.z + _4bz * Q.x) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + _4bx * Q.y * (_4bx * (q0q2 + q1q3) + _4bz * (0.5 - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2 * (q1q3 - q0q2) - ax) + _2q0 * (2 * (q0q1 + q2q3) - ay) + -4 * Q.x * (2 * (0.5 - q1q1 - q2q2) - az) + _4bz * Q.z * (_4bx * (0.5 - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (_4bx * Q.y + _4bz * Q.w) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + (_4bx * Q.z - _8bz * Q.x) * (_4bx * (q0q2 + q1q3) + _4bz * (0.5 - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2 * (q1q3 - q0q2) - ax) + _2q3 * (2 * (q0q1 + q2q3) - ay) + (-4 * Q.y) * (2 * (0.5 - q1q1 - q2q2) - az) + (-_8bx * Q.y - _4bz * Q.w) * (_4bx * (0.5 - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (_4bx * Q.x + _4bz * Q.z) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + (_4bx * Q.w - _8bz * Q.y) * (_4bx * (q0q2 + q1q3) + _4bz * (0.5 - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2 * (q1q3 - q0q2) - ax) + _2q2 * (2 * (q0q1 + q2q3) - ay)+(-_8bx * Q.z + _4bz * Q.x) * (_4bx * (0.5 - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (-_4bx * Q.w + _4bz * Q.y) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + (_4bx * Q.x) * (_4bx * (q0q2 + q1q3) + _4bz * (0.5 - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;
		// Apply feedback step
		qDot1 -= betaDef * s0;
		qDot2 -= betaDef * s1;
		qDot3 -= betaDef * s2;
		qDot4 -= betaDef * s3;
	}
	// Integrate rate of change of quaternion to yield quaternion
	Q.w += qDot1 * dT / 1000.0f;
	Q.x += qDot2 * dT / 1000.0f;
	Q.y += qDot3 * dT / 1000.0f;
	Q.z += qDot4 * dT / 1000.0f;
	// Normalise quaternion
	recipNorm = invSqrt(Q.w * Q.w + Q.x * Q.x + Q.y * Q.y + Q.z * Q.z);
	Q.w *= recipNorm;
	Q.x *= recipNorm;
	Q.y *= recipNorm;
	Q.z *= recipNorm;
	return 1;
}

uint8_t MadgwickAHRS_6(float ax, float ay, float az, float gx, float gy, float gz, uint32_t dT, Quaternion_t *SIQ) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-Q.x * gx - Q.y * gy - Q.z * gz);
	qDot2 = 0.5f * (Q.w * gx + Q.y * gz - Q.z * gy);
	qDot3 = 0.5f * (Q.w * gy - Q.x * gz + Q.z * gx);
	qDot4 = 0.5f * (Q.w * gz + Q.x * gy - Q.y * gx);
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   
		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * Q.w;
		_2q1 = 2.0f * Q.x;
		_2q2 = 2.0f * Q.y;
		_2q3 = 2.0f * Q.z;
		_4q0 = 4.0f * Q.w;
		_4q1 = 4.0f * Q.x;
		_4q2 = 4.0f * Q.y;
		_8q1 = 8.0f * Q.x;
		_8q2 = 8.0f * Q.y;
		q0q0 = Q.w * Q.w;
		q1q1 = Q.x * Q.x;
		q2q2 = Q.y * Q.y;
		q3q3 = Q.z * Q.z;
		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * Q.x - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * Q.y + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * Q.z - _2q1 * ax + 4.0f * q2q2 * Q.z - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;
		// Apply feedback step
		qDot1 -= betaDef * s0;
		qDot2 -= betaDef * s1;
		qDot3 -= betaDef * s2;
		qDot4 -= betaDef * s3;
	}
	// Integrate rate of change of quaternion to yield quaternion
	Q.w += qDot1 * dT / 1000.0f;
	Q.x += qDot2 * dT / 1000.0f;
	Q.y += qDot3 * dT / 1000.0f;
	Q.z += qDot4 * dT / 1000.0f;
	// Normalise quaternion
	recipNorm = invSqrt(Q.w * Q.w + Q.x * Q.x + Q.y * Q.y + Q.z * Q.z);
	Q.w *= recipNorm;
	Q.x *= recipNorm;
	Q.y *= recipNorm;
	Q.z *= recipNorm;
	return 1;
}

/*****************************************************************************************
*                   CONVERSION                                                           *
*****************************************************************************************/
Quaternion_t EulerAngleToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{   // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    Quaternion_t q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

EulerAngles_t QuaternionToEulerAngles(Quaternion_t q, uint8_t deg) {
    EulerAngles_t angles;
    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);
    // pitch (y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1.0)
        angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asin(sinp);
    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = atan2(siny_cosp, cosy_cosp);
	if (deg)
	{
		angles.pitch *= (180.0 / M_PI);
		angles.roll *= (180.0 / M_PI);
		angles.yaw *= (180.0 / M_PI);
		if (angles.pitch < 0) angles.pitch = 360.0f + angles.pitch;
		if (angles.roll < 0) angles.roll = 360.0f + angles.roll;
		if (angles.yaw < 0) angles.yaw = 360.0f + angles.yaw;
	}
    return angles;
}
