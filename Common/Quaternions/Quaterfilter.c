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
*********************************************************************************/

#include "QuaterFilter.h"

// System constants
#define gyroMeasError (M_PI * (5.0f / 180.0f))      // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasDrift (M_PI * (0.2f / 180.0f))      // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define beta (sqrt(3.0f / 4.0f) * gyroMeasError)    // compute beta
#define zeta (sqrt(3.0f / 4.0f) * gyroMeasDrift)    // compute zeta


static Quaternion_t SIQ = {1.0, 0.0, 0.0, 0.0}; // static internal quaternion
float b_x = 1, b_z = 0;                         // reference direction of flux in earth frame
float w_bx = 0, w_by = 0, w_bz = 0;             // estimate gyroscope biases error

Quaternion_t FilterUpdate_6DOF(Axis_t accel, Axis_t gyro, uint32_t deltat) {
    // Local system variables
    float norm;             // vector norm
    float SEqDot_omega[4];  // quaternion derrivative from gyroscopes elements
    float f[3];             // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float SEqHatDot[4]; // estimated direction of the gyroscope error
    // Axulirary variables to avoid reapeated calcualtions
    float halfSEq[4];
    halfSEq[0] = 0.5f * SIQ.w; halfSEq[1] = 0.5f * SIQ.x;
    halfSEq[2] = 0.5f * SIQ.y; halfSEq[3] = 0.5f * SIQ.z;
    float twow = 2.0f * SIQ.w;
    float twox = 2.0f * SIQ.x;
    float twoy = 2.0f * SIQ.y;
    // Normalise the accelerometer measurement
    norm = sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    accel.x /= norm;
    accel.y /= norm;
    accel.z /= norm;
    // Compute the objective function and Jacobian
    f[0] = twox * SIQ.z - twow * SIQ.y - accel.x;
    f[1] = twow * SIQ.x + twoy * SIQ.z - accel.y;
    f[2] = 1.0f - twox * SIQ.x - twoy * SIQ.y - accel.z;
    J_11or24 = twoy; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SIQ.z;
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
    SIQ.w += (SEqDot_omega[0] - (beta * SEqHatDot[0])) * deltat / 1000;
    SIQ.x += (SEqDot_omega[1] - (beta * SEqHatDot[1])) * deltat / 1000;
    SIQ.y += (SEqDot_omega[2] - (beta * SEqHatDot[2])) * deltat / 1000;
    SIQ.z += (SEqDot_omega[3] - (beta * SEqHatDot[3])) * deltat / 1000;
    // Normalise quaternion
    norm = sqrt(SIQ.w * SIQ.w + SIQ.x * SIQ.x + SIQ.y *SIQ.y + SIQ.z * SIQ.z);
    SIQ.w /= norm;
    SIQ.x /= norm;
    SIQ.y /= norm;
    SIQ.z /= norm;
    return SIQ;
}

Quaternion_t FilterUpdate_6(float a_x, float a_y, float a_z, float g_x, float g_y, float g_z, uint32_t deltat) {
    // Local system variables
    float norm;             // vector norm
    float SEqDot_omega[4];  // quaternion derrivative from gyroscopes elements
    float f[3];             // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float SEqHatDot[4]; // estimated direction of the gyroscope error
    // Axulirary variables to avoid reapeated calcualtions
    float halfSEq[4];
    halfSEq[0] = 0.5f * SIQ.w; halfSEq[1] = 0.5f * SIQ.x;
    halfSEq[2] = 0.5f * SIQ.y; halfSEq[3] = 0.5f * SIQ.z;
    float twow = 2.0f * SIQ.w;
    float twox = 2.0f * SIQ.x;
    float twoy = 2.0f * SIQ.y;
    // Normalise the accelerometer measurement
    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= norm;
    a_y /= norm;
    a_z /= norm;
    // Compute the objective function and Jacobian
    f[0] = twox * SIQ.z - twow * SIQ.y - a_x;
    f[1] = twow * SIQ.x + twoy * SIQ.z - a_y;
    f[2] = 1.0f - twox * SIQ.x - twoy * SIQ.y - a_z;
    J_11or24 = twoy; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SIQ.z;
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
    SEqDot_omega[0] = -halfSEq[1] * g_x - halfSEq[2] * g_y - halfSEq[3] * g_z;
    SEqDot_omega[1] = halfSEq[0] * g_x + halfSEq[2] * g_z - halfSEq[3] * g_y;
    SEqDot_omega[2] = halfSEq[0] * g_y - halfSEq[1] * g_z + halfSEq[3] * g_x;
    SEqDot_omega[3] = halfSEq[0] * g_z + halfSEq[1] * g_y - halfSEq[2] * g_x;
    // Compute then integrate the estimated quaternion derrivative
    SIQ.w += (SEqDot_omega[0] - (beta * SEqHatDot[0])) * deltat / 1000;
    SIQ.x += (SEqDot_omega[1] - (beta * SEqHatDot[1])) * deltat / 1000;
    SIQ.y += (SEqDot_omega[2] - (beta * SEqHatDot[2])) * deltat / 1000;
    SIQ.z += (SEqDot_omega[3] - (beta * SEqHatDot[3])) * deltat / 1000;
    // Normalise quaternion
    norm = sqrt(SIQ.w * SIQ.w + SIQ.x * SIQ.x + SIQ.y *SIQ.y + SIQ.z * SIQ.z);
    SIQ.w /= norm;
    SIQ.x /= norm;
    SIQ.y /= norm;
    SIQ.z /= norm;
    return SIQ;
}

// Function to compute one filter iteration
Quaternion_t FilterUpdate_9DOF(Axis_t accel, Axis_t gyro, Axis_t mag, uint32_t deltat) {
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
    float halfw = 0.5f * SIQ.w;
    float halfx = 0.5f * SIQ.x;
    float halfy = 0.5f * SIQ.y;
    float halfz = 0.5f * SIQ.z;
    float twow = 2.0f * SIQ.w;
    float twox = 2.0f * SIQ.x;
    float twoy = 2.0f * SIQ.y;
    float twoz = 2.0f * SIQ.z;
    float twob_x = 2.0f * b_x;
    float twob_z = 2.0f * b_z;
    float twob_xw = 2.0f * b_x * SIQ.w;
    float twob_xx = 2.0f * b_x * SIQ.x;
    float twob_xy = 2.0f * b_x * SIQ.y;
    float twob_xz = 2.0f * b_x * SIQ.z;
    float twob_zw = 2.0f * b_z * SIQ.w;
    float twob_zx = 2.0f * b_z * SIQ.x;
    float twob_zy = 2.0f * b_z * SIQ.y;
    float twob_zz = 2.0f * b_z * SIQ.z;
    float wx;
    float wy = SIQ.w * SIQ.y;
    float wz;
    float xy;
    float xz = SIQ.x * SIQ.z;
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
    f[0] = twox * SIQ.z - twow * SIQ.y - accel.x;
    f[1] = twow * SIQ.x + twoy * SIQ.z - accel.y;
    f[2] = 1.0f - twox * SIQ.x - twoy * SIQ.y - accel.z;
    f[3] = twob_x * (0.5f - SIQ.y * SIQ.y - SIQ.z * SIQ.z) + twob_z * (xz - wy) - mag.x;
    f[4] = twob_x * (SIQ.x * SIQ.y - SIQ.w * SIQ.z) + twob_z * (SIQ.w * SIQ.x + SIQ.y * SIQ.z) - mag.y;
    f[5] = twob_x * (wy + xz) + twob_z * (0.5f - SIQ.x * SIQ.x - SIQ.y * SIQ.y) - mag.z;
    J_11or24 = twoy; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SIQ.z;
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
    w_bx += w_err_x * deltat * zeta;
    w_by += w_err_y * deltat * zeta;
    w_bz += w_err_z * deltat * zeta;
    gyro.x -= w_bx;
    gyro.y -= w_by;
    gyro.z -= w_bz;
    // compute the quaternion rate measured by gyroscopes
    SEqDot_omega[0] = -halfx * gyro.x - halfy * gyro.y - halfz * gyro.z;
    SEqDot_omega[1] = halfw * gyro.x + halfy * gyro.z - halfz * gyro.y;
    SEqDot_omega[2] = halfw * gyro.y - halfx * gyro.z + halfz * gyro.x;
    SEqDot_omega[3] = halfw * gyro.z + halfx * gyro.y - halfy * gyro.x;
    // compute then integrate the estimated quaternion rate
    SIQ.w += (SEqDot_omega[0] - (beta * SEqHatDot_1)) * deltat / 1000;
    SIQ.x += (SEqDot_omega[1] - (beta * SEqHatDot_2)) * deltat / 1000;
    SIQ.y += (SEqDot_omega[2] - (beta * SEqHatDot_3)) * deltat / 1000;
    SIQ.z += (SEqDot_omega[3] - (beta * SEqHatDot_4)) * deltat / 1000;
    // normalise quaternion
    norm = sqrt(SIQ.w * SIQ.w + SIQ.x * SIQ.x + SIQ.y * SIQ.y + SIQ.z * SIQ.z);
    SIQ.w /= norm;
    SIQ.x /= norm;
    SIQ.y /= norm;
    SIQ.z /= norm;
    // compute flux in the earth frame
    wx = SIQ.w * SIQ.x; // recompute axulirary variables
    wy = SIQ.w * SIQ.y;
    wz = SIQ.w * SIQ.z;
    yz = SIQ.y * SIQ.z;
    xy = SIQ.x * SIQ.y;
    xz = SIQ.x * SIQ.z;
    h_x = twom_x * (0.5f - SIQ.y * SIQ.y - SIQ.z * SIQ.z) + twom_y * (xy - wz) + twom_z * (xz + wy);
    h_y = twom_x * (xy + wz) + twom_y * (0.5f - SIQ.x * SIQ.x - SIQ.z * SIQ.z) + twom_z * (yz - wx);
    h_z = twom_x * (xz - wy) + twom_y * (yz + wx) + twom_z * (0.5f - SIQ.x * SIQ.x - SIQ.y * SIQ.y);
    // normalise the flux vector to have only components in the x and z
    b_x = sqrt((h_x * h_x) + (h_y * h_y));
    b_z = h_z;
    return SIQ;
}

Quaternion_t FilterUpdate9(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z, uint32_t deltat) {
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
    float halfw = 0.5f * SIQ.w;
    float halfx = 0.5f * SIQ.x;
    float halfy = 0.5f * SIQ.y;
    float halfz = 0.5f * SIQ.z;
    float twow = 2.0f * SIQ.w;
    float twox = 2.0f * SIQ.x;
    float twoy = 2.0f * SIQ.y;
    float twoz = 2.0f * SIQ.z;
    float twob_x = 2.0f * b_x;
    float twob_z = 2.0f * b_z;
    float twob_xw = 2.0f * b_x * SIQ.w;
    float twob_xx = 2.0f * b_x * SIQ.x;
    float twob_xy = 2.0f * b_x * SIQ.y;
    float twob_xz = 2.0f * b_x * SIQ.z;
    float twob_zw = 2.0f * b_z * SIQ.w;
    float twob_zx = 2.0f * b_z * SIQ.x;
    float twob_zy = 2.0f * b_z * SIQ.y;
    float twob_zz = 2.0f * b_z * SIQ.z;
    float wx;
    float wy = SIQ.w * SIQ.y;
    float wz;
    float xy;
    float xz = SIQ.x * SIQ.z;
    float yz;
    float twom_x = 2.0f * m_x;
    float twom_y = 2.0f * m_y;
    float twom_z = 2.0f * m_z;
    // normalise the accelerometer measurement
    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= norm;
    a_y /= norm;
    a_z /= norm;
    // normalise the magnetometer measurement
    norm = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
    m_x /= norm;
    m_y /= norm;
    m_z /= norm;
    // compute the objective function and Jacobian
    f[0] = twox * SIQ.z - twow * SIQ.y - a_x;
    f[1] = twow * SIQ.x + twoy * SIQ.z - a_y;
    f[2] = 1.0f - twox * SIQ.x - twoy * SIQ.y - a_z;
    f[3] = twob_x * (0.5f - SIQ.y * SIQ.y - SIQ.z * SIQ.z) + twob_z * (xz - wy) - m_x;
    f[4] = twob_x * (SIQ.x * SIQ.y - SIQ.w * SIQ.z) + twob_z * (SIQ.w * SIQ.x + SIQ.y * SIQ.z) - m_y;
    f[5] = twob_x * (wy + xz) + twob_z * (0.5f - SIQ.x * SIQ.x - SIQ.y * SIQ.y) - m_z;
    J_11or24 = twoy; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SIQ.z;
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
    w_bx += w_err_x * deltat * zeta;
    w_by += w_err_y * deltat * zeta;
    w_bz += w_err_z * deltat * zeta;
    w_x -= w_bx;
    w_y -= w_by;
    w_z -= w_bz;
    // compute the quaternion rate measured by gyroscopes
    SEqDot_omega[0] = -halfx * w_x - halfy * w_y - halfz * w_z;
    SEqDot_omega[1] = halfw * w_x + halfy * w_z - halfz * w_y;
    SEqDot_omega[2] = halfw * w_y - halfx * w_z + halfz * w_x;
    SEqDot_omega[3] = halfw * w_z + halfx * w_y - halfy * w_x;
    // compute then integrate the estimated quaternion rate
    SIQ.w += (SEqDot_omega[0] - (beta * SEqHatDot_1)) * deltat / 1000;
    SIQ.x += (SEqDot_omega[1] - (beta * SEqHatDot_2)) * deltat / 1000;
    SIQ.y += (SEqDot_omega[2] - (beta * SEqHatDot_3)) * deltat / 1000;
    SIQ.z += (SEqDot_omega[3] - (beta * SEqHatDot_4)) * deltat / 1000;
    // normalise quaternion
    norm = sqrt(SIQ.w * SIQ.w + SIQ.x * SIQ.x + SIQ.y * SIQ.y + SIQ.z * SIQ.z);
    SIQ.w /= norm;
    SIQ.x /= norm;
    SIQ.y /= norm;
    SIQ.z /= norm;
    // compute flux in the earth frame
    wx = SIQ.w * SIQ.x; // recompute axulirary variables
    wy = SIQ.w * SIQ.y;
    wz = SIQ.w * SIQ.z;
    yz = SIQ.y * SIQ.z;
    xy = SIQ.x * SIQ.y;
    xz = SIQ.x * SIQ.z;
    h_x = twom_x * (0.5f - SIQ.y * SIQ.y - SIQ.z * SIQ.z) + twom_y * (xy - wz) + twom_z * (xz + wy);
    h_y = twom_x * (xy + wz) + twom_y * (0.5f - SIQ.x * SIQ.x - SIQ.z * SIQ.z) + twom_z * (yz - wx);
    h_z = twom_x * (xz - wy) + twom_y * (yz + wx) + twom_z * (0.5f - SIQ.x * SIQ.x - SIQ.y * SIQ.y);
    // normalise the flux vector to have only components in the x and z
    b_x = sqrt((h_x * h_x) + (h_y * h_y));
    b_z = h_z;
    return SIQ;
}

Quaternion_t EulerAngleToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
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
