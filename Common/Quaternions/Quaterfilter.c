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
   
 	QuaterFilter.c
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

#include "QuaterFilter.h"

// System constants
static const float deltat = 0.001; // sampling period in seconds (shown as 1 ms)
static const float gyroMeasError = M_PI * (5.0f / 180.0f); // gyroscope measurement error in rad/s (shown as 5 deg/s)
static const float gyroMeasDrift = M_PI * (0.2f / 180.0f); // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
static const float beta = sqrt(3.0f / 4.0f) * gyroMeasError; // compute beta
static const float zeta = sqrt(3.0f / 4.0f) * gyroMeasDrift; // compute zeta

// Global system variables
Quat_DOF SEQ = {1.0, 0.0, 0.0, 0.0};
//float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f; // estimated orientation quaternion elements with initial conditions
float b_x = 1, b_z = 0; // reference direction of flux in earth frame
float w_bx = 0, w_by = 0, w_bz = 0; // estimate gyroscope biases error

/*
Следующий исходный код является реализацией фильтра ориентации без магнитометра. 
код был оптимизирован чтобы минимизировать необходимое количество арифметических операций за счет оперативной памяти.
На каждое обновление фильтра требуется 109 скалярных арифметических операций 
(18 операций сложения, 20 вычитания, 57 умножений, 11 делений и 3 получения квадратного корня).
40 байт оперативной памяти для глобальных переменных 
100 байт оперативной памяти для локальных переменных во время каждого вызова функции обновления фильтра.
*/
void FilterUpdate_6DOF(AccelAxis *accel, GyroAxis *gyro) {
    // Local system variables
    float norm; // vector norm
    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
    float f_1, f_2, f_3; // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
    // Axulirary variables to avoid reapeated calcualtions
    float halfSEq_1 = 0.5f * SEQ.SEq_1;
    float halfSEq_2 = 0.5f * SEQ.SEq_2;
    float halfSEq_3 = 0.5f * SEQ.SEq_3;
    float halfSEq_4 = 0.5f * SEQ.SEq_4;
    float twoSEq_1 = 2.0f * SEQ.SEq_1;
    float twoSEq_2 = 2.0f * SEQ.SEq_2;
    float twoSEq_3 = 2.0f * SEQ.SEq_3;
    // Normalise the accelerometer measurement
    norm = sqrt(accel->a_x * accel->a_x + accel->a_y * accel->a_y + accel->a_z * accel->a_z);
    accel->a_x /= norm;
    accel->a_y /= norm;
    accel->a_z /= norm;
    // Compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEQ.SEq_4 - twoSEq_1 * SEQ.SEq_3 - accel->a_x;
    f_2 = twoSEq_1 * SEQ.SEq_2 + twoSEq_3 * SEQ.SEq_4 - accel->a_y;
    f_3 = 1.0f - twoSEq_2 * SEQ.SEq_2 - twoSEq_3 * SEQ.SEq_3 - accel->a_z;
    J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SEQ.SEq_4;
    J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2;
    J_32 = 2.0f * J_14or21; // negated in matrix multiplication
    J_33 = 2.0f * J_11or24; // negated in matrix multiplication
    // Compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
    // Normalise the gradient
    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 /= norm;
    SEqHatDot_2 /= norm;
    SEqHatDot_3 /= norm;
    SEqHatDot_4 /= norm;
    // Compute the quaternion derrivative measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * gyro->w_x - halfSEq_3 * gyro->w_y - halfSEq_4 * gyro->w_z;
    SEqDot_omega_2 = halfSEq_1 * gyro->w_x + halfSEq_3 * gyro->w_z - halfSEq_4 * gyro->w_y;
    SEqDot_omega_3 = halfSEq_1 * gyro->w_y - halfSEq_2 * gyro->w_z + halfSEq_4 * gyro->w_x;
    SEqDot_omega_4 = halfSEq_1 * gyro->w_z + halfSEq_2 * gyro->w_y - halfSEq_3 * gyro->w_x;
    // Compute then integrate the estimated quaternion derrivative
    SEQ.SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
    SEQ.SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
    SEQ.SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
    SEQ.SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
    // Normalise quaternion
    norm = sqrt(SEQ.SEq_1 * SEQ.SEq_1 + SEQ.SEq_2 * SEQ.SEq_2 + SEQ.SEq_3 *SEQ.SEq_3 + SEQ.SEq_4 * SEQ.SEq_4);
    SEQ.SEq_1 /= norm;
    SEQ.SEq_2 /= norm;
    SEQ.SEq_3 /= norm;
    SEQ.SEq_4 /= norm;
}

void FilterUpdate6(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z) {
    // Local system variables
    float norm; // vector norm
    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
    float f_1, f_2, f_3; // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
    // Axulirary variables to avoid reapeated calcualtions
    float halfSEq_1 = 0.5f * SEQ.SEq_1;
    float halfSEq_2 = 0.5f * SEQ.SEq_2;
    float halfSEq_3 = 0.5f * SEQ.SEq_3;
    float halfSEq_4 = 0.5f * SEQ.SEq_4;
    float twoSEq_1 = 2.0f * SEQ.SEq_1;
    float twoSEq_2 = 2.0f * SEQ.SEq_2;
    float twoSEq_3 = 2.0f * SEQ.SEq_3;
    // Normalise the accelerometer measurement
    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= norm;
    a_y /= norm;
    a_z /= norm;
    // Compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEQ.SEq_4 - twoSEq_1 * SEQ.SEq_3 - a_x;
    f_2 = twoSEq_1 * SEQ.SEq_2 + twoSEq_3 * SEQ.SEq_4 - a_y;
    f_3 = 1.0f - twoSEq_2 * SEQ.SEq_2 - twoSEq_3 * SEQ.SEq_3 - a_z;
    J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SEQ.SEq_4;
    J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2;
    J_32 = 2.0f * J_14or21; // negated in matrix multiplication
    J_33 = 2.0f * J_11or24; // negated in matrix multiplication
    // Compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
    // Normalise the gradient
    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 /= norm;
    SEqHatDot_2 /= norm;
    SEqHatDot_3 /= norm;
    SEqHatDot_4 /= norm;
    // Compute the quaternion derrivative measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
    // Compute then integrate the estimated quaternion derrivative
    SEQ.SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
    SEQ.SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
    SEQ.SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
    SEQ.SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
    // Normalise quaternion
    norm = sqrt(SEQ.SEq_1 * SEQ.SEq_1 + SEQ.SEq_2 * SEQ.SEq_2 + SEQ.SEq_3 * SEQ.SEq_3 + SEQ.SEq_4 * SEQ.SEq_4);
    SEQ.SEq_1 /= norm;
    SEQ.SEq_2 /= norm;
    SEQ.SEq_3 /= norm;
    SEQ.SEq_4 /= norm;
}
//========================================================================

/*
Следующий исходный код является реализацией фильтра ориентации с магнитометром и системой компенсации дрейфа гироскопа.
код был оптимизирован чтобы минимизировать необходимое количество арифметических операций за счет оперативной памяти.
На каждое обновление фильтра требуется 277 скалярных арифметических операций
(51 операция сложения, 57 вычитания, 155 умножений, 14 делений и 5 получений квадратного корня).
требует 72 байта оперативной памяти для глобальных переменных
260 байт оперативной памяти для локальных переменных во время каждого вызова функции обновления фильтра.
*/
// Function to compute one filter iteration
void FilterUpdate_9DOF(AccelAxis *accel, GyroAxis *gyro, MagAxis *mag) {
    // local system variables
    float norm; // vector norm
    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion rate from gyroscopes elements
    float f_1, f_2, f_3, f_4, f_5, f_6; // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, // objective function Jacobian elements
    J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
    float w_err_x, w_err_y, w_err_z; // estimated direction of the gyroscope error (angular)
    float h_x, h_y, h_z; // computed flux in the earth frame
    // axulirary variables to avoid reapeated calcualtions
    float halfSEq_1 = 0.5f * SEQ.SEq_1;
    float halfSEq_2 = 0.5f * SEQ.SEq_2;
    float halfSEq_3 = 0.5f * SEQ.SEq_3;
    float halfSEq_4 = 0.5f * SEQ.SEq_4;
    float twoSEq_1 = 2.0f * SEQ.SEq_1;
    float twoSEq_2 = 2.0f * SEQ.SEq_2;
    float twoSEq_3 = 2.0f * SEQ.SEq_3;
    float twoSEq_4 = 2.0f * SEQ.SEq_4;
    float twob_x = 2.0f * b_x;
    float twob_z = 2.0f * b_z;
    float twob_xSEq_1 = 2.0f * b_x * SEQ.SEq_1;
    float twob_xSEq_2 = 2.0f * b_x * SEQ.SEq_2;
    float twob_xSEq_3 = 2.0f * b_x * SEQ.SEq_3;
    float twob_xSEq_4 = 2.0f * b_x * SEQ.SEq_4;
    float twob_zSEq_1 = 2.0f * b_z * SEQ.SEq_1;
    float twob_zSEq_2 = 2.0f * b_z * SEQ.SEq_2;
    float twob_zSEq_3 = 2.0f * b_z * SEQ.SEq_3;
    float twob_zSEq_4 = 2.0f * b_z * SEQ.SEq_4;
    float SEq_1SEq_2;
    float SEq_1SEq_3 = SEQ.SEq_1 * SEQ.SEq_3;
    float SEq_1SEq_4;
    float SEq_2SEq_3;
    float SEq_2SEq_4 = SEQ.SEq_2 * SEQ.SEq_4;
    float SEq_3SEq_4;
    float twom_x = 2.0f * mag->m_x;
    float twom_y = 2.0f * mag->m_y;
    float twom_z = 2.0f * mag->m_z;
    // normalise the accelerometer measurement
    norm = sqrt(accel->a_x * accel->a_x + accel->a_y * accel->a_y + accel->a_z * accel->a_z);
    accel->a_x /= norm;
    accel->a_y /= norm;
    accel->a_z /= norm;
    // normalise the magnetometer measurement
    norm = sqrt(mag->m_x * mag->m_x + mag->m_y * mag->m_y + mag->m_z * mag->m_z);
    mag->m_x /= norm;
    mag->m_y /= norm;
    mag->m_z /= norm;
    // compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEQ.SEq_4 - twoSEq_1 * SEQ.SEq_3 - accel->a_x;
    f_2 = twoSEq_1 * SEQ.SEq_2 + twoSEq_3 * SEQ.SEq_4 - accel->a_y;
    f_3 = 1.0f - twoSEq_2 * SEQ.SEq_2 - twoSEq_3 * SEQ.SEq_3 - accel->a_z;
    f_4 = twob_x * (0.5f - SEQ.SEq_3 * SEQ.SEq_3 - SEQ.SEq_4 * SEQ.SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - mag->m_x;
    f_5 = twob_x * (SEQ.SEq_2 * SEQ.SEq_3 - SEQ.SEq_1 * SEQ.SEq_4) + twob_z * (SEQ.SEq_1 * SEQ.SEq_2 + SEQ.SEq_3 * SEQ.SEq_4) - mag->m_y;
    f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - SEQ.SEq_2 * SEQ.SEq_2 - SEQ.SEq_3 * SEQ.SEq_3) - mag->m_z;
    J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SEQ.SEq_4;
    J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2;
    J_32 = 2.0f * J_14or21; // negated in matrix multiplication
    J_33 = 2.0f * J_11or24; // negated in matrix multiplication
    J_41 = twob_zSEq_3; // negated in matrix multiplication
    J_42 = twob_zSEq_4;
    J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication
    J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
    J_51 = twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
    J_52 = twob_xSEq_3 + twob_zSEq_1;
    J_53 = twob_xSEq_2 + twob_zSEq_4;
    J_54 = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication
    J_61 = twob_xSEq_3;
    J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
    J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
    J_64 = twob_xSEq_2;
    // compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
    // normalise the gradient to estimate direction of the gyroscope error
    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 = SEqHatDot_1 / norm;
    SEqHatDot_2 = SEqHatDot_2 / norm;
    SEqHatDot_3 = SEqHatDot_3 / norm;
    SEqHatDot_4 = SEqHatDot_4 / norm;
    // compute angular estimated direction of the gyroscope error
    w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
    w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
    w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
    // compute and remove the gyroscope baises
    w_bx += w_err_x * deltat * zeta;
    w_by += w_err_y * deltat * zeta;
    w_bz += w_err_z * deltat * zeta;
    gyro->w_x -= w_bx;
    gyro->w_y -= w_by;
    gyro->w_z -= w_bz;
    // compute the quaternion rate measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * gyro->w_x - halfSEq_3 * gyro->w_y - halfSEq_4 * gyro->w_z;
    SEqDot_omega_2 = halfSEq_1 * gyro->w_x + halfSEq_3 * gyro->w_z - halfSEq_4 * gyro->w_y;
    SEqDot_omega_3 = halfSEq_1 * gyro->w_y - halfSEq_2 * gyro->w_z + halfSEq_4 * gyro->w_x;
    SEqDot_omega_4 = halfSEq_1 * gyro->w_z + halfSEq_2 * gyro->w_y - halfSEq_3 * gyro->w_x;
    // compute then integrate the estimated quaternion rate
    SEQ.SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
    SEQ.SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
    SEQ.SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
    SEQ.SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
    // normalise quaternion
    norm = sqrt(SEQ.SEq_1 * SEQ.SEq_1 + SEQ.SEq_2 * SEQ.SEq_2 + SEQ.SEq_3 * SEQ.SEq_3 + SEQ.SEq_4 * SEQ.SEq_4);
    SEQ.SEq_1 /= norm;
    SEQ.SEq_2 /= norm;
    SEQ.SEq_3 /= norm;
    SEQ.SEq_4 /= norm;
    // compute flux in the earth frame
    SEq_1SEq_2 = SEQ.SEq_1 * SEQ.SEq_2; // recompute axulirary variables
    SEq_1SEq_3 = SEQ.SEq_1 * SEQ.SEq_3;
    SEq_1SEq_4 = SEQ.SEq_1 * SEQ.SEq_4;
    SEq_3SEq_4 = SEQ.SEq_3 * SEQ.SEq_4;
    SEq_2SEq_3 = SEQ.SEq_2 * SEQ.SEq_3;
    SEq_2SEq_4 = SEQ.SEq_2 * SEQ.SEq_4;
    h_x = twom_x * (0.5f - SEQ.SEq_3 * SEQ.SEq_3 - SEQ.SEq_4 * SEQ.SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
    h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - SEQ.SEq_2 * SEQ.SEq_2 - SEQ.SEq_4 * SEQ.SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
    h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - SEQ.SEq_2 * SEQ.SEq_2 - SEQ.SEq_3 * SEQ.SEq_3);
    // normalise the flux vector to have only components in the x and z
    b_x = sqrt((h_x * h_x) + (h_y * h_y));
    b_z = h_z;
}

void FilterUpdate9(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z) {
    // local system variables
    float norm; // vector norm
    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion rate from gyroscopes elements
    float f_1, f_2, f_3, f_4, f_5, f_6; // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, // objective function Jacobian elements
    J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
    float w_err_x, w_err_y, w_err_z; // estimated direction of the gyroscope error (angular)
    float h_x, h_y, h_z; // computed flux in the earth frame
    // axulirary variables to avoid reapeated calcualtions
    float halfSEq_1 = 0.5f * SEQ.SEq_1;
    float halfSEq_2 = 0.5f * SEQ.SEq_2;
    float halfSEq_3 = 0.5f * SEQ.SEq_3;
    float halfSEq_4 = 0.5f * SEQ.SEq_4;
    float twoSEq_1 = 2.0f * SEQ.SEq_1;
    float twoSEq_2 = 2.0f * SEQ.SEq_2;
    float twoSEq_3 = 2.0f * SEQ.SEq_3;
    float twoSEq_4 = 2.0f * SEQ.SEq_4;
    float twob_x = 2.0f * b_x;
    float twob_z = 2.0f * b_z;
    float twob_xSEq_1 = 2.0f * b_x * SEQ.SEq_1;
    float twob_xSEq_2 = 2.0f * b_x * SEQ.SEq_2;
    float twob_xSEq_3 = 2.0f * b_x * SEQ.SEq_3;
    float twob_xSEq_4 = 2.0f * b_x * SEQ.SEq_4;
    float twob_zSEq_1 = 2.0f * b_z * SEQ.SEq_1;
    float twob_zSEq_2 = 2.0f * b_z * SEQ.SEq_2;
    float twob_zSEq_3 = 2.0f * b_z * SEQ.SEq_3;
    float twob_zSEq_4 = 2.0f * b_z * SEQ.SEq_4;
    float SEq_1SEq_2;
    float SEq_1SEq_3 = SEQ.SEq_1 * SEQ.SEq_3;
    float SEq_1SEq_4;
    float SEq_2SEq_3;
    float SEq_2SEq_4 = SEQ.SEq_2 * SEQ.SEq_4;
    float SEq_3SEq_4;
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
    f_1 = twoSEq_2 * SEQ.SEq_4 - twoSEq_1 * SEQ.SEq_3 - a_x;
    f_2 = twoSEq_1 * SEQ.SEq_2 + twoSEq_3 * SEQ.SEq_4 - a_y;
    f_3 = 1.0f - twoSEq_2 * SEQ.SEq_2 - twoSEq_3 * SEQ.SEq_3 - a_z;
    f_4 = twob_x * (0.5f - SEQ.SEq_3 * SEQ.SEq_3 - SEQ.SEq_4 * SEQ.SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
    f_5 = twob_x * (SEQ.SEq_2 * SEQ.SEq_3 - SEQ.SEq_1 * SEQ.SEq_4) + twob_z * (SEQ.SEq_1 * SEQ.SEq_2 + SEQ.SEq_3 * SEQ.SEq_4) - m_y;
    f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - SEQ.SEq_2 * SEQ.SEq_2 - SEQ.SEq_3 * SEQ.SEq_3) - m_z;
    J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SEQ.SEq_4;
    J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2;
    J_32 = 2.0f * J_14or21; // negated in matrix multiplication
    J_33 = 2.0f * J_11or24; // negated in matrix multiplication
    J_41 = twob_zSEq_3; // negated in matrix multiplication
    J_42 = twob_zSEq_4;
    J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication
    J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
    J_51 = twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
    J_52 = twob_xSEq_3 + twob_zSEq_1;
    J_53 = twob_xSEq_2 + twob_zSEq_4;
    J_54 = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication
    J_61 = twob_xSEq_3;
    J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
    J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
    J_64 = twob_xSEq_2;
    // compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
    // normalise the gradient to estimate direction of the gyroscope error
    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 = SEqHatDot_1 / norm;
    SEqHatDot_2 = SEqHatDot_2 / norm;
    SEqHatDot_3 = SEqHatDot_3 / norm;
    SEqHatDot_4 = SEqHatDot_4 / norm;
    // compute angular estimated direction of the gyroscope error
    w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
    w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
    w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
    // compute and remove the gyroscope baises
    w_bx += w_err_x * deltat * zeta;
    w_by += w_err_y * deltat * zeta;
    w_bz += w_err_z * deltat * zeta;
    w_x -= w_bx;
    w_y -= w_by;
    w_z -= w_bz;
    // compute the quaternion rate measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
    // compute then integrate the estimated quaternion rate
    SEQ.SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
    SEQ.SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
    SEQ.SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
    SEQ.SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
    // normalise quaternion
    norm = sqrt(SEQ.SEq_1 * SEQ.SEq_1 + SEQ.SEq_2 * SEQ.SEq_2 + SEQ.SEq_3 * SEQ.SEq_3 + SEQ.SEq_4 * SEQ.SEq_4);
    SEQ.SEq_1 /= norm;
    SEQ.SEq_2 /= norm;
    SEQ.SEq_3 /= norm;
    SEQ.SEq_4 /= norm;
    // compute flux in the earth frame
    SEq_1SEq_2 = SEQ.SEq_1 * SEQ.SEq_2; // recompute axulirary variables
    SEq_1SEq_3 = SEQ.SEq_1 * SEQ.SEq_3;
    SEq_1SEq_4 = SEQ.SEq_1 * SEQ.SEq_4;
    SEq_3SEq_4 = SEQ.SEq_3 * SEQ.SEq_4;
    SEq_2SEq_3 = SEQ.SEq_2 * SEQ.SEq_3;
    SEq_2SEq_4 = SEQ.SEq_2 * SEQ.SEq_4;
    h_x = twom_x * (0.5f - SEQ.SEq_3 * SEQ.SEq_3 - SEQ.SEq_4 * SEQ.SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
    h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - SEQ.SEq_2 * SEQ.SEq_2 - SEQ.SEq_4 * SEQ.SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
    h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - SEQ.SEq_2 * SEQ.SEq_2 - SEQ.SEq_3 * SEQ.SEq_3);
    // normalise the flux vector to have only components in the x and z
    b_x = sqrt((h_x * h_x) + (h_y * h_y));
    b_z = h_z;
}

Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

EulerAngles ToEulerAngles(Quaternion q, uint8_t deg) {
    EulerAngles angles;
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
