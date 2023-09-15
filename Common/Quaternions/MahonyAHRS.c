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
   
 	MahonyAHRS.c
	Created on: 18.04.2021
*********************************************************************************/
#include "MahonyAHRS.h"

// 2 * proportional gain (Kp)
// 2 * integral gain (Ki)
static Quaternion_t Q = {1.0, 0.0, 0.0, 0.0}; 	// static internal quaternion of sensor frame relative to auxiliary frame
static Axis_t integralFB = {0.0f,  0.0f, 0.0f};	// integral error terms scaled by Ki

Quaternion_t MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, uint32_t deltat) {
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		return MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az, twoKp, twoKi, deltat);
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
			integralFB.x += twoKi * halfex * deltat / 1000.0f;	// integral error scaled by Ki
			integralFB.y += twoKi * halfey * deltat / 1000.0f;
			integralFB.z += twoKi * halfez * deltat / 1000.0f;
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
	gx *= (0.5f * deltat / 1000.0f);		// pre-multiply common factors
	gy *= (0.5f * deltat / 1000.0f);
	gz *= (0.5f * deltat / 1000.0f);
	qa = Q.w;
	qb = Q.x;
	qc = Q.y;
	Q.w += (-qb * gx - qc * gy - Q.z * gz);
	Q.x += (qa * gx + qc * gz - Q.z * gy);
	Q.y += (qa * gy - qb * gz + Q.z * gx);
	Q.z += (qa * gz + qb * gy - qc * gx); 
	// Normalise quaternion
	recipNorm = invSqrt(Q.w * Q.w + Q.x * Q.x + Q.y * Q.y + Q.z * Q.z);
	Q.w *= recipNorm;
	Q.x *= recipNorm;
	Q.y *= recipNorm;
	Q.z *= recipNorm;
	return Q;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

Quaternion_t MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float twoKp, float twoKi, uint32_t deltat) {
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
		halfvx = Q.x * Q.z - Q.w * Q.y;
		halfvy = Q.w * Q.x + Q.y * Q.z;
		halfvz = Q.w * Q.w - 0.5f + Q.z * Q.z;
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFB.x += twoKi * halfex * deltat / 1000.0f;	// integral error scaled by Ki
			integralFB.y += twoKi * halfey * deltat / 1000.0f;
			integralFB.z += twoKi * halfez * deltat / 1000.0f;
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
	gx *= (0.5f * deltat / 1000.0f);		// pre-multiply common factors
	gy *= (0.5f * deltat / 1000.0f);
	gz *= (0.5f * deltat / 1000.0f);
	qa = Q.w;
	qb = Q.x;
	qc = Q.y;
	Q.w += (-qb * gx - qc * gy - Q.z * gz);
	Q.x += (qa * gx + qc * gz - Q.z * gy);
	Q.y += (qa * gy - qb * gz + Q.z * gx);
	Q.z += (qa * gz + qb * gy - qc * gx); 
	// Normalise quaternion
	recipNorm = invSqrt(Q.w * Q.w + Q.x * Q.x + Q.y * Q.y + Q.z * Q.z);
	Q.w *= recipNorm;
	Q.x *= recipNorm;
	Q.y *= recipNorm;
	Q.z *= recipNorm;
	return Q;
}
