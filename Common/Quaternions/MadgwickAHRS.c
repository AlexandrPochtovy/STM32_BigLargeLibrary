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
   
 	MadgwickAHRS.c
	Created on: 30.05.2021

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

#include "MadgwickAHRS.h"

#define betaDef		0.1f					// 2 * proportional gain
Quaternion_t quatM = {1.0, 0.0, 0.0, 0.0};	// quaternion of sensor frame relative to auxiliary frame

// AHRS algorithm update
Quaternion_t MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		return MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
	}
	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-quatM.x * gx - quatM.y * gy - quatM.z * gz);
	qDot2 = 0.5f * (quatM.w * gx + quatM.y * gz - quatM.z * gy);
	qDot3 = 0.5f * (quatM.w * gy - quatM.x * gz + quatM.z * gx);
	qDot4 = 0.5f * (quatM.w * gz + quatM.x * gy - quatM.y * gx);
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
		_2q0mx = 2.0f * quatM.w * mx;
		_2q0my = 2.0f * quatM.w * my;
		_2q0mz = 2.0f * quatM.w * mz;
		_2q1mx = 2.0f * quatM.x * mx;
		_2q0 = 2.0f * quatM.w;
		_2q1 = 2.0f * quatM.x;
		_2q2 = 2.0f * quatM.y;
		_2q3 = 2.0f * quatM.z;
		_2q0q2 = 2.0f * quatM.w * quatM.y;
		_2q2q3 = 2.0f * quatM.y * quatM.z;
		q0q0 = quatM.w * quatM.w;
		q0q1 = quatM.w * quatM.x;
		q0q2 = quatM.w * quatM.y;
		q0q3 = quatM.w * quatM.z;
		q1q1 = quatM.x * quatM.x;
		q1q2 = quatM.x * quatM.y;
		q1q3 = quatM.x * quatM.z;
		q2q2 = quatM.y * quatM.y;
		q2q3 = quatM.y * quatM.z;
		q3q3 = quatM.z * quatM.z;
		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * quatM.z + _2q0mz * quatM.y + mx * q1q1 + _2q1 * my * quatM.y + _2q1 * mz * quatM.z - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * quatM.z + my * q0q0 - _2q0mz * quatM.x + _2q1mx * quatM.y - my * q1q1 + my * q2q2 + _2q2 * mz * quatM.z - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * quatM.y + _2q0my * quatM.x + mz * q0q0 + _2q1mx * quatM.z - mz * q1q1 + _2q2 * my * quatM.z - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;
		// Gradient decent algorithm corrective step
		//old
		//s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * quatM.y * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * quatM.z + _2bz * quatM.x) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * quatM.y * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		//s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * quatM.x * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * quatM.z * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * quatM.y + _2bz * quatM.w) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * quatM.z - _4bz * quatM.x) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		//s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * quatM.y * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * quatM.y - _2bz * quatM.w) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * quatM.x + _2bz * quatM.z) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * quatM.w - _4bz * quatM.y) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		//s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * quatM.z + _2bz * quatM.x) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * quatM.w + _2bz * quatM.y) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * quatM.x * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		//new fix
		s0 = -_2q2 * (2 * (q1q3 - q0q2) - ax) + _2q1 * (2 * (q0q1 + q2q3) - ay) + -_4bz * quat.y * (_4bx * (0.5 - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (-_4bx * quat.z + _4bz * quat.x) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + _4bx * quat.y * (_4bx * (q0q2 + q1q3) + _4bz * (0.5 - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2 * (q1q3 - q0q2) - ax) + _2q0 * (2 * (q0q1 + q2q3) - ay) + -4 * quat.x * (2 * (0.5 - q1q1 - q2q2) - az) + _4bz * quat.z * (_4bx * (0.5 - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (_4bx * quat.y + _4bz * quat.w) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + (_4bx * quat.z - _8bz * quat.x) * (_4bx * (q0q2 + q1q3) + _4bz * (0.5 - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2 * (q1q3 - q0q2) - ax) + _2q3 * (2 * (q0q1 + q2q3) - ay) + (-4 * quat.y) * (2 * (0.5 - q1q1 - q2q2) - az) + (-_8bx * quat.y - _4bz * quat.w) * (_4bx * (0.5 - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (_4bx * quat.x + _4bz * quat.z) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + (_4bx * quat.w - _8bz * quat.y) * (_4bx * (q0q2 + q1q3) + _4bz * (0.5 - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2 * (q1q3 - q0q2) - ax) + _2q2 * (2 * (q0q1 + q2q3) - ay)+(-_8bx * q3 + _4bz * q1) * (_4bx * (0.5 - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (-_4bx * q0 + _4bz * q2) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + (_4bx * q1) * (_4bx * (q0q2 + q1q3) + _4bz * (0.5 - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;
		// Apply feedback step
		qDot1 -= quatM.beta * s0;
		qDot2 -= quatM.beta * s1;
		qDot3 -= quatM.beta * s2;
		qDot4 -= quatM.beta * s3;
	}
	// Integrate rate of change of quaternion to yield quaternion
	quatM.w += qDot1 * (1.0f / quatM.freq);
	quatM.x += qDot2 * (1.0f / quatM.freq);
	quatM.y += qDot3 * (1.0f / quatM.freq);
	quatM.z += qDot4 * (1.0f / quatM.freq);
	// Normalise quaternion
	recipNorm = invSqrt(quatM.w * quatM.w + quatM.x * quatM.x + quatM.y * quatM.y + quatM.z * quatM.z);
	quatM.w *= recipNorm;
	quatM.x *= recipNorm;
	quatM.y *= recipNorm;
	quatM.z *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update
Quaternion_t MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-quatM.x * gx - quatM.y * gy - quatM.z * gz);
	qDot2 = 0.5f * (quatM.w * gx + quatM.y * gz - quatM.z * gy);
	qDot3 = 0.5f * (quatM.w * gy - quatM.x * gz + quatM.z * gx);
	qDot4 = 0.5f * (quatM.w * gz + quatM.x * gy - quatM.y * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * quatM.w;
		_2q1 = 2.0f * quatM.x;
		_2q2 = 2.0f * quatM.y;
		_2q3 = 2.0f * quatM.z;
		_4q0 = 4.0f * quatM.w;
		_4q1 = 4.0f * quatM.x;
		_4q2 = 4.0f * quatM.y;
		_8q1 = 8.0f * quatM.x;
		_8q2 = 8.0f * quatM.y;
		q0q0 = quatM.w * quatM.w;
		q1q1 = quatM.x * quatM.x;
		q2q2 = quatM.y * quatM.y;
		q3q3 = quatM.z * quatM.z;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * quatM.x - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * quatM.y + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * quatM.z - _2q1 * ax + 4.0f * q2q2 * quatM.z - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= quatM.beta * s0;
		qDot2 -= quatM.beta * s1;
		qDot3 -= quatM.beta * s2;
		qDot4 -= quatM.beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	quatM.w += qDot1 * (1.0f / quatM.freq);
	quatM.x += qDot2 * (1.0f / quatM.freq);
	quatM.y += qDot3 * (1.0f / quatM.freq);
	quatM.z += qDot4 * (1.0f / quatM.freq);

	// Normalise quaternion
	recipNorm = invSqrt(quatM.w * quatM.w + quatM.x * quatM.x + quatM.y * quatM.y + quatM.z * quatM.z);
	quatM.w *= recipNorm;
	quatM.x *= recipNorm;
	quatM.y *= recipNorm;
	quatM.z *= recipNorm;
}
