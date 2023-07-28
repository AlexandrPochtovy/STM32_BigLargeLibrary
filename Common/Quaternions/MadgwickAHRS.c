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

//---------------------------------------------------------------------------------------------------
// Variable declaration
//main quaternion 				q0,  q1,  q2,  q3,  beta,    samplefreq
//extern volatile float beta = betaDef;								// algorithm gain = 2 * proportional gain (Kp)
//extern volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
QuatMadgwick quatM = {1.0, 0.0, 0.0, 0.0, betaDef, sampleFreq};
//---------------------------------------------------------------------------------------------------
// Function declarations

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-quatM.q1 * gx - quatM.q2 * gy - quatM.q3 * gz);
	qDot2 = 0.5f * (quatM.q0 * gx + quatM.q2 * gz - quatM.q3 * gy);
	qDot3 = 0.5f * (quatM.q0 * gy - quatM.q1 * gz + quatM.q3 * gx);
	qDot4 = 0.5f * (quatM.q0 * gz + quatM.q1 * gy - quatM.q2 * gx);

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
		_2q0mx = 2.0f * quatM.q0 * mx;
		_2q0my = 2.0f * quatM.q0 * my;
		_2q0mz = 2.0f * quatM.q0 * mz;
		_2q1mx = 2.0f * quatM.q1 * mx;
		_2q0 = 2.0f * quatM.q0;
		_2q1 = 2.0f * quatM.q1;
		_2q2 = 2.0f * quatM.q2;
		_2q3 = 2.0f * quatM.q3;
		_2q0q2 = 2.0f * quatM.q0 * quatM.q2;
		_2q2q3 = 2.0f * quatM.q2 * quatM.q3;
		q0q0 = quatM.q0 * quatM.q0;
		q0q1 = quatM.q0 * quatM.q1;
		q0q2 = quatM.q0 * quatM.q2;
		q0q3 = quatM.q0 * quatM.q3;
		q1q1 = quatM.q1 * quatM.q1;
		q1q2 = quatM.q1 * quatM.q2;
		q1q3 = quatM.q1 * quatM.q3;
		q2q2 = quatM.q2 * quatM.q2;
		q2q3 = quatM.q2 * quatM.q3;
		q3q3 = quatM.q3 * quatM.q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * quatM.q3 + _2q0mz * quatM.q2 + mx * q1q1 + _2q1 * my * quatM.q2 + _2q1 * mz * quatM.q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * quatM.q3 + my * q0q0 - _2q0mz * quatM.q1 + _2q1mx * quatM.q2 - my * q1q1 + my * q2q2 + _2q2 * mz * quatM.q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * quatM.q2 + _2q0my * quatM.q1 + mz * q0q0 + _2q1mx * quatM.q3 - mz * q1q1 + _2q2 * my * quatM.q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		//old
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * quatM.q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * quatM.q3 + _2bz * quatM.q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * quatM.q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * quatM.q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * quatM.q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * quatM.q2 + _2bz * quatM.q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * quatM.q3 - _4bz * quatM.q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * quatM.q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * quatM.q2 - _2bz * quatM.q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * quatM.q1 + _2bz * quatM.q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * quatM.q0 - _4bz * quatM.q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * quatM.q3 + _2bz * quatM.q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * quatM.q0 + _2bz * quatM.q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * quatM.q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		
		//new fix
/*	s0 = -_2q2 * (2 * (q1q3 - q0q2) - ax) + _2q1 * (2 * (q0q1 + q2q3) - ay) + -_4bz * quat.q2 * (_4bx * (0.5 - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (-_4bx * quat.q3 + _4bz * quat.q1) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + _4bx * quat.q2 * (_4bx * (q0q2 + q1q3) + _4bz * (0.5 - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2 * (q1q3 - q0q2) - ax) + _2q0 * (2 * (q0q1 + q2q3) - ay) + -4 * quat.q1 * (2 * (0.5 - q1q1 - q2q2) - az) + _4bz * quat.q3 * (_4bx * (0.5 - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (_4bx * quat.q2 + _4bz * quat.q0) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + (_4bx * quat.q3 - _8bz * quat.q1) * (_4bx * (q0q2 + q1q3) + _4bz * (0.5 - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2 * (q1q3 - q0q2) - ax) + _2q3 * (2 * (q0q1 + q2q3) - ay) + (-4 * quat.q2) * (2 * (0.5 - q1q1 - q2q2) - az) + (-_8bx * quat.q2 - _4bz * quat.q0) * (_4bx * (0.5 - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (_4bx * quat.q1 + _4bz * quat.q3) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + (_4bx * quat.q0 - _8bz * quat.q2) * (_4bx * (q0q2 + q1q3) + _4bz * (0.5 - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2 * (q1q3 - q0q2) - ax) + _2q2 * (2 * (q0q1 + q2q3) - ay)+(-_8bx * q3 + _4bz * q1) * (_4bx * (0.5 - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (-_4bx * q0 + _4bz * q2) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + (_4bx * q1) * (_4bx * (q0q2 + q1q3) + _4bz * (0.5 - q1q1 - q2q2) - mz);
*/

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
	quatM.q0 += qDot1 * (1.0f / quatM.freq);
	quatM.q1 += qDot2 * (1.0f / quatM.freq);
	quatM.q2 += qDot3 * (1.0f / quatM.freq);
	quatM.q3 += qDot4 * (1.0f / quatM.freq);

	// Normalise quaternion
	recipNorm = invSqrt(quatM.q0 * quatM.q0 + quatM.q1 * quatM.q1 + quatM.q2 * quatM.q2 + quatM.q3 * quatM.q3);
	quatM.q0 *= recipNorm;
	quatM.q1 *= recipNorm;
	quatM.q2 *= recipNorm;
	quatM.q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-quatM.q1 * gx - quatM.q2 * gy - quatM.q3 * gz);
	qDot2 = 0.5f * (quatM.q0 * gx + quatM.q2 * gz - quatM.q3 * gy);
	qDot3 = 0.5f * (quatM.q0 * gy - quatM.q1 * gz + quatM.q3 * gx);
	qDot4 = 0.5f * (quatM.q0 * gz + quatM.q1 * gy - quatM.q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * quatM.q0;
		_2q1 = 2.0f * quatM.q1;
		_2q2 = 2.0f * quatM.q2;
		_2q3 = 2.0f * quatM.q3;
		_4q0 = 4.0f * quatM.q0;
		_4q1 = 4.0f * quatM.q1;
		_4q2 = 4.0f * quatM.q2;
		_8q1 = 8.0f * quatM.q1;
		_8q2 = 8.0f * quatM.q2;
		q0q0 = quatM.q0 * quatM.q0;
		q1q1 = quatM.q1 * quatM.q1;
		q2q2 = quatM.q2 * quatM.q2;
		q3q3 = quatM.q3 * quatM.q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * quatM.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * quatM.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * quatM.q3 - _2q1 * ax + 4.0f * q2q2 * quatM.q3 - _2q2 * ay;
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
	quatM.q0 += qDot1 * (1.0f / quatM.freq);
	quatM.q1 += qDot2 * (1.0f / quatM.freq);
	quatM.q2 += qDot3 * (1.0f / quatM.freq);
	quatM.q3 += qDot4 * (1.0f / quatM.freq);

	// Normalise quaternion
	recipNorm = invSqrt(quatM.q0 * quatM.q0 + quatM.q1 * quatM.q1 + quatM.q2 * quatM.q2 + quatM.q3 * quatM.q3);
	quatM.q0 *= recipNorm;
	quatM.q1 *= recipNorm;
	quatM.q2 *= recipNorm;
	quatM.q3 *= recipNorm;
}
