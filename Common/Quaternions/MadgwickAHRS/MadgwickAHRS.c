//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include <math.h>
#include "MadgwickAHRS.h"


//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	20.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;								// 2 * proportional gain (Kp)
static Quaternion_t base = { .w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f };// quaternion of sensor frame relative to auxiliary frame
//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

Quaternion_t MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		return MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		}
	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-base.x * gx - base.y * gy - base.z * gz);
	qDot2 = 0.5f * (base.w * gx + base.y * gz - base.z * gy);
	qDot3 = 0.5f * (base.w * gy - base.x * gz + base.z * gx);
	qDot4 = 0.5f * (base.w * gz + base.x * gy - base.y * gx);
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
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
		_2q0mx = 2.0f * base.w * mx;
		_2q0my = 2.0f * base.w * my;
		_2q0mz = 2.0f * base.w * mz;
		_2q1mx = 2.0f * base.x * mx;
		_2q0 = 2.0f * base.w;
		_2q1 = 2.0f * base.x;
		_2q2 = 2.0f * base.y;
		_2q3 = 2.0f * base.z;
		_2q0q2 = 2.0f * base.w * base.y;
		_2q2q3 = 2.0f * base.y * base.z;
		q0q0 = base.w * base.w;
		q0q1 = base.w * base.x;
		q0q2 = base.w * base.y;
		q0q3 = base.w * base.z;
		q1q1 = base.x * base.x;
		q1q2 = base.x * base.y;
		q1q3 = base.x * base.z;
		q2q2 = base.y * base.y;
		q2q3 = base.y * base.z;
		q3q3 = base.z * base.z;
		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * base.z + _2q0mz * base.y + mx * q1q1 + _2q1 * my * base.y + _2q1 * mz * base.z - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * base.z + my * q0q0 - _2q0mz * base.x + _2q1mx * base.y - my * q1q1 + my * q2q2 + _2q2 * mz * base.z - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * base.y + _2q0my * base.x + mz * q0q0 + _2q1mx * base.z - mz * q1q1 + _2q2 * my * base.z - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;
		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * base.y * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * base.z + _2bz * base.x) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * base.y * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * base.x * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * base.z * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * base.y + _2bz * base.w) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * base.z - _4bz * base.x) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * base.y * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * base.y - _2bz * base.w) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * base.x + _2bz * base.z) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * base.w - _4bz * base.y) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * base.z + _2bz * base.x) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * base.w + _2bz * base.y) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * base.x * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;
		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
		}
	// Integrate rate of change of quaternion to yield quaternion
	base.w += qDot1 * (1.0f / sampleFreq);
	base.x += qDot2 * (1.0f / sampleFreq);
	base.y += qDot3 * (1.0f / sampleFreq);
	base.z += qDot4 * (1.0f / sampleFreq);
	// Normalise quaternion
	recipNorm = invSqrt(base.w * base.w + base.x * base.x + base.y * base.y + base.z * base.z);
	base.w *= recipNorm;
	base.x *= recipNorm;
	base.y *= recipNorm;
	base.z *= recipNorm;
	return base;
	}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

Quaternion_t MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-base.x * gx - base.y * gy - base.z * gz);
	qDot2 = 0.5f * (base.w * gx + base.y * gz - base.z * gy);
	qDot3 = 0.5f * (base.w * gy - base.x * gz + base.z * gx);
	qDot4 = 0.5f * (base.w * gz + base.x * gy - base.y * gx);
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;
		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * base.x;
		_2q1 = 2.0f * base.x;
		_2q2 = 2.0f * base.y;
		_2q3 = 2.0f * base.z;
		_4q0 = 4.0f * base.w;
		_4q1 = 4.0f * base.x;
		_4q2 = 4.0f * base.y;
		_8q1 = 8.0f * base.z;
		_8q2 = 8.0f * base.y;
		q0q0 = base.w * base.w;
		q1q1 = base.x * base.x;
		q2q2 = base.y * base.y;
		q3q3 = base.z * base.z;
		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * base.x - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * base.y + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * base.z - _2q1 * ax + 4.0f * q2q2 * base.z - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;
		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
		}
	// Integrate rate of change of quaternion to yield quaternion
	base.w += qDot1 * (1.0f / sampleFreq);
	base.x += qDot2 * (1.0f / sampleFreq);
	base.y += qDot3 * (1.0f / sampleFreq);
	base.z += qDot4 * (1.0f / sampleFreq);
	// Normalise quaternion
	recipNorm = invSqrt(base.w * base.w + base.x * base.x + base.y * base.y + base.z * base.z);
	base.w *= recipNorm;
	base.x *= recipNorm;
	base.y *= recipNorm;
	base.z *= recipNorm;
	return base;
	}

//====================================================================================================
// END OF CODE
//====================================================================================================
