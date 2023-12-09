/*
 * Converters.c
 *
 *  Created on: Nov 26, 2023
 *      Author: alexm
 */

#include "Converters.h"

Quaternion_t ToQuaternion(double yaw, double pitch, double roll) {
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

EulerAngles_t ToEulerAngles(Quaternion_t q, uint8_t deg) {
    EulerAngles_t angles;
    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = sqrtf(1 + 2 * (q.w * q.y - q.x * q.z));
    float cosp = sqrtf(1 - 2 * (q.w * q.y - q.x * q.z));
    angles.pitch = 2 * atan2f(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = atan2f(siny_cosp, cosy_cosp);
	if (deg)
	{
		angles.pitch = RadiansToDegrees(angles.pitch);
		angles.roll = RadiansToDegrees(angles.roll);
		angles.yaw = RadiansToDegrees(angles.yaw);
		//if (angles.pitch < 0) angles.pitch = 360.0f + angles.pitch;
		//if (angles.roll < 0) angles.roll = 360.0f + angles.roll;
		//if (angles.yaw < 0) angles.yaw = 360.0f + angles.yaw;
	}
    return angles;
}
