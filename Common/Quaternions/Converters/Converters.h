/*
 * Converters.h
 *
 *  Created on: Nov 26, 2023
 *      Author: alexm
 */

#ifndef QUATERNIONS_CONVERTERS_CONVERTERS_H_
#define QUATERNIONS_CONVERTERS_CONVERTERS_H_

#include "Quaternions/QuaternionUDT.h"


Quaternion_t ToQuaternion(double yaw, double pitch, double roll);
EulerAngles_t ToEulerAngles(Quaternion_t q, uint8_t deg);

#endif /* QUATERNIONS_CONVERTERS_CONVERTERS_H_ */
