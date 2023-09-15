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
   
 	MadgwickAHRS.h
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
#ifndef _MADGWICK_AHRS_h
#define _MADGWICK_AHRS_h

#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include "Quaternion.h"
#include "Function/Function.h"

Quaternion_t MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
Quaternion_t MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

#endif /* _MADGWICK_AHRS_h */
