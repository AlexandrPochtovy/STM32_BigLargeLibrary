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

  QuaterFilter.h
	Created on: 11.02.2021
*********************************************************************************/

#ifndef _QUATERFILTER_H_
#define _QUATERFILTER_H_

#ifdef __cplusplus
extern "C" {
#endif


#include <stddef.h>
#include <stdint.h>
#include <math.h>   // Math library required for ‘sqrt’
#include <string.h>
#include "Quaternion.h"
#include "Function/Function.h"

Quaternion_t FilterUpdate_6DOF(Axis_t accel, Axis_t gyro, uint32_t deltat);

Quaternion_t FilterUpdate_6(float a_x, float a_y, float a_z, float g_x, float g_y, float g_z, uint32_t deltat);

Quaternion_t FilterUpdate_9DOF(Axis_t accel, Axis_t gyro, Axis_t mag, uint32_t deltat);

Quaternion_t FilterUpdate9(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z, uint32_t deltat);

Quaternion_t EulerAngleToQuaternion(double yaw, double pitch, double roll);

EulerAngles_t QuaternionToEulerAngles(Quaternion_t q, uint8_t deg);
#ifdef __cplusplus
}
#endif

#endif // _QUATERFILTER_H_
