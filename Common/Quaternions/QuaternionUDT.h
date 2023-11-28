/*********************************************************************************
	Original author:  Aliaksandr Pachtovy<alex.mail.prime@gmail.com>
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

 * 	QuaternionUDT.h
 * 	Created on: 24.11.2023
 */

#ifndef _DATATYPES_H_
#define _DATATYPES_H_

#include <stdint.h>
#include <stddef.h>
#include <math.h>

 /**********************************************************************
 *                       TYPEDEF's                                     *
 ***********************************************************************/
typedef struct vector {
	float x;// X- axis measurements
	float y;// Y- axis measurements
	float z;// Z- axis measurements
	} vector_t;

//Quaternion
typedef struct Quaternion {
	float w;
	float x;
	float y;
	float z;
	} Quaternion_t;

typedef struct EulerAngles {
	float roll;
	float pitch;
	float yaw;
	} EulerAngles_t;


#endif
