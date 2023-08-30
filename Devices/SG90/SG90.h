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

 * 	SG90.h
 *  Created on: Nov 6, 2021
 */

#ifndef SG90_MASTER_SG90_H_
#define SG90_MASTER_SG90_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct Servo {
	uint16_t max;
	uint16_t min;
	uint16_t step;
} Servo_t;

uint16_t Servo_Init(Servo_t sg, uint16_t lowLimit, uint16_t hiLimit, uint16_t step);
uint16_t ServoSetPWM_IT(Servo_t sg, uint16_t actual, uint16_t SP);

#ifdef __cplusplus
}
#endif
#endif /* SG90_MASTER_SG90_H_ */
