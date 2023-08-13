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

#include <stdint.h>

typedef struct SG90_dev_t {
	const uint16_t max;
	const uint16_t min;
	int16_t step;
	const uint16_t N;
} SG90_dev;

uint16_t SG90_Init(SG90_dev *sg);
uint16_t SG90Write(SG90_dev *sg, uint16_t SP,  uint16_t act);

#endif /* SG90_MASTER_SG90_H_ */
