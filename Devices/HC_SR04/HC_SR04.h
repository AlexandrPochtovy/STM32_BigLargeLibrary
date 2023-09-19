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

 * 	HC_SR04.h
 *  Created on: 6 dec 2020
 */

#ifndef _HC_SR04_H_
#define _HC_SR04_H_

#ifdef __cplusplus
extern "C" {
#endif
/* include -------------------------------------------------------------------*/
#include <stdint.h>
#include "Function/Function.h"
/* typedef -------------------------------------------------------------------*/
typedef struct HC_SR04 {
	uint16_t start;
	uint16_t stop;
	uint16_t distance_mm;
} HC_SR04_t;

//calculate distance to object from HC-SR04
void HC_SR04DistanceSimpleCalc(HC_SR04_t *data, uint16_t soundSpeed, uint16_t countLimit);
void HC_SR04DistanceAdvancedCalc(HC_SR04_t *data, uint16_t countLimit, float temp);

#ifdef __cplusplus
}
#endif

#endif /* _HC_SR04_H_ */
