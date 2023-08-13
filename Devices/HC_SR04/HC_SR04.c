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

 * 	HC_SR04.c
 *  Created on: 6 dec 2020
 */
#include "HC_SR04.h"

//calculate distance from HC-SR04
void HC_SR04DistanceSimpleCalc(HC_SR04_t *data, uint16_t soundSpeed, uint16_t countLimit) {
	 uint32_t delta;
	if (data->stop >= data->start) {
		delta = data->stop - data->start;
	} else {
		delta = countLimit - data->start + data->stop;
	}
	data->distance_mm =  (uint16_t)(((delta / 2) * soundSpeed) / 1000);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * **
 * temp		| -20.0 | -10.0 |  0.0  |  10.0 | 20.0  | 30.0  |
 * speed	| 318.8 | 325.1 | 331.5 | 337.3 | 343.1 | 348.9 |
 *
 * speed = temp * 301/500 + 8271 / 25
 * speed = 0.6 * temp + 331.1
 *  * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void HC_SR04DistanceAdvancedCalc(HC_SR04_t *data, uint16_t countLimit, float temp){
	uint32_t delta = 0;
	float range = 0.0;
	float soundSpeed = 0.6 * temp + 331.1;
	if (data->stop >= data->start) {
		delta = data->stop - data->start;
	} else {
		delta = countLimit - data->start + data->stop;
	}
	range = delta * soundSpeed / 2000;
	data->distance_mm = (uint16_t)invSqrt(range * range - 2.25);
}

