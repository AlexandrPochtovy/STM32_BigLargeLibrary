/*********************************************************************************
 Original author: Aliaksandr Pachtovy<alex.mail.prime@gmail.com>

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.

 Created on: Sep 17, 2020
*********************************************************************************/
#ifndef FUNCTION_H_
#define FUNCTION_H_

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>


size_t Min(size_t val, size_t min);
size_t Max(size_t val, size_t max);

size_t CONCAT_TWO_BYTES(uint8_t msb, uint8_t lsb);
size_t CONCAT_FOUR_BYTES(uint8_t hh_b, uint8_t h_b, uint8_t l_b, uint8_t ll_b);

static inline void BusRequestOn(uint32_t req, uint32_t mask) {
	req |= mask;
}
static inline void BusRequestOff(uint32_t req, uint32_t mask) {
	req &= ~mask;
}

float invSqrt(float x);

int16_t signum_t(int16_t x);
int16_t signum_f(float x);

float sigmoida(float x, float a);
float sigmoida_zero(float x, float a);

uint16_t alphabeta(uint16_t act, uint16_t last, uint8_t deep);

size_t Ramp(size_t sp, size_t act, size_t acc, size_t dec, size_t LL, size_t HH);

#endif /* FUNCTION_H_ */
