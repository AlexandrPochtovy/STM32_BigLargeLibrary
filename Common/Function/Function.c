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

 Created on: Sep 17, 2020
*********************************************************************************/

#include "Function.h"

static inline size_t Min(size_t val, size_t min) {
	return (val > min ? val : min);
}

static inline size_t Max(size_t val, size_t max) {
	return (val < max ? val : max);
}

size_t CONCAT_TWO_BYTES(uint8_t msb, uint8_t lsb) {
	return (((size_t)msb << 8) | (size_t)lsb);
}

size_t CONCAT_FOUR_BYTES(uint8_t msbh, uint8_t msb, uint8_t lsbh, uint8_t lsb) {
	return ((size_t)msbh << 24) | ((size_t)msb << 16) | ((size_t)lsbh << 8) | (size_t)lsb;
}

float invSqrt(float x) {
 union {
  float    f;
  uint32_t i;
 } conv = { .f = x };
 conv.i  = 0x5f3759df - (conv.i >> 1);
 conv.f *= 1.5F - (x * 0.5F * conv.f * conv.f);
 return conv.f;
}

int16_t signum_t(size_t x) {
	if (x > 0) {
		return 1;
	} else if (x < 0) {
		return -1;
	} else return 0;
}

float sigmoida(float x, float a) {
	return 1 / (1 + expf(-a * x));
}

float sigmoida_zero(float x, float a) {
	return (2 / (1 + expf(-a * x))) - 1;
}

uint16_t alphabeta(uint16_t new, uint16_t last, uint8_t deep) {
	uint32_t tmp = last * (deep - 1) + new;
	return tmp / deep;
}

