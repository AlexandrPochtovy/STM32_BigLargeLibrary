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

#include "Function.h"


//-------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	uint32_t i = *(uint32_t*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

int16_t signum_t(int16_t x) {
	if (x > 0) {
		return 1;
	} else if (x < 0) {
		return -1;
	} else return 0;
}

int16_t signum_f(float x) {
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
	return (1 / (1 + expf(-a * x))) - 0.5;
}

uint16_t alphabeta(uint16_t new, uint16_t last, uint8_t deep) {
	uint32_t tmp = last * (deep - 1) + new;
	return tmp / deep;
}

size_t Ramp(size_t sp, size_t act, size_t acc, size_t dec, size_t LL, size_t HH) {
	size_t tmp;
	if (act < sp) {
		tmp = sp + acc;
		if (tmp > HH) { return HH; }
		else { return tmp;}
	} else if (act > sp) {
		tmp = sp - dec;
		if (tmp < LL) { return LL; }
		else { return tmp; }
	}
	else {return sp;}
}
