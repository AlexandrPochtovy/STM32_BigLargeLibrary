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
#ifndef _FUNCTION_H_
#define _FUNCTION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/*****************************************************************
  * @brief  selects the minimum value from two
  * @param  value, min
  * @retval minimal
  */
size_t Min(size_t val, size_t min);

/*****************************************************************
  * @brief selects the maximum value from two
  * @param value, max
  * @retval max
  */
size_t Max(size_t val, size_t max);

/*****************************************************************
  * @brief  concat two bytes into 16bits value
  * @param  msb - hi byte
  * 				lsb - low byte
  * @retval uint16_t or int16_t value
  */
static inline uint16_t CONCAT_TWO_BYTES(uint8_t msb, uint8_t lsb) {
	return (((uint16_t)msb << 8) | (uint16_t)lsb);
}

static inline int16_t ConvertTwoCompl(uint16_t val) {
	return 0x8000 & val ? (~(0x7FFF & val) + 1) : val;
}

/*****************************************************************
  * @brief  concat four bytes into 32bits value
  * @param  msbh - most hi byte
  *         msb - hi byte
  * 				lsbh - low byte
  * 				lsb - most low byte
  * @retval uint32_t or int32_t value
  */
size_t CONCAT_FOUR_BYTES(uint8_t msbh, uint8_t msb, uint8_t lsbh, uint8_t lsb);

/*****************************************************************
  * @brief  simple set bit in value on mask
  * @param  req - value, mask - mask
  * @retval inline
  */
static inline void BusRequestOn(uint32_t req, uint32_t mask) {
	req |= mask;
}

/*****************************************************************
  * @brief  simple reset bit in value on mask
  * @param  req - value, mask - mask
  * @retval inline
  */
static inline void BusRequestOff(uint32_t req, uint32_t mask) {
	req &= ~mask;
}

/*****************************************************************
  * @brief  Fast inverse square root https://en.wikipedia.org/wiki/Fast_inverse_square_root
  * @param  x - value
  * @retval return square root from x
  */
float invSqrt(float x);

/*****************************************************************
  * @brief  define value's signum
  * @param  x - value
  * @retval 1 for positive x, -1 for negative x and 0 for 0
  */
int16_t signum_t(size_t x);

/*****************************************************************
  * @brief calculate sigmoid function see https://en.wikipedia.org/wiki/Sigmoid_function
  * @param  x - value, a - x-axis compression ratio
  * @retval 0.0 .. 1.0
  */
float sigmoida(float x, float a);

/*****************************************************************
  * @brief calculate sigmoid function see https://en.wikipedia.org/wiki/Sigmoid_function
  * @param  x - value, a - x-axis compression ratio
  * @retval -1.0 .. 1.0
  */
float sigmoida_zero(float x, float a);

/*****************************************************************
  * @brief simple alpha-beta filter 
  * @param  act - actua value, last - last actual value, deep - filtration factor, must be 2 power
  * @retval filtered value
  */
uint16_t alphabeta(uint16_t act, uint16_t last, uint8_t deep);

size_t SimpleRamp_IT(size_t actual, size_t SP, size_t min, size_t max, size_t step);

float MovingAverageFilter(float act, float last, float k);

#ifdef __cplusplus
}
#endif

#endif /* _FUNCTION_H_ */
