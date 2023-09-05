/*********************************************************************************
	Original author: user

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
   
 * BytesMath.h
 * Created on: Sep 4, 2023
 ********************************************************************************/

#ifndef FUNCTION_BYTESMATH_H_
#define FUNCTION_BYTESMATH_H_

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>

size_t __Min(size_t val, size_t min) {
	return val > min ? val : min;
}
/*****************************************************************
  * @brief
  * @param
  * @retval
  */
size_t __Max(size_t val, size_t max) {
	return val < max ? val : max;
}

/*****************************************************************
  * @brief  concat two bytes into word
  * @param  msb - hi byte, lsb - lo byte
  * @retval uint16_t
  */
size_t CONCAT_TWO_BYTES(uint8_t msb, uint8_t lsb) {
	return (((size_t)msb << 8) | (size_t)lsb);
}

size_t CONCAT_FOUR_BYTES(uint8_t hh_b, uint8_t h_b, uint8_t l_b, uint8_t ll_b) {
	return ((size_t)hh_b << 24) | ((size_t)h_b << 16) | ((size_t)l_b << 8) | (size_t)ll_b;
}


#endif /* FUNCTION_BYTESMATH_H_ */
