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
   
 * I2C_API.h
 * Created on: Aug 30, 2023
 ********************************************************************************/

#ifndef I2C_MIDDLELEVEL_I2C_API_H_
#define I2C_MIDDLELEVEL_I2C_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Peripherals/I2C_LowLevel/I2C_HW.h"

	__STATIC_INLINE void BusRequestOn(uint32_t req, uint32_t mask) {
		req |= mask;
	}
	__STATIC_INLINE void BusRequestOff(uint32_t req, uint32_t mask) {
		req &= ~mask;
	}
	/* @brief
 * write 3 bytes:addr reg value
 * @param
 * i2c connection, addr, register and value data
 * @retval
 * 0 - processing, 1 - complite
*/
uint8_t WriteOneRegByte(I2C_IRQ_Conn_t *_i2c, uint8_t addr, uint8_t reg, uint8_t value);

/*
 * @brief
 * write many bytes:addr, reg, values array
 * @param
 * i2c connection, addr, register and data array
 * @retval
 * 0 - processing, 1 - complite
*/
uint8_t WriteRegBytes(I2C_IRQ_Conn_t *_i2c, uint8_t addr, uint8_t reg, void *data, uint8_t size);

/*****************************************************************
 * @brief
 * read one byte :addr, reg, value
 * @param
 * i2c connection, addr, register and pointer data
 * @retval
 * 0 - processing, 1 - complite
*/
uint8_t ReadOneRegByte(I2C_IRQ_Conn_t *_i2c, uint8_t addr, uint8_t reg, uint8_t *value);

/*****************************************************************
  * @brief
  * @param
  * @retval
  */
uint8_t ReadRegBytes(I2C_IRQ_Conn_t *_i2c, uint8_t addr, uint8_t reg, void *data, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif /* I2C_MIDDLELEVEL_I2C_API_H_ */
