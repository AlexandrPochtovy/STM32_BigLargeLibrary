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
   
 * SPI_API.h
 * Created on: Sep 13, 2023
 ********************************************************************************/

#ifndef _SPI_MIDDLELEVEL_SPI_API_H_
#define _SPI_MIDDLELEVEL_SPI_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Peripherals/SPI_LowLevel/SPI_HW.h"

/*****************************************************************
  * @brief  write 3 bytes in i2c bus: addr, reg, value in interrupt mode.
  * @param  *_i2c - pointer to i2c connection struct
  * @retval 0 - processing, 1 - complite
  */
uint8_t WriteOneRegByte(SPI_IRQ_Conn_t *_spi, uint8_t reg, uint8_t value);

/*****************************************************************
  * @brief  write multiple bytes in i2c bus: addr, reg, value's array in interrupt mode.
  * @param  *_i2c - pointer to i2c connection struct
  * @retval 0 - processing, 1 - complite
  */
uint8_t WriteRegBytes(I2C_IRQ_Conn_t *_i2c, uint8_t addr, uint8_t reg, void *data, uint8_t size);

/*****************************************************************
  * @brief  read one byte frow i2c bus, but send 2 before: addr and reg
  * @param  *_i2c - pointer to i2c connection struct
  * @retval 0 - processing, 1 - complite
  */
uint8_t ReadOneRegByte(I2C_IRQ_Conn_t *_i2c, uint8_t addr, uint8_t reg, uint8_t *value);

/*****************************************************************
  * @brief  read multiple bytes frow i2c bus, but send 2 before: addr and reg
  * @param  *_i2c - pointer to i2c connection struct
  * @retval 0 - processing, 1 - complite
  */
uint8_t ReadRegBytes(I2C_IRQ_Conn_t *_i2c, uint8_t addr, uint8_t reg, void *data, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif /* _I2C_MIDDLELEVEL_I2C_API_H_ */
