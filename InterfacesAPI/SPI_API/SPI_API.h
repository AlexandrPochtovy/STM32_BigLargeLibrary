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
  * @brief write 1 byte to spi and read 1 byte from spi in full-duplex mode in interrupt
  * @param _spi - pointer to spi full duplex connection struct
  * @param tx - data 1 byte for write
  * @param rx - pointer to data 1 byte to read
  * @retval 0 - processing, 1 - complite
  */
uint8_t SPI_WriteReadOneByte(SPI_Conn_TWO_t *_spi, uint8_t tx, uint8_t *rx);

/*****************************************************************
  * @brief  write multi bytes to spi and read multi byte from spi in full-duplex mode in interrupt
  * @param _spi - pointer to spi full duplex connection struct
  * @param tx - pointer to data bytes for write
  * @param txLen - size data for write
  * @param rx - pointer to data bytes for read
  * @param rxLen - size data for read
  * @retval 0 - processing, 1 - complite
  */
uint8_t SPI_WriteReadBytes(SPI_Conn_TWO_t *_spi, uint8_t *tx, uint8_t txLen, uint8_t *rx, uint8_t rxLen);

/*****************************************************************
  * @brief write 1 byte to spi in half-duplex mode in interrupt
  * @param _spi - pointer to spi half-duplex connection struct
  * @param tx - data 1 byte for write
  * @retval 0 - processing, 1 - complite
  */
uint8_t SPI_WriteOnlyOneByte(SPI_Conn_ONE_t *_spi, uint8_t tx);

/*****************************************************************
  * @brief write multi bytes to spi in half-duplex mode in interrupt
  * @param _spi - pointer to spi half-duplex connection struct
  * @param tx - pointer to data bytes for write
  * @param txLen - size data for write
  * @retval 0 - processing, 1 - complite
  */
uint8_t SPI_WriteOnlyBytes(SPI_Conn_ONE_t *_spi, uint8_t *tx, uint8_t txLen);

/*****************************************************************
  * @brief read 1 byte from spi in half-duplex mode in interrupt
  * @param _spi - pointer to spi half-duplex connection struct
  * @param rx - pointer to data 1 byte to read
  * @retval 0 - processing, 1 - complite
  */
uint8_t SPI_ReadOnlyOneByte(SPI_Conn_ONE_t *_spi, uint8_t *rx);

/*****************************************************************
  * @brief read multi byte from spi in half-duplex mode in interrupt
  * @param _spi - pointer to spi half-duplex connection struct
  * @param rx - pointer to data bytes for read
  * @param rxLen - size data for read
  * @retval 0 - processing, 1 - complite
  */
uint8_t SPI_ReadOnlyBytes(SPI_Conn_ONE_t *_spi, uint8_t *rx, uint8_t rxLen);

#ifdef __cplusplus
}
#endif

#endif /* _I2C_MIDDLELEVEL_I2C_API_H_ */
