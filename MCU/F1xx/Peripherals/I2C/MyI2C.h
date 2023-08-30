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

 * 	MyI2C.h
 *	Created on: 30 nov. 2020
 */
#ifndef _MYI2C_H_
#define _MYI2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "DataTypes/InterfaceDataTypes.h"
#include "FIFObuffer/FIFObuffer.h"
#include "Peripherals/DMA_Template/DMA_Template.h"



/*	control function	******************************************/
void I2C_Start_IRQ(I2C_IRQ_Conn_t *_i2c);
void I2C_Start_DMA(I2C_DMA_Conn_t *_i2c);
void ClearBusyI2C1(void);
/*	interrupt processing function	******************************/
void I2C_Raw_IRQ_CallBack(I2C_IRQ_Conn_t *_i2c);
void I2C_EV_IRQ_DMA_CallBack(I2C_DMA_Conn_t *_i2c);
void I2C_ERR_CallBack(I2C_IRQ_Conn_t *_i2c);

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

#endif /* _MYI2C_H_ */
