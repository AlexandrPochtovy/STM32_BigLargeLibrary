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

 * 	I2C_HW.h
 *	Created on: 30 nov. 2021
 */
#ifndef _MYI2C_H_
#define _MYI2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f4xx_ll_i2c.h>
#include "Peripherals/DataTypes.h"
#include "FIFObuffer/FIFObuffer.h"

typedef enum I2C_Mode {
	I2C_MODE_WRITE,	//for write
	I2C_MODE_READ,	//for read
	I2C_MODE_RW		//for read as write restart internal use only
} I2C_Mode_t;

typedef struct I2C_IRQ_Conn {
	I2C_TypeDef *i2c;	//pointer to HW i2c bus
	PortStatus_t status;//status I2C bus
	uint8_t step;		//step processing
	uint8_t addr;		//device I2C address
	uint8_t len;		//length data
	I2C_Mode_t mode;	//device mode
	fifo_t *buffer;		//pointer circular buffer
} I2C_IRQ_Conn_t;

typedef struct I2C_DMA_Conn {
	I2C_TypeDef *i2c;	//pointer to HW i2c bus
	void *DMAx;			//pointer to DMA MCU peripheral
	uint32_t Channel;	//pointer to dma channel
	PortStatus_t status;//status I2C bus
	uint8_t step;		//step processing
	uint8_t addr;		//device I2C address
	uint8_t reg;		//register I2C
	uint8_t len;		//length data
	I2C_Mode_t mode;	//device mode
	uint8_t *buffer;	//pointer linear buffer
} I2C_DMA_Conn_t;

/*	control function	******************************************/
void I2C_Start_IRQ(I2C_IRQ_Conn_t *_i2c);//запускает обмен и устанавливает флаг "занято" для устройства
void I2C_Start_DMA(I2C_DMA_Conn_t *_i2c);//запускает обмен и устанавливает флаг "занято" для устройства
void ClearBusyI2C1(void);//сбрасывает флаг зависшей шины согласно эррате
/*	interrupt processing function	******************************/
void I2C_Raw_IRQ_CallBack(I2C_IRQ_Conn_t *_i2c);
void I2C_EV_IRQ_DMA_CallBack(I2C_DMA_Conn_t *_i2c);
void I2C_ERR_IRQ_CallBack(I2C_IRQ_Conn_t *_i2c);

#ifdef __cplusplus
}
#endif

#endif /* _MYI2C_H_ */
