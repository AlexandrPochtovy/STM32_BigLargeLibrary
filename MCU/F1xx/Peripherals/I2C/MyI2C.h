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

//========================================================================================================================
/*	команда работы с устройством: чтение или запись данных, используется как переключатель
*	при переходе от записи к чтению при работе с шиной, внутренний признак, используется только в обработчике прерываний
 */
typedef enum I2C_Mode {
	I2C_MODE_WRITE,	//for write
	I2C_MODE_READ,	//for read
	I2C_MODE_RW			//for read as write restart
} I2C_Mode_t;

/* структура работы с шиной состоит из:
* указателя на аппаратную шину
* статуса шины
* параметров запроса: адрес устройства, режим чтение/запись, длина запроса
* указатель на структуру кольцевого буфера
*/
typedef struct I2C_IRQ_Connection {
	I2C_TypeDef *i2c;		//pointer to HW i2c bus
	PortStatus_t status;//status I2C bus
	uint8_t step;				//step processing
	uint8_t addr;				//device I2C address
	uint8_t len;				//length data
	I2C_Mode_t mode;		//device mode
	fifo_t *buffer;			//pointer circular buffer
} I2C_IRQ_Connection_t;

typedef struct I2C_DMA_Connection {
	I2C_TypeDef *i2c;		//pointer to HW i2c bus
	void *dma_channel;	//pointer to dma channel restart function
	PortStatus_t status;//status I2C bus
	uint8_t step;				//step processing
	uint8_t addr;				//device I2C address
	uint8_t len;				//length data
	I2C_Mode_t mode;		//device mode
	fifo_t *buffer;			//pointer circular buffer
} I2C_DMA_Connection_t;

/*	control function	******************************************/
void I2C_Start_IRQ(I2C_IRQ_Connection_t *_i2c);//запускает обмен и устанавливает флаг "занято" для устройства
void I2C_Start_DMA(I2C_DMA_Connection_t *_i2c);//запускает обмен и устанавливает флаг "занято" для устройства
void ClearBusyI2C1(void);//сбрасывает флаг зависшей шины согласно эррате
/*	interrupt processing function	******************************/
void I2C_Raw_IRQ_CallBack(I2C_IRQ_Connection_t *_i2c);
void I2C_EV_IRQ_DMA_CallBack(I2C_DMA_Connection_t *_i2c);
void I2C_ERR_IRQ_CallBack(I2C_IRQ_Connection_t *_i2c);

/* @brief
 * write 3 bytes:addr reg value
 * @param
 * i2c connection, addr, register and value data
 * @retval
 * 0 - processing, 1 - complite
*/
uint8_t WriteOneRegByte(I2C_IRQ_Connection_t *_i2c, uint8_t addr, uint8_t reg, uint8_t value);

/*
 * @brief
 * write many bytes:addr, reg, values array
 * @param
 * i2c connection, addr, register and data array
 * @retval
 * 0 - processing, 1 - complite
*/
uint8_t WriteRegBytes(I2C_IRQ_Connection_t *_i2c, uint8_t addr, uint8_t reg, void *data, uint8_t size);

/*****************************************************************
 * @brief
 * read one byte :addr, reg, value
 * @param
 * i2c connection, addr, register and pointer data
 * @retval
 * 0 - processing, 1 - complite
*/
uint8_t ReadOneRegByte(I2C_IRQ_Connection_t *_i2c, uint8_t addr, uint8_t reg, uint8_t *value);

/*****************************************************************
  * @brief
  * @param
  * @retval
  */
uint8_t ReadRegBytes(I2C_IRQ_Connection_t *_i2c, uint8_t addr, uint8_t reg, void *data, uint8_t size);


#ifdef __cplusplus
}
#endif

#endif /* _MYI2C_H_ */
