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
   
 * InterfaceDataTypes.h
 * Created on: 24/02/2022
 ********************************************************************************/

#ifndef INTERFACEDATATYPES_H_
#define INTERFACEDATATYPES_H_

#include <stm32f407xx.h>
#include "CommonDataTypes.h"
#include "FIFObuffer/FIFObuffer.h"
#include "FILObuffer/FILObuffer.h"

/************************************************************************************
*									I2C												*
************************************************************************************/
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

/************************************************************************************
*									SPI												*
************************************************************************************/
typedef enum SPI_Mode {
	SPI_MODE_WRITE,		//half-duplex write only
	SPI_MODE_READ,		//half-duplex read only
	SPI_MODE_DUPLEX,	//full-duplex write and read both
	SPI_MODE_RO,			//receive-only mode (3 wire)
	SPI_MODE_TO				//transmit-only mode (3 wire)
} SPI_Mode_t;

typedef struct SPI_Conn_TWO {
	SPI_TypeDef *SPIbus;	        //pointer to HW SPI port
	volatile PortStatus_t status;//status port
	volatile SPI_Mode_t mode;			//read write mode
	fifo_t *txbuffer;		          //pointer circular buffer
	volatile uint8_t txlen;			  //length data
	fifo_t *rxbuffer;		          //pointer circular buffer
	volatile uint8_t rxlen;			  //length data
} SPI_Conn_TWO_t;

typedef struct SPI_Conn_ONE {
	SPI_TypeDef *SPIbus;	        //pointer to HW SPI port
	volatile PortStatus_t status;//status port
	volatile SPI_Mode_t mode;			//read write mode
	fifo_t *data;		          //pointer circular buffer
	volatile uint8_t len;			  //length data
} SPI_Conn_ONE_t;

/************************************************************************************
*									USART											*
************************************************************************************/
	/*	общая структура соединения с любым устройством на шине состоит из:
	 * структуры запроса по шине: адрес устройства, адрес регистра, длина запроса, режим чтение/запись
	 * структуры работы с шиной: аппаратный адрес шины, состояние шины, буфер приема/передачи
	 * состояния устройства: не настроено, настроено и готово, ошибка
	 */
	typedef struct USART_Conn {
		USART_TypeDef *USART;	//pointer to HW USART port
		PortStatus_t txStatus;	//status USART port
		fifo_t *txbuffer;		//pointer circular buffer
		uint8_t txlen;			//length data
		PortStatus_t rxStatus;	//status USART port
		fifo_t *rxbuffer;		//pointer circular buffer
		uint8_t rxlen;			//length data
	} USART_Conn_t;


#endif /* INTERFACEDATATYPES_H_ */
