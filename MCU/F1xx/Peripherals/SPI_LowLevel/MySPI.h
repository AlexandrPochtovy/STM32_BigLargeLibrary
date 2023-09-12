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

 * 	MySPI.h
 *  Created on: 30 nov 2020
 */
#ifndef SPI_MYSPI_H_
#define SPI_MYSPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include "stm32f1xx.h"
#include "stm32f1xx_ll_spi.h"
#include "FIFObuffer/FIFObuffer.h"
#include "DataTypes.h"

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


/*	control function	******************************************/
void SPI_Start_IRQ_HWNSS(SPI_Conn_TWO_t *_spi);//
void SPI_Start_DMA_HWNSS(SPI_Conn_TWO_t *_spi);//
void SPI_Start_IRQ_HWNSS(SPI_Conn_ONE_t *_spi);//
void SPI_Start_DMA_HWNSS(SPI_Conn_ONE_t *_spi);//
/*	interrupt processing function	******************************/
void SPI_IRQ_CallBack(SPI_Conn_TWO_t *_spi);
void SPI_IRQ_DMA_CallBack(SPI_Conn_TWO_t *_spi);
void SPI_IRQ_CallBack(SPI_Conn_ONE_t *_spi);
void SPI_IRQ_DMA_CallBack(SPI_Conn_ONE_t *_spi);

#ifdef __cplusplus
}
#endif

#endif /* SRC_MYSPI_H_ */
