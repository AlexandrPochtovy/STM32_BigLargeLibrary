/*********************************************************************************
   Original author: Alexandr Pochtovy<alex.mail.prime@gmail.com>

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

#ifndef F4xx_MYSPI_H_
#define F4xx_MYSPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f4xx_ll_spi.h>
#include "DataTypes/InterfaceDataTypes.h"
//#include "DMA_Template/DMA_Template.h"


/*	control function	******************************************/
//запускает обмен и устанавливает флаг "занято" для устройства
void SPI_Start_IRQ_TWO_HWNSS(SPI_Conn_TWO_t *_spi);

//запускает обмен и устанавливает флаг "занято" для устройства
void SPI_Start_DMA_TWO_HWNSS(SPI_Conn_TWO_t *_spi);

//запускает обмен и устанавливает флаг "занято" для устройства
void SPI_Start_IRQ_ONE_HWNSS(SPI_Conn_ONE_t *_spi);

//запускает обмен и устанавливает флаг "занято" для устройства
void SPI_Start_DMA_ONE_HWNSS(SPI_Conn_ONE_t *_spi);
/*	interrupt processing function	******************************/
void SPI_IRQ_TWO_CallBack(SPI_Conn_TWO_t *_spi);

void SPI_IRQ_TO_CallBack(SPI_Conn_ONE_t *_spi);

void SPI_IRQ_RO_CallBack(SPI_Conn_ONE_t *_spi);

void SPI_IRQ_DMA_CallBack(SPI_Conn_TWO_t *_spi);

#ifdef __cplusplus
}
#endif

#endif /* SRC_MYSPI_H_ */
