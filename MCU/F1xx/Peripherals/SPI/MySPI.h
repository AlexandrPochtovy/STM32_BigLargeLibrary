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
#ifdef __SPI_H__

#ifndef SPI_MYSPI_H_
#define SPI_MYSPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "Common/DataTypes/InterfaceDataTypes.h"
#include "Common/FIFObuffer/FIFObuffer.h"
#include "DMA_Template/DMA_Template.h"

/*	control function	******************************************/
void SPI_Start_IRQ_HWNSS(SPI_Connection_t *_spi);//запускает обмен и устанавливает флаг "занято" для устройства
void SPI_Start_DMA_HWNSS(SPI_Connection_t *_spi);//запускает обмен и устанавливает флаг "занято" для устройства
/*	interrupt processing function	******************************/
void SPI_IRQ_CallBack(SPI_Connection_t *_spi);
void SPI_IRQ_DMA_CallBack(SPI_Connection_t *_spi);

#ifdef __cplusplus
}
#endif

#endif /* SRC_MYSPI_H_ */

#endif
