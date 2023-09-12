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
   
 *  DMA_Template.h
 *  Created on: 03.05.2021
 ********************************************************************************/

#ifndef DMA_TEMPLATE_H_
#define DMA_TEMPLATE_H_

#ifdef __cplusplus
extern "C" {
#endif
/* include -------------------------------------------------------------------*/
#include "stm32f1xx.h"

void DMA_Ch1_IRQ_Callback(DMA_TypeDef *DMAnum);
void DMA_Ch2_IRQ_Callback(DMA_TypeDef *DMAnum);//SPI1 RX
void DMA_Ch3_IRQ_Callback(DMA_TypeDef *DMAnum);//SPI1 TX
void DMA_Ch4_IRQ_Callback(DMA_TypeDef *DMAnum);//I2C2 TX
void DMA_Ch5_IRQ_Callback(DMA_TypeDef *DMAnum);//I2C2 RX
void DMA_Ch6_IRQ_Callback(DMA_TypeDef *DMAnum);//USART2 RX
void DMA_Ch7_IRQ_Callback(DMA_TypeDef *DMAnum);//USART2 TX
/*----------------------------------------------------------------------------*/
void DMA_Ch1_Restart(DMA_TypeDef *DMAnum, uint8_t *data, uint16_t lenght);
void DMA_Ch2_Restart(DMA_TypeDef *DMAnum, uint8_t *data, uint16_t lenght);
void DMA_Ch3_3estart(DMA_TypeDef *DMAnum, uint8_t *data, uint16_t lenght);
void DMA_Ch4_Restart(DMA_TypeDef *DMAnum, uint8_t *data, uint16_t lenght);
void DMA_Ch5_Restart(DMA_TypeDef *DMAnum, uint8_t *data, uint16_t lenght);
void DMA_Ch6_Restart(DMA_TypeDef *DMAnum, uint8_t *data, uint16_t lenght);
void DMA_Ch7_Restart(DMA_TypeDef *DMAnum, uint8_t *data, uint16_t lenght);
/*----------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* DMA_TEMPLATE_H_ */
