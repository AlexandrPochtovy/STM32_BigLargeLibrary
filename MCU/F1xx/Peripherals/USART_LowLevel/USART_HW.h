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

 * 	USART_HW.h
 *  Created on: Sep 17, 2020
 */
#ifndef SRC_MYUSART_H_
#define SRC_MYUSART_H_

#include <stddef.h>
#include <stdint.h>
#include "stm32f1xx_ll_usart.h"
#include "FIFObuffer/FIFObuffer.h"
#include "DataTypes.h"

#ifdef DMA_H
	#include "DMA_Template/DMA_Template.h"
#endif

typedef struct USART_Conn {
	USART_TypeDef *USART;	  //pointer to HW USART port
	PortStatus_t txStatus;	//status USART port
	fifo_t *txbuffer;		    //pointer circular buffer
	uint8_t txlen;			    //length data
	PortStatus_t rxStatus;	//status USART port
	fifo_t *rxbuffer;		    //pointer circular buffer
	uint8_t rxlen;			    //length data
} USART_Conn_t;

//==============================================================================================
void USART_Send(USART_Conn_t *_usart, uint8_t len);//пишет в порт и устанавливает флаг "занято" для устройства
void USART_Receive(USART_Conn_t *usart);//читает из порта пока не будет тишина
void USART_Start_DMA(USART_Conn_t *usart);//запускает обмен и устанавливает флаг "занято" для устройства
/*	interrupt processing function	******************************/
void USART_EV_IRQ_CallBack(USART_Conn_t *usart);
void USART_EV_DMA_CallBack(USART_Conn_t *usart);
/*********************************************************************************/

#endif /* SRC_MYFUNCTION_MYUSART_H_ */

