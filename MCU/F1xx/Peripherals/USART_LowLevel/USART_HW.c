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

 * 	MyUsart.c
 *  Created on: Sep 17, 2020
 */

#include "MyUsart.h"

/*
 *
 * */
void USART_Send(USART_Conn_t *usart, uint8_t len) {
	usart->txStatus = PORT_BUSY;
	usart->txlen = len;
	usart->USART->CR1 |= (  //USART_CR1_RE 		| // Receiver Enable
	    USART_CR1_TE |  // Transmitter Enable
	        //USART_CR1_IDLEIE	|// IDLE interrupt enable
	        //USART_CR1_RXNEIE 	|// RXNE interrupt enable
	        USART_CR1_TCIE |// Transmission complete interrupt enable
	        USART_CR1_TXEIE |  // TXE interrupt enable
	        //USART_CR1_PEIE	|// PE interrupt enable
	        USART_CR1_UE);
}

void USART_Receive(USART_Conn_t *usart) {  //читает из порта пока не будет тишина
	usart->rxStatus = PORT_BUSY;
	usart->rxlen = 0;
	usart->USART->CR1 |= ( USART_CR1_RE |  // Receiver Enable
	    //USART_CR1_TE 		| // Transmitter Enable
	    USART_CR1_IDLEIE |// IDLE interrupt enable
	    USART_CR1_RXNEIE |  // RXNE interrupt enable
	    //USART_CR1_TCIE 		|// Transmission complete interrupt enable
	    //USART_CR1_TXEIE 	|// TXE interrupt enable
	    //USART_CR1_PEIE	|// PE interrupt enable
	    USART_CR1_UE);
}
void USART_Start_DMA(USART_Conn_t *usart) {  //запускает обмен и устанавливает флаг "занято" для устройства
}

void USART_EV_IRQ_CallBack(USART_Conn_t *usart) {
	volatile uint16_t SR = usart->USART->SR;  //read status register
	if (SR
	    & (USART_SR_RXNE | USART_SR_PE | USART_SR_IDLE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
		//OverRun Error Noise Error   Framing Error
		if (SR & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) {
			(void)usart->USART->DR;
			return;
		}
	}

	if (SR & USART_SR_PE) {  //Parity Error
		__NOP();
		(void)usart->USART->DR;
		return;
	}
//-----------------------		Receive		---------------------------------
	if (SR & USART_SR_IDLE) {  // IDLE line detected
		__NOP();
		(void)usart->USART->DR;
		usart->rxStatus = PORT_FREE;
		return;
	}
	if (SR & USART_SR_RXNE) {  //Read Data Register Not Empty
		__NOP();
		FIFO_PutOne(usart->rxbuffer, usart->USART->DR);
		++usart->rxlen;
	}
//-----------------------		Transmission		---------------------------------
	if ((SR & USART_SR_TC) && (usart->txlen == 0)) {  // Transmission Complete, clear - write 0 to USART_SR_TC
		usart->USART->SR = SR & (~USART_SR_TC);  //clear TC flag
		usart->USART->CR1 &= ~(USART_CR1_TE | USART_CR1_TCIE);  //disable transmission and TC interrupt
		usart->txStatus = PORT_FREE;
	}
	else if (SR & USART_SR_TXE) {  // Transmit Data Register Empty, clear - write DR
		FIFO_GetOne(usart->txbuffer, ((uint8_t*)&usart->USART->DR));
		--usart->txlen;
		if (usart->txlen == 0) {  //send all byte
			usart->USART->CR1 &= ~USART_CR1_TXEIE;  //disable TXE interrupt, goto TC flag
		}
	}
}

void USART_EV_DMA_CallBack(USART_Conn_t *_usart) {

}
