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

 * 	SPI_HW.c
 *  Created on: 08 mar 2023
 */

#include "SPI_HW.h"

void SPI_Start_IRQ_TWO_HWNSS(SPI_Conn_TWO_t *_spi) {
	_spi->status = PORT_BUSY;
	_spi->SPIbus->CR2 |= (SPI_CR2_TXEIE | SPI_CR2_RXNEIE); //включили прерывание чтобы данные пошли
	/*попробовать просто отключать интерфейс без постоянного включения/выключения прерываний*/
	FIFO_GetOne(_spi->txbuffer, (uint8_t*)&_spi->SPIbus->DR); //записали регистр который читаем пишем
	LL_SPI_Enable(_spi->SPIbus); //enable SPI
}

void SPI_Start_IRQ_ONE_HWNSS(SPI_Conn_ONE_t *_spi) {
	_spi->status = PORT_BUSY;
	if (_spi->mode == SPI_HALFDUPLEX_WRITE) {
		_spi->SPIbus->CR2 |= SPI_CR2_TXEIE;
	} else if (_spi->mode == SPI_HALFDUPLEX_READ) {
		_spi->SPIbus->CR2 |= SPI_CR2_RXNEIE;
	}
	/*попробовать просто отключать интерфейс без постоянного включения/выключения прерываний*/
	LL_SPI_Enable(_spi->SPIbus); //enable SPI
}

void SPI_Start_DMA_ONE_HWNSS(SPI_Conn_ONE_t *_spi) {
	__NOP(); //prototype
}

void SPI_IRQ_TWO_CallBack(SPI_Conn_TWO_t *_spi) {
	_spi->status = PORT_BUSY;
	volatile uint16_t SPI_SR = _spi->SPIbus->SR;
	if (SPI_SR & (SPI_SR_MODF | SPI_SR_OVR | SPI_SR_CRCERR)) {/* Mode fault */
		__NOP();
		(void) _spi->SPIbus->DR;//clear data register
		if (SPI_SR & SPI_SR_OVR) { /*!< Overrun flag */
			(void) _spi->SPIbus->SR;//add read status register
		}
		if (SPI_SR & SPI_SR_CRCERR) { /*!< CRC Error flag */
			_spi->SPIbus->SR &= ~SPI_SR_CRCERR;//clear CRC error flag
		}
		LL_SPI_Disable(_spi->SPIbus);	//SPI off
		_spi->status = PORT_ERROR;		//set error status
		return;
	}
	if (SPI_SR & SPI_SR_RXNE) {
		if (_spi->rxlen > 0) {
			FIFO_PutOne(_spi->rxbuffer, _spi->SPIbus->DR); //read byte
			--_spi->rxlen;
		} else if (_spi->txlen > 0) {
			(void)_spi->SPIbus->DR;
		} else {
			_spi->SPIbus->CR2 &= ~SPI_CR2_RXNEIE; //interrupt off
		}
		return;
	}
	if (SPI_SR & SPI_SR_TXE) {
		if (_spi->txlen > 0) {
				FIFO_GetOne(_spi->txbuffer, (uint8_t *)&_spi->SPIbus->DR);
			--_spi->txlen;
		} else if (_spi->rxlen > 0) {
			_spi->SPIbus->DR = 0xFF;
		} else {
			while (SPI_SR & SPI_SR_BSY) {__NOP();}
			_spi->SPIbus->CR2 &= ~SPI_CR2_TXEIE;
			LL_SPI_Disable(_spi->SPIbus);
			_spi->status = PORT_DONE;
		}
		return;
	}
}

void SPI_IRQ_TO_CallBack(SPI_Conn_ONE_t *_spi) {
	_spi->status = PORT_BUSY;
	volatile uint16_t SPI_SR = _spi->SPIbus->SR;
	/*if (SPI_SR & (SPI_SR_MODF | SPI_SR_OVR | SPI_SR_CRCERR)) {// Mode fault
		__NOP();
		(void) _spi->SPIbus->DR;//clear data register
		if (SPI_SR & SPI_SR_OVR) { // Overrun flag
			(void) _spi->SPIbus->SR;//add read status register
		}
		if (SPI_SR & SPI_SR_CRCERR) { //< CRC Error flag
			_spi->SPIbus->SR &= ~SPI_SR_CRCERR;//clear CRC error flag
		}
		LL_SPI_Disable(_spi->SPIbus);	//SPI off
		_spi->status = PORT_ERROR;		//set error status
		return;
	}*/
	if (SPI_SR & SPI_SR_TXE) {
 		if (_spi->len > 0) {
 			uint8_t tmp;
			FIFO_GetOne(_spi->buffer, &tmp);
			_spi->SPIbus->DR = tmp;
			--_spi->len;
		}
		else {
			//while (SPI_SR & SPI_SR_BSY) {__NOP();}
			_spi->SPIbus->CR2 &= ~SPI_CR2_TXEIE;
			LL_SPI_Disable(_spi->SPIbus);
			_spi->status = PORT_DONE;
		}
	}
}

void SPI_IRQ_RO_CallBack(SPI_Conn_ONE_t *_spi) {
	_spi->status = PORT_BUSY;
	volatile uint16_t SPI_SR = _spi->SPIbus->SR;
	/*if (SPI_SR & (SPI_SR_MODF | SPI_SR_OVR | SPI_SR_CRCERR)) {// Mode fault
		__NOP();
		(void) _spi->SPIbus->DR;//clear data register
		if (SPI_SR & SPI_SR_OVR) { // Overrun flag
			(void) _spi->SPIbus->SR;//add read status register
		}
		if (SPI_SR & SPI_SR_CRCERR) { //< CRC Error flag
			_spi->SPIbus->SR &= ~SPI_SR_CRCERR;//clear CRC error flag
		}
		LL_SPI_Disable(_spi->SPIbus);	//SPI off
		_spi->status = PORT_ERROR;		//set error status
		return;
	}*/
	if (SPI_SR & SPI_SR_RXNE) {
		if (_spi->len > 1) {
			FIFO_PutOne(_spi->buffer, (uint8_t)_spi->SPIbus->DR); //read byte
			--_spi->len;
		}
		else {
			_spi->SPIbus->CR2 &= ~SPI_CR2_RXNEIE; //interrupt off
			FIFO_PutOne(_spi->buffer, (uint8_t)_spi->SPIbus->DR); //read last byte
			LL_SPI_Disable(_spi->SPIbus);
			_spi->status = PORT_DONE;
		}
	}
}
