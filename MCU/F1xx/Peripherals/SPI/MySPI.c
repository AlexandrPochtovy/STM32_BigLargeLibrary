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

 * 	MySPI.c
 *  Created on: 30 nov 2020
 */
#ifdef SPI_H

#include "MySPI.h"

void SPI_Start_IRQ_HWNSS(SPI_Connection_t *_spi) {
	_spi->status = PORT_BUSY;
	_spi->SPIbus->CR2 |= (SPI_CR2_TXEIE | SPI_CR2_RXNEIE); //включили прерывание чтобы данные пошли
	/*попробовать просто отключать интерфейс без постоянного включения/выключения прерываний*/
	GetOne(&_spi->txbuffer, ((uint8_t*) &_spi->SPIbus->DR)); //записали регистр который читаем пишем
	LL_SPI_Enable(_spi->SPIbus); //enable SPI
}

void SPI_Start_DMA_HWNSS(SPI_Connection_t *_spi) {
	__NOP(); //prototype
}

void SPI_IRQ_CallBack(SPI_Connection_t *_spi) {
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
			PutOne(&_spi->rxbuffer, _spi->SPIbus->DR); //read byte
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
				GetOne(&_spi->txbuffer, (uint8_t *)&_spi->SPIbus->DR);
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

void SPI_IRQ_DMA_CallBack(SPI_Connection_t *_spi) {
	__NOP(); //prototype
}

#endif
