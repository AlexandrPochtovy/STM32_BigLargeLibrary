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
   
 * SPI_API.c
 * Created on: Aug 30, 2023
 ********************************************************************************/

#include "SPI_API.h"

uint8_t SPI_WriteReadOneByte(SPI_Conn_TWO_t *_spi, uint8_t tx, uint8_t *rx) {
	switch (_spi->status) {
	case PORT_FREE:
		_spi->status = PORT_BUSY;
		FIFO_PutOne(_spi->txbuffer, tx);
		_spi->txlen = 1;
        _spi->rxlen = 1;
		_spi->mode = SPI_FULLDUPLEX_RW;
		SPI_Start_IRQ_HWNSS(_spi);
		return 0;
		break;
	case PORT_DONE:
		_spi->txbuffer->status = BUFFER_FREE;
		_spi->rxbuffer->status = BUFFER_FREE;
		FIFO_GetOne(_spi->rxbuffer, rx);
		_spi->status = PORT_FREE;
		return 1;// exit
		break;
	case PORT_ERROR://TODO ADD error processing
		_spi->txbuffer->status = BUFFER_FREE;
		_spi->rxbuffer->status = BUFFER_FREE;
		FIFO_Init(_spi->txbuffer);
        FIFO_Init(_spi->rxbuffer);
		_spi->status = PORT_FREE;
		return 1;
		break;
	default:
		return 0;
		break;
	}
}

uint8_t SPI_WriteReadBytes(SPI_Conn_TWO_t *_spi, uint8_t *tx, uint8_t txLen, uint8_t *rx, uint8_t rxLen) {
	switch (_spi->status) {
	case PORT_FREE:
		_spi->status = PORT_BUSY;
		FIFO_PutMulti(_spi->txbuffer, tx, txLen);
		_spi->txlen = txLen;
        _spi->rxlen = rxLen;
		_spi->mode = SPI_FULLDUPLEX_RW;
		SPI_Start_IRQ_HWNSS(_spi);
		return 0;
		break;
	case PORT_DONE:
		_spi->txbuffer->status = BUFFER_FREE;
		_spi->rxbuffer->status = BUFFER_FREE;
		FIFO_GetMulti(_spi->rxbuffer, rx, rxLen);
		_spi->status = PORT_FREE;
		return 1;// exit
		break;
	case PORT_ERROR://TODO ADD error processing
		_spi->txbuffer->status = BUFFER_FREE;
		_spi->rxbuffer->status = BUFFER_FREE;
		FIFO_Init(_spi->txbuffer);
        FIFO_Init(_spi->rxbuffer);
		_spi->status = PORT_FREE;
		return 1;
		break;
	default:
		return 0;
		break;
	}
}

uint8_t SPI_WriteOnlyOneByte(SPI_Conn_ONE_t *_spi, uint8_t tx) {
	switch (_spi->status) {
	case PORT_FREE:
		_spi->status = PORT_BUSY;
		FIFO_PutOne(_spi->buffer, tx);
		_spi->len = 1;
		_spi->mode = SPI_HALFDUPLEX_WRITE;
		SPI_Start_IRQ_ONE_HWNSS(_spi);
		return 0;
		break;
	case PORT_DONE:
		_spi->buffer->status = BUFFER_FREE;
		_spi->status = PORT_FREE;
		return 1;// exit
		break;
	case PORT_ERROR://TODO ADD error processing
		_spi->buffer->status = BUFFER_FREE;
		FIFO_Init(_spi->buffer);
		_spi->status = PORT_FREE;
		return 1;
		break;
	default:
		return 0;
		break;
	}
}

uint8_t SPI_WriteOnlyBytes(SPI_Conn_ONE_t *_spi, uint8_t *tx, uint8_t txLen) {
	switch (_spi->status) {
	case PORT_FREE:
		_spi->status = PORT_BUSY;
		FIFO_PutMulti(_spi->buffer, tx, txLen);
		_spi->len = txLen;
		_spi->mode = SPI_HALFDUPLEX_WRITE;
		SPI_Start_IRQ_ONE_HWNSS(_spi);
		return 0;
		break;
	case PORT_DONE:
		_spi->buffer->status = BUFFER_FREE;
		_spi->status = PORT_FREE;
		return 1;// exit
		break;
	case PORT_ERROR://TODO ADD error processing
		_spi->buffer->status = BUFFER_FREE;
		FIFO_Init(_spi->buffer);
		_spi->status = PORT_FREE;
		return 1;
		break;
	default:
		return 0;
		break;
	}
}

uint8_t SPI_ReadOnlyOneByte(SPI_Conn_ONE_t *_spi, uint8_t *rx) {
	switch (_spi->status) {
	case PORT_FREE:
		_spi->status = PORT_BUSY;
		_spi->len = 1;
		_spi->mode = SPI_HALFDUPLEX_READ;
		SPI_Start_IRQ_ONE_HWNSS(_spi);
		return 0;
		break;
	case PORT_DONE:
		_spi->buffer->status = BUFFER_FREE;
		FIFO_GetOne(_spi->buffer, rx);
		_spi->status = PORT_FREE;
		return 1;// exit
		break;
	case PORT_ERROR://TODO ADD error processing
		_spi->buffer->status = BUFFER_FREE;
		FIFO_Init(_spi->buffer);
		_spi->status = PORT_FREE;
		return 1;
		break;
	default:
		return 0;
		break;
	}
}

uint8_t SPI_ReadOnlyBytes(SPI_Conn_ONE_t *_spi, uint8_t *rx, uint8_t rxLen) {
	switch (_spi->status) {
	case PORT_FREE:
		_spi->status = PORT_BUSY;
		_spi->len = rxLen;
		_spi->mode = SPI_HALFDUPLEX_READ;
		SPI_Start_IRQ_ONE_HWNSS(_spi);
		return 0;
		break;
	case PORT_DONE:
		_spi->buffer->status = BUFFER_FREE;
		FIFO_GetMulti(_spi->buffer, rx, rxLen);
		_spi->status = PORT_FREE;
		return 1;// exit
		break;
	case PORT_ERROR://TODO ADD error processing
		_spi->buffer->status = BUFFER_FREE;
		FIFO_Init(_spi->buffer);
		_spi->status = PORT_FREE;
		return 1;
		break;
	default:
		return 0;
		break;
	}
}

