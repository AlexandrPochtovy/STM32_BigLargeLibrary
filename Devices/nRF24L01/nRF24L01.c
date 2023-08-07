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

 * 	nRF24L01.c
 *  Created on: 23 feb 2022
 */

#include "nRF24L01.h"

static const uint8_t EN_ALL_PIPES_MASK = 0x3F;
static const uint8_t MagicVal = 0x73;							//ВЗЯТО ИЗ ПРИМЕРА

//====	SOME INTERNAL FUNCTION	=================
static inline size_t SET_FLAG(size_t value, size_t bit) {
	return ((value) |= (1 << bit));
}

static inline size_t CLEAR_FLAG(size_t value, size_t bit) {
	return ((value) &= (~(1 << bit)));
}

static inline uint8_t SET_CONFIG_MODE_TX(uint8_t cfg) {
	return cfg &= ~0x01;
}

static inline uint8_t SET_CONFIG_MODE_RX(uint8_t cfg) {
	return cfg |= 0x01;
}

static inline uint8_t SET_CONFIG_POWER_ON(uint8_t cfg) {
	return cfg |= 0x02;
}

static inline uint8_t SET_CONFIG_POWER_OFF(uint8_t cfg) {
	return cfg &= ~0x02;
}

static inline uint8_t SET_CONFIG(	uint8_t MASK_RX_DR, uint8_t MASK_TX_DS, uint8_t MASK_MAX_RT,
																	RF_CRC_t crc, uint8_t power, uint8_t mode) {
	return ((~MASK_RX_DR & 0x01) << 6) | ((~MASK_TX_DS & 0x01) << 5) | ((~MASK_MAX_RT & 0x01) << 4) |
			crc | (power << 1) | (mode << 0);
}

static inline uint8_t ENABLE_PIPES (uint8_t pipe0, uint8_t pipe1, uint8_t pipe2,
		uint8_t pipe3, uint8_t pipe4, uint8_t pipe5) {
	return 	((pipe0 << ENAA_P0) | (pipe1 << ENAA_P1) | (pipe2 << ENAA_P2) |
					(pipe3 << ENAA_P3) | (pipe4 << ENAA_P4) | (pipe5 << ENAA_P5)) & EN_ALL_PIPES_MASK;
}

static inline uint8_t WidthCheck (uint8_t width) {
	return ((width >=3) && (width <=5)) ? width : 5;
}

static inline uint8_t LenghtCheck (uint8_t size) {
	return (size > 0) && (size <=32) ? size : 32;
}

static inline uint8_t CheckChannel (uint8_t channel) {
	return ((channel & 0x7F) <= 125) ? channel : 125;
}

static inline uint8_t SET_RF_REG(	uint8_t CONT_WAVE, uint8_t PLL_LOCK, RF_Speed_t RF_Speed,
																	RF_Power_t RF_Power, uint8_t LNA) {
	return (CONT_WAVE << 6) | (PLL_LOCK << 6) | RF_Speed | RF_Power | LNA;
}

//====	SPI LOW LEVEL FUNCTION	=========================================================
	//write one byte command and read hw status
uint8_t WriteCommand(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t cmd) {
	switch (_spi->status) {
		case PORT_FREE:
			FIFO_PutOne(_spi->txbuffer, cmd);
			_spi->txlen = 1;
			_spi->rxlen = 1;
			SPI_Start_IRQ_HWNSS(_spi);
			return 0;
		case PORT_DONE:
			FIFO_GetOne(_spi->rxbuffer, &dev->CHIP_STATUS_REG);
			_spi->status = PORT_FREE;
			return 1;
		default: return 0;
	}
}
	//write one byte command and read hw status
uint8_t WriteShortCommand(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t cmd, uint8_t value) {
	switch (_spi->status) {
		case PORT_FREE:
			FIFO_PutOne(_spi->txbuffer, cmd);
			FIFO_PutOne(_spi->txbuffer, value);
			_spi->txlen = 2;
			_spi->rxlen = 1;
			SPI_Start_IRQ_HWNSS(_spi);
			return 0;
		case PORT_DONE:
			FIFO_GetOne(_spi->rxbuffer, &dev->CHIP_STATUS_REG);
			_spi->status = PORT_FREE;
			return 1;
		default: return 0;
	}
}
	//write multiple byte command and read hw status
uint8_t WriteLongComm(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t cmd, uint8_t *data, uint8_t len) {
	switch (_spi->status) {
		case PORT_FREE:
			FIFO_PutOne(_spi->txbuffer, cmd);
			FIFO_PutMulti(_spi->txbuffer, data, len);
			_spi->txlen = len + 1;
			_spi->rxlen = 1;
			SPI_Start_IRQ_HWNSS(_spi);
			return 0;
		case PORT_DONE:
			FIFO_GetOne(_spi->rxbuffer, &dev->CHIP_STATUS_REG);
			_spi->status = PORT_FREE;
			return 1;
		default:
 			return 0;
	}
}
//read one byte command and read hw status
uint8_t ReadShortComm(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t cmd, uint8_t *value) {
	switch (_spi->status) {
		case PORT_FREE:
			FIFO_PutOne(_spi->txbuffer, cmd);
			_spi->txlen = 1;
			_spi->rxlen = 2;
			SPI_Start_IRQ_HWNSS(_spi);
			return 0;
		case PORT_DONE:
			FIFO_GetOne(_spi->rxbuffer, &dev->CHIP_STATUS_REG);
			FIFO_GetOne(_spi->rxbuffer, value);
			_spi->status = PORT_FREE;
			return 1;
		default:
			return 0;
	}
}
//read multiple byte command and read hw status
uint8_t ReadLongComm(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t cmd, uint8_t *data, uint8_t len) {
	switch (_spi->status) {
		case PORT_FREE:
			FIFO_PutOne(_spi->txbuffer, cmd);
			_spi->txlen = 1;
			_spi->rxlen = len + 1;
			SPI_Start_IRQ_HWNSS(_spi);
			return 0;
		case PORT_DONE:
			FIFO_GetOne(_spi->rxbuffer, &dev->CHIP_STATUS_REG);
			FIFO_GetMulti(_spi->rxbuffer, data, len);
			_spi->status = PORT_FREE;
			return 1;
		default:
			return 0;
	}
}

//internal function
uint8_t GetStatus(SPI_Connection_t *_spi, nRF24_dev *dev) {
	return WriteCommand(_spi, dev, RF24_NOP);
}

//CONFIG REG	-------------------------------------------
uint8_t GetConfig(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *cfg) {
	return ReadShortComm(_spi, dev, CONFIG | R_REGISTER, cfg);
}
uint8_t SetConfig(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t cfg) {
	return WriteShortCommand(_spi, dev, CONFIG | W_REGISTER, cfg);
}

//EN_AA REG	enable autoask for select channels
uint8_t SetAutoAck(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t en_aa) {
	return WriteShortCommand(_spi, dev, EN_AA | W_REGISTER, en_aa);
}

//EN_RXADDR REG включает/выкдючает каналы приёмника
uint8_t SetActiveLines(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t pipes) {
	return WriteShortCommand(_spi, dev, EN_RXADDR | W_REGISTER, pipes);
}

//-------------------------------------------------------------
//work set addr size 3bytes = 0x01, 4bytes = 0x10, 5bytes = 0x11
uint8_t SetAddrWidth(SPI_Connection_t *_spi, nRF24_dev *dev, AddrWidth_t a_width) {
	return WriteShortCommand(_spi, dev, SETUP_AW | W_REGISTER, WidthCheck(a_width));
}

//задает число попыток и задержку между попытками доставки максимум 15*15
uint8_t SetRetries(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t delay, uint8_t count) {
	return WriteShortCommand(_spi, dev, SETUP_RETR | W_REGISTER, ((delay << 4) & 0xF0) | (count & 0x0F));
}

//----------------------------------------------------------
uint8_t SetChannel(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t channel) { //work
	return WriteShortCommand(_spi, dev, RF_CH | W_REGISTER, CheckChannel(channel));
}

//---------------------------------------------------------------
uint8_t SetRF(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t mask) {
	return WriteShortCommand(_spi, dev, RF_SETUP | W_REGISTER, mask | 0x01);
}

uint8_t CheckFLAG(nRF24_dev *dev, uint8_t flag) {
	return dev->CHIP_STATUS_REG & flag;
}

Pipes_t CheckPipe(nRF24_dev *dev) {
	return ((~(RX_DR | TX_DS | MAX_RT )) & dev->CHIP_STATUS_REG) > 1;
}

uint8_t ClearIRQ(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t irq) {//write singlebyte command and read hw status
	return WriteShortCommand(_spi, dev, STATUS | W_REGISTER, dev->CHIP_STATUS_REG | irq);
}

//задает адрес передатчика
uint8_t SetAddrTX(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *addr, uint8_t width) {
	return WriteLongComm(_spi, dev, TX_ADDR | W_REGISTER, addr, WidthCheck(width));
}
//задает адрес приемника
uint8_t SetAddrRX(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *addr, uint8_t width, Pipes_t pipe) {
	return WriteLongComm(_spi, dev, (RX_ADDR_P0 + pipe) | W_REGISTER, addr, WidthCheck(width));
}
//задает размер данных при обмене, 1-32 байта
uint8_t SetDataSize(SPI_Connection_t *_spi, nRF24_dev *dev, Pipes_t line, uint8_t size) {
	return WriteShortCommand(_spi, dev, (RX_PW_P0 + line) | W_REGISTER, LenghtCheck(size));
}
//-----------------------------------------------------------
uint8_t GetStatusFIFO(SPI_Connection_t *_spi, nRF24_dev *dev) {
	return ReadShortComm(_spi, dev, FIFO_STATUS | R_REGISTER, &dev->FIFO_status);
}
//----------------------------------------------------------------------------------
uint8_t SetDynamicDataLen(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t mask) {
	return WriteShortCommand(_spi, dev, DYNPD | W_REGISTER, mask & EN_ALL_PIPES_MASK);
}
//----------------------------------------------------------------------------------
uint8_t GetFeature(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *feature) {
	return ReadShortComm(_spi, dev, FEATURE | R_REGISTER, feature);
}
uint8_t SetFeature(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t feature) {
	return WriteShortCommand(_spi, dev, FEATURE | W_REGISTER, feature & 0x03);
}
//------------------------------------------------------------------
uint8_t ActivateButt(SPI_Connection_t *_spi, nRF24_dev *dev) {
	return WriteShortCommand(_spi, dev, ACTIVATE, MagicVal);
}

//====	COMMAND FUNCTION		======================
/* read payload data from FIFO and read hw status in first byte
	ret value 0 - processing, 1 - complite
*/
//TODO
uint8_t ReadPayloadFromChip(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data, uint8_t len) {
	return ReadLongComm(_spi, dev, R_RX_PAYLOAD, data, LenghtCheck(len));
}

/* write payload to chip W_TX_PAYLOAD and read hw status in first byte
ret value 0 - processing, 1 - complite
*/
uint8_t WritePayloadToChip(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data, uint8_t len) {
	return WriteLongComm(_spi, dev, W_TX_PAYLOAD, data, LenghtCheck(len));
}

//write command byte FLUSH_TX and read hw status clear TX buffer in chip
uint8_t FlushTX(SPI_Connection_t *_spi, nRF24_dev *dev) {
	return WriteCommand(_spi, dev, FLUSH_TX);
}

//write one command byte FLUSH_RX and read hw status clear RX buffer in chip
uint8_t FlushRX(SPI_Connection_t *_spi, nRF24_dev *dev) {
	return WriteCommand(_spi, dev, FLUSH_RX);
}

//write one command byte REUSE_TX_PL and read hw status
//repeat send last data in TX buffer
uint8_t RepeatLastTransfer(SPI_Connection_t *_spi, nRF24_dev *dev) {
	return WriteCommand(_spi, dev, REUSE_TX_PL);
}

//-----------------------------------------------------
//write one command byte R_RX_PL_WID and read hw status
//read RX size data, if > 32 need clear RX buffer
uint8_t ReadReceiveSize(SPI_Connection_t *_spi, nRF24_dev *dev) {
	return ReadShortComm(_spi, dev, R_RX_PL_WID, &dev->rxSize);
}
//-----------------------------------------------------
uint8_t AddDataForAsk(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data, uint8_t len, Pipes_t pipe) {
	return WriteLongComm(_spi, dev, W_ACK_PAYLOAD | pipe, data, LenghtCheck(len));
}
//-----------------------------------------------------
uint8_t WritePayloadNOASK(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data, uint8_t len) {
	return WriteLongComm(_spi, dev, W_TX_PAYLOAD_NOACK, data, LenghtCheck(len));
}

//main radio processing functions	=====================================================
/* read CONFIG register
 * write mode TX to register
 * set LOW CE pin
 * */
uint8_t SwitchModeTX(SPI_Connection_t *_spi, nRF24_dev *dev){
	uint8_t cfg;
	switch (dev->internalFunctionStep) {
	case 0:
		if (GetConfig(_spi, dev, &cfg)) {
			FIFO_PutOne(_spi->rxbuffer, SET_CONFIG_MODE_TX(cfg));
			dev->internalFunctionStep = 1;
		}
		break;
	case 1:
		if (SetConfig(_spi, dev, cfg)) {
			LL_GPIO_ResetOutputPin(dev->CE_Port, dev->CE_Pin);
			dev->internalFunctionStep = 0;
			return 1;
		}
		break;
	default:
		dev->internalFunctionStep = 0;
		break;
	}
	return 0;
}

/* read CONFIG register
 * write mode RX to register
 * set HIGH CE pin
 * */
uint8_t SwitchModeRX(SPI_Connection_t *_spi, nRF24_dev *dev){
	uint8_t cfg;
	switch(dev->internalFunctionStep) {
	case 0:
		if (GetConfig(_spi, dev, &cfg)) {
			FIFO_PutOne(_spi->rxbuffer, SET_CONFIG_MODE_RX(cfg));
			dev->internalFunctionStep = 1;
		}
		break;
	case 1:
		if (SetConfig(_spi, dev, cfg)) {
			LL_GPIO_SetOutputPin(dev->CE_Port, dev->CE_Pin);
			dev->internalFunctionStep = 0;
			return 1;
		}
		break;
	default:
		dev->internalFunctionStep = 0;
		break;
	}
	return 0;
}


/* nRF24 init function
 *
 * */
uint8_t RF24_Init (SPI_Connection_t *_spi, nRF24_dev *dev) {
	uint8_t feature;
	switch (dev->internalFunctionStep) {
		case 0://read status and check connection
			if (GetStatus(_spi, dev)) {dev->internalFunctionStep+=1;}
			break;
		case 1://записали число попыток 5 ожидание 15
			if (SetRetries(_spi, dev, 5, 15)) {dev->internalFunctionStep+=1;}
			break;
		case 2: //запись ширины адреса 5 байт
			if (SetAddrWidth(_spi, dev, 5))  {dev->internalFunctionStep+=1;}
			break;
		case 3://запись радиоканала 76
			if (SetChannel(_spi, dev, 76)) {dev->internalFunctionStep+=1;}
			break;
		case 4:
			if (SetAddrTX(_spi, dev, dev->pipeTx, 5)) { dev->internalFunctionStep+=1; }
			break;
		case 5:
			if (SetAddrRX(_spi, dev, dev->pipeRx0, 5, 0)) { dev->internalFunctionStep+=1; }
			break;
		case 6:
			if (SetAddrRX(_spi, dev, dev->pipeRx1, 5, 1)) { dev->internalFunctionStep+=1; }
			break;
		case 7:
			if (SetAddrRX(_spi, dev, dev->pipeRx2, 5, 2)) { dev->internalFunctionStep+=1; }
			break;
		case 8:
			if (SetAddrRX(_spi, dev, dev->pipeRx3, 5, 3)) { dev->internalFunctionStep+=1; }
			break;
		case 9:
			if (SetAddrRX(_spi, dev, dev->pipeRx4, 5, 4)) { dev->internalFunctionStep+=1; }
			break;
		case 10:
			if (SetAddrRX(_spi, dev, dev->pipeRx5, 5, 5)) { dev->internalFunctionStep+=1; }
			break;
		case 11://запись мощности и скорости 1Mbs 0dB и странный бит 0 (типа включает LNA)
			if (SetRF(_spi, dev, SET_RF_REG(0, 0, RF_2Mbps, RF_PWR_m12db, 1))) {dev->internalFunctionStep+=1;}
			break;
		case 12://чтение features
			if (GetFeature(_spi, dev, &feature)) {dev->internalFunctionStep+=1;}
			break;
		case 13: //недокументированная команда
			if (ActivateButt(_spi, dev)) {dev->internalFunctionStep+=1;}
			break;
		case 14:	//чтение features снова
			if (GetFeature(_spi, dev, &feature)) {dev->internalFunctionStep+=1;}
			break;
		case 15://запись динамического пакета, выключено
			if (SetDynamicDataLen(_spi, dev, EN_ALL_PIPES_MASK)) {dev->internalFunctionStep+=1;}
			break;
		case 16://запись подтверждения каналов все включили
			if (SetAutoAck(_spi, dev, EN_ALL_PIPES_MASK)) {dev->internalFunctionStep+=1;}
			break;
		case 17: //задаем размер пакета
			if (SetDataSize(_spi, dev, 0, 32)) {dev->internalFunctionStep+=1;}
			break;
		case 18: //задаем размер пакета
			if (SetDataSize(_spi, dev, 1, 32)) {dev->internalFunctionStep+=1;}
			break;
		case 19: //задаем размер пакета
			if (SetDataSize(_spi, dev, 2, 32)) {dev->internalFunctionStep+=1;}
			break;
		case 20: //задаем размер пакета
			if (SetDataSize(_spi, dev, 3, 32)) {dev->internalFunctionStep+=1;}
			break;
		case 21: //задаем размер пакета
			if (SetDataSize(_spi, dev, 4, 32)) {dev->internalFunctionStep+=1;}
			break;
		case 22: //задаем размер пакета
			if (SetDataSize(_spi, dev, 5, 32)) {dev->internalFunctionStep+=1;}
			break;
		case 23://запись конфига питание включили
			if (SetConfig(_spi, dev, SET_CONFIG(1, 1, 1, crc_2byte, 1, 0)))   {//0x0e
				LL_mDelay(5);//здесь пауза ждем когда включится модуль
				dev->internalFunctionStep+=1;
			}
			break;
		case 24://запись настройки включенных каналов
			if (SetActiveLines(_spi, dev, EN_ALL_PIPES_MASK)) { dev->internalFunctionStep+=1; }
			break;
		case 25://clear buffer RX
			if (FlushRX(_spi, dev)) {dev->internalFunctionStep+=1;}
			break;
		case 26://clear buffer TX
			if (FlushTX(_spi, dev)) {dev->internalFunctionStep+=2;}
			break;
		case 27://запись для стирания всех флагов прерываний
			if (ClearIRQ(_spi, dev, RX_DR | TX_DS | MAX_RT)) {dev->internalFunctionStep+=1;}
			break;
		case 28://запись настройки включенных каналов
			if (SetActiveLines(_spi, dev, 0x03)) {
				dev->internalFunctionStep = 0;
				return 1;
			}
			break;
		default:
			dev->internalFunctionStep = 0;
			break;
	}
	return 0;
}

uint8_t RF24_Processing(SPI_Connection_t *_spi, nRF24_dev *dev) {
	return 0;
}

uint8_t RF24_SendData (SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data, uint8_t len) {
	return 0;
}
//----------------------------------------------------------------------------------
uint8_t RF24_ReceiveData (SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data) {
	return 0;
}
