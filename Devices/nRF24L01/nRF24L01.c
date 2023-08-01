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

#define EN_AA_MASK 			0x3F
#define ACTIVE_LINE_MASK 	0x3F
#define DYN_MASK 			0x3F

//====	SOME INTERNAL FUNCTION	=================
static inline uint8_t WidthCheck (uint8_t width) {
	return ((width >=3) && (width <=5)) ? width : 5;
}

static inline uint8_t LenghtCheck (uint8_t size) {
	return (size > 0) && (size <=32) ? size : 32;
}

static inline Line_t GetLineNumber (nRF24_dev *dev) {
	return ((dev->Chip_st >> 1) & empty);
}

static inline uint8_t CheckChannel (uint8_t channel) {
	return ((channel & 0x7F) <= 125) ? channel : 125;
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
			FIFO_GetOne(_spi->rxbuffer, &dev->Chip_st);
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
			FIFO_GetOne(_spi->rxbuffer, &dev->Chip_st);
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
			FIFO_GetOne(_spi->rxbuffer, &dev->Chip_st);
			_spi->status = PORT_FREE;
			return 1;
		default:
 			return 0;
	}
}
//read one byte command and read hw status
uint8_t ReadShortComm(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t cmd) {
	switch (_spi->status) {
		case PORT_FREE:
			FIFO_PutOne(_spi->txbuffer, cmd);
			_spi->txlen = 1;
			_spi->rxlen = 2;
			SPI_Start_IRQ_HWNSS(_spi);
			return 0;
		case PORT_DONE:
			FIFO_GetOne(_spi->rxbuffer, &dev->Chip_st);
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
			FIFO_PutOne(_spi->txbuffer, cmd);//_spi->reg = cmd;//send command
			_spi->txlen = 1;
			_spi->rxlen = len + 1;
			SPI_Start_IRQ_HWNSS(_spi);
			return 0;
		case PORT_DONE:
			FIFO_GetOne(_spi->rxbuffer, &dev->Chip_st);//memcpy(&dev->Chip_st, &dev->rxp[0], 1);
			FIFO_GetMulti(_spi->rxbuffer, data, len);//memcpy(data, &dev->rxp[1], len);
			_spi->status = PORT_FREE;
			return 1;
		default:
			return 0;
	}
}
//====	COMMAND FUNCTION		======================
/* read payload data from FIFO and read hw status in first byte
	ret value 0 - processing, 1 - complite
*/
uint8_t ReadPayload(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data, uint8_t len) {
	return ReadLongComm(_spi, dev, R_RX_PAYLOAD | GetLineNumber(dev), data, LenghtCheck(len));
}

/* write payload to chip W_TX_PAYLOAD and read hw status in first byte
ret value 0 - processing, 1 - complite
*/
uint8_t WritePayload(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data, uint8_t len) {
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
	if (ReadShortComm(_spi, dev, R_RX_PL_WID)) {
		FIFO_GetOne(_spi->rxbuffer, &dev->rxSize);
		return 1;
	}
	return 0;
}
//-----------------------------------------------------
uint8_t AddDataForAsk(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data, uint8_t len, Line_t line) {
	return WriteLongComm(_spi, dev, W_ACK_PAYLOAD | line, data, LenghtCheck(len));
}
//-----------------------------------------------------
uint8_t WritePayloadNOASK(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data, uint8_t len) {
	return WriteLongComm(_spi, dev, W_TX_PAYLOAD_NOACK, data, LenghtCheck(len));
}
//=======================================================================================================
//		REGISTER FUNCTIONS
//=======================================================================================================
uint8_t GetStatus(SPI_Connection_t *_spi, nRF24_dev *dev) {
	return WriteCommand(_spi, dev, RF24_NOP);
}
//	CONFIG	-------------------------------------------
uint8_t GetConfig(SPI_Connection_t *_spi, nRF24_dev *dev) {
	return ReadShortComm(_spi, dev, CONFIG | R_REGISTER);
}
uint8_t SetConfig(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t cfg) {
	return WriteShortCommand(_spi, dev, CONFIG | W_REGISTER, cfg);
}

//включает функцию подтверждения получения данных на линиях
uint8_t SetAutoAck(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t mask) {//work
	return WriteShortCommand(_spi, dev, EN_AA | W_REGISTER, mask & EN_AA_MASK);
}
//---------------------------------------------------------------
//включает/выкдючает каналы приёмника по маске
uint8_t SetActiveLines(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t mask) {
	return WriteShortCommand(_spi, dev, EN_RXADDR | W_REGISTER, mask & ACTIVE_LINE_MASK);
}
//-------------------------------------------------------------
//work set addr size 3bytes = 0x01, 4bytes = 0x10, 5bytes = 0x11
uint8_t SetAddrWidth(SPI_Connection_t *_spi, nRF24_dev *dev, AddrWidth_t a_width) {
	return WriteShortCommand(_spi, dev, SETUP_AW | W_REGISTER, a_width);
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
//----------------------------------------------------------------
uint8_t ClearIRQ(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t irq) {//write singlebyte command and read hw status
	return WriteShortCommand(_spi, dev, STATUS | W_REGISTER, dev->Chip_st | irq);
}
//задает адрес передатчика
uint8_t SetAddrTX(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *addr, uint8_t width) {
	return WriteLongComm(_spi, dev, TX_ADDR | W_REGISTER, addr, WidthCheck(width));
}
//задает адрес приемника
uint8_t SetAddrRX(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *addr, uint8_t width, Line_t line) {
	return WriteLongComm(_spi, dev, (RX_ADDR_P0 + line) | W_REGISTER, addr, WidthCheck(width));
}
//задает размер данных при обмене, 1-32 байта
uint8_t SetDataSize(SPI_Connection_t *_spi, nRF24_dev *dev, Line_t line, uint8_t size) {
	return WriteShortCommand(_spi, dev, (RX_PW_P0 + line) | W_REGISTER, LenghtCheck(size));
}
//-----------------------------------------------------------
uint8_t GetStatusFIFO(SPI_Connection_t *_spi, nRF24_dev *dev) {
	if (ReadShortComm(_spi, dev, FIFO_STATUS | R_REGISTER)){
		FIFO_GetOne(_spi->rxbuffer, &dev->FIFO_st);
		return 1;
	} 
	return 0;
}
//----------------------------------------------------------------------------------
uint8_t SetDynamicDataLen(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t mask) {
	return WriteShortCommand(_spi, dev, DYNPD | W_REGISTER, mask & DYN_MASK, 1);
}
//----------------------------------------------------------------------------------
uint8_t GetFeature(SPI_Connection_t *_spi, nRF24_dev *dev) {
	return ReadShortComm(_spi, dev, FEATURE | R_REGISTER);
}
uint8_t SetFeature(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t mask) {
	return WriteShortCommand(_spi, dev, FEATURE | W_REGISTER, mask & 0x03);
}
//------------------------------------------------------------------
uint8_t ActivateButt(SPI_Connection_t *_spi, nRF24_dev *dev) {
	const uint8_t val = 0x73;//ВЗЯТО ИЗ ПРИМЕРА
	return WriteShortCommand(_spi, dev, ACTIVATE, val);
}
//==================================================================================
uint8_t SwitchModeTX(SPI_Connection_t *_spi, nRF24_dev *dev){
	//прочитать конфиг
	//записать в конфиг бит режима приемника
	//установить в 0 ножку CE
	switch (dev->step) {
	case 0:
		if (GetConfig(_spi, dev)) {
			uint8_t cfg;
			GetOne(&_spi->rxbuffer, &cfg);
			PutOne(&_spi->rxbuffer, cfg & 0xFE);
			dev->step = 1;
		}
		return 0;
	case 1:
		if (SetConfig(_spi, dev, cfg)) {
			LL_GPIO_ResetOutputPin(dev->CE_Port, dev->CE_Pin);
			dev->step = 0;
			return 1;
		}
		return 0;
	default:
		dev->step = 0;
		return 0;
	}
}
//----------------------------------------------------------------------------------
uint8_t SwitchModeRX(SPI_Connection_t *_spi, nRF24_dev *dev){
	//прочитать конфиг
	//записать в конфиг бит режима передатчика
	//установить в 1 ножку CEstate
	switch(dev->step) {
	case 0:
		if (GetConfig(_spi, dev)) {
			uint8_t cfg;
			GetOne(&_spi->rxbuffer, &cfg);
			PutOne(&_spi->rxbuffer, cfg | 0x01);
			dev->step = 1;
		}
		return 0;
	case 1:
		if (SetConfig(_spi, dev, cfg)) {
			LL_GPIO_SetOutputPin(dev->CE_Port, dev->CE_Pin);
			dev->step = 0;
			return 1;
		}
		return 0;
	default:
		dev->step = 0;
		return 0;
	}
}
//============================================================================
uint8_t RF24_Init (SPI_Connection_t *_spi, nRF24_dev *dev, nRF24_MainConfig_t *cfg) {
	uint8_t addr[2][5] = { {0xCE, 0xCF, 0xAE, 0xAF, 0xD1}, {0xCE, 0xCF, 0xAE, 0xAF, 0xD0} };
	switch (dev->step) {
		case 0://read status and check connection
			if (GetStatus(_spi, dev)) {dev->step+=1;}
			break;
		case 1://записали число попыток 5 ожидание 15
			if (SetRetries(_spi, dev, cfg->sendRetr, cfg->sendTimeout)) {dev->step+=1;}
			break;
		case 2: //запись ширины адреса 5 байт
			if (SetAddrWidth(_spi, dev, cfg->addrWidth))  {dev->step+=1;}
			break;
		case 3://запись радиоканала 76
			if (SetChannel(_spi, dev, cfg->channel)) {dev->step+=1;}
			break;
		case 4:
			if (SetAddrRX(_spi, dev, addr[1], 5, 0)) { dev->step+=1; }
			break;
		case 5:
			if (SetAddrTX(_spi, dev, addr[1], 5)) { dev->step+=1; }
			break;
		case 6:
			if (SetAddrRX(_spi, dev, addr[0], 5, 1)) { dev->step+=1; }
			break;
		case 7://запись мощности и скорости 1Mbs 0dB и странный бит 0 (типа включает LNA)
			{
				uint8_t value = cfg->rf.CONT_WAVE << 7 	|
								cfg->rf.PLL_LOCK << 4	|
								cfg->rf.radiance 		|
								cfg->rf.speed 			|
								0x01;//0x07
				if (SetRF(_spi, dev, value)) {dev->step+=1;}
			}
			break;
		case 8://чтение features
			if (GetFeature(_spi, dev)) {dev->step+=1;}
			break;
		case 9: //недокументированная команда
			if (ActivateButt(_spi, dev)) {dev->step+=1;}
			break;
		case 10:	//чтение features снова
			if (GetFeature(_spi, dev)) {dev->step+=1;}
			break;
		case 11://запись динамического пакета, выключено
			if (SetDynamicDataLen(_spi, dev, cfg->dynpd)) {dev->step+=1;}
			break;
		case 12://запись подтверждения каналов все включили
			if (SetAutoAck(_spi, dev, cfg->enaa)) {dev->step+=1;}
			break;
		case 13: //задаем размер пакета
			if (SetDataSize(_spi, dev, 0, cfg->datasize)) {dev->step+=1;}
			break;
		case 14: //задаем размер пакета
			if (SetDataSize(_spi, dev, 1, cfg->datasize)) {dev->step+=1;}
			break;
		case 15: //задаем размер пакета
			if (SetDataSize(_spi, dev, 2, cfg->datasize)) {dev->step+=1;}
			break;
		case 16: //задаем размер пакета
			if (SetDataSize(_spi, dev, 3, cfg->datasize)) {dev->step+=1;}
			break;
		case 17: //задаем размер пакета
			if (SetDataSize(_spi, dev, 4, cfg->datasize)) {dev->step+=1;}
			break;
		case 18: //задаем размер пакета
			if (SetDataSize(_spi, dev, 5, cfg->datasize)) {dev->step+=1;}
			break;
		case 19://запись конфига питание включили
			{
				uint8_t value = cfg->config.RX_DR_IRQ << 6 	|//6 bit pos
								cfg->config.TX_DS_IRQ << 5 	|//5 bit pos
								cfg->config.MAX_TX_IRQ << 4 |//4 bit pos
								cfg->config.crc 			|//3 & 2 bit pos
								cfg->config.power << 1 		|//1 bit pos
								cfg->config.mode;			//0 bit pos
				if (SetConfig(_spi, dev, value))   {//0x0e
					LL_mDelay(5);//здесь пауза ждем когда включится модуль
					dev->step+=1;
				}
			}
			break;
		case 20://запись настройки включенных каналов
			if (SetActiveLines(_spi, dev, cfg->enrxaddr)) { dev->step+=1; }
			break;
		case 21://clear buffer RX
			if (FlushRX(_spi, dev)) {dev->step+=1;}
			break;
		case 22://clear buffer TX
			if (FlushTX(_spi, dev)) {dev->step+=2;}
			break;
		case 23://запись для стирания всех флагов прерываний
			if (ClearIRQ(_spi, dev, RX_DR | TX_DS | MAX_RT)) {dev->step+=1;}
			break;
		case 24://запись настройки включенных каналов
			if (SetActiveLines(_spi, dev, 0x03)) {
				dev->step = 0;
				return 1;
			}
			break;
		default:
			dev->step = 0;
			break;
	}
	return 0;
}

uint8_t RF24_Processing(SPI_Connection_t *_spi, nRF24_dev *dev) {
/*проверяем флаги и смотрим
если флаг активации передачи - переключаемся на передачу, передаем, переключаемся на прием
если флаг прерывания - проверяем какие прерывания пришли, считываем данные, сбрасываем прерывания*/
	switch (dev->state) {
		case 0://check flags
			if (ptt) { dev->state = 10; }
			else if (irqFlag) { dev->state = 20; }
			else { return; }
			break;
		case 10://switch TX mode
			if (SwitchModeTX(_spi, dev)) {
				dev->state = 11;
			}
			break;
		case 11://TX send data
			if (RF24_SendData(_spi, dev, data, sendLen)) {
				dev->state = 12;
			}
			break;
		case 12://switch RX mode for listening
			if (SwitchModeRX(_spi, dev)) {
				dev->state = 0;
			}
			break;

		case 20://check status
			if (GetStatus(_spi, dev)) {
				if (dev->Chip_st & TX_DS) {
					++dev->txCount;
					dev->state =  21;
				}
				else if (dev->Chip_st & RX_DR) {
					dev->state = 22;
				}
				else if (dev->Chip_st & MAX_RT) { dev->state = 30; }
			}
			break;
		case 21:
			if (ClearIRQ(_spi, dev, TX_DS)) { dev->state = 20;} 
			break;	
		case 22:
			if (RF24_ReceiveData(_spi, dev, data)) {
				++dev->rxCount;
				dev->state = 23;
			}
			break;
		case 23:
			if (ClearIRQ(_spi, dev, RX_DR)) { dev->state = 20; }
			break;	
		/*case 25:
			if (RepeatLastTransfer(_spi, dev)) {dev->state = 26;}
			break;*/
		case 30://MAX_RT dana not send need retransmitt
			if (ClearIRQ(_spi, dev, MAX_RT)) {dev->state = 0;}
			break;
		default:
			dev->state = 0;
			break;
	}
}

uint8_t RF24_SendData (SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data, uint8_t len) {
	switch (dev->step) {
		case 0://читаем статус модуля и статус фифо
			if (GetStatusFIFO(_spi, dev)) {//смотрим на статус и проверяем что делать
				if (dev->FIFO_st & TX_FIFO_EMPTY) {
					dev->step = 2;
				}
				else {
					dev->step = 1;
				}
			}
			break;
		case 1:
			if (FlushTX(_spi, dev)) {
				dev->step = 2;
			}
			break;
		case 2:
			if (WritePayload(_spi, dev, data, len)) {
				LL_GPIO_SetOutputPin(dev->CE_Port, dev->CE_Pin);
				for (uint32_t i = 0; i < 60; i++) {
					__NOP();
				}
				LL_GPIO_ResetOutputPin(dev->CE_Port, dev->CE_Pin);
				dev->step = 0;
				return 1;
			}
			break;
		default:
			dev->step = 0;
			return 0;
	}
	return 0;
}
//----------------------------------------------------------------------------------
uint8_t RF24_ReceiveData (SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data) {
	static uint8_t datasize;
	switch (dev->step) {
		case 0://читать статус и статус буфера, затем смотреть что за прерывание пришло
			if (GetStatusFIFO(_spi, dev)) {//смотрим на статус и проверяем что делать
				if ((GetLineNumber(dev) != empty)){
					dev->step = 1;//go read
				}
				else {
					dev->step = 3;//flush & exit
				}
			}
			break;
		case 1://читаем размер принятого пакета и номер линии по которой пришло
			if (ReadReceiveSize(_spi, dev)) { 
				datasize = dev->rxp[1];
				dev->step = 2; }
			break;
		case 2://read data
			if (ReadPayload(_spi, dev, data, datasize)) {dev->step = 0;}
			break;
		case 3://clear IRQ
			if (ClearIRQ(_spi, dev, RX_DR | TX_DS | MAX_RT)) {
				dev->step = 0; return 1;
			}
			break;
		default:
			dev->step = 0;
			break;
	}
	return 0;
}
