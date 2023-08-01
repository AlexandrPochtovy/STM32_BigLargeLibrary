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

 * 	nRF24L01.h
 *  Created on: 23 feb 2022
 */

#ifndef _NRF24L01_H_
#define _NRF24L01_H_
#ifdef __cplusplus
extern "C" {
#endif


#include "string.h"
#include "Peripherals/SPI/MySPI.h"

#include "RF24Register.h"

typedef struct nRF24_MainConfig {
	CONFIG_SET_t config;	//value for CONFIG reg
	uint8_t enaa;			//value for EN_AA reg
	uint8_t enrxaddr;		//value for EN_RXADDR reg
	AddrWidth_t addrWidth;	//value for SETUP_AW reg
	uint8_t sendRetr;		//value for SETUP_RETR reg
	uint8_t sendTimeout;	//value for SETUP_RETR reg
	uint8_t channel;		//value for RF_CH reg
	RF_SET_t rf;			//value for RF_SETUP reg
	uint8_t rx_addr[5];		//value for RX_ADDR_P0 & TX_ADDR regs 
	uint8_t datasize;		//value for RX_PW_P0..RX_PW_P5 regs 
	uint8_t dynpd;			//value for DYNPD reg
	FEATURE_Set_t feat;		//value for FEATURE reg
} nRF24_MainConfig_t;

//common data struct for module
typedef struct nRF24_dev_t {
	GPIO_TypeDef *CE_Port;	//CE port for CE tx|rx pinout
	uint32_t CE_Pin;				//CE pin for CE tx|rx pinout
	uint8_t step;			//step for internal function
	uint8_t state;		//step for procedure processing
	DeviceStatus_t status;	//device status
	uint8_t Chip_st;	//chip STATUS reg
	uint8_t FIFO_st;	//FIFO status reg
	Line_t lineNumber;//line number receive
	uint8_t rxSize;		//receive data size
	uint32_t txCount;	//transmit count
	uint32_t rxCount;	//receive count
	uint32_t failCount;//error transmit count
} nRF24_dev;
//========================		LOW LEVEL		===============================
uint8_t WriteCommand(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t cmd);
uint8_t WriteShortCommand(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t cmd, uint8_t value);

uint8_t WriteLongComm(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t cmd, uint8_t *data, uint8_t len);

uint8_t ReadShortComm(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t cmd);

uint8_t ReadLongComm(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t cmd, uint8_t *data, uint8_t len);
//========================		MIDDLE LEVEL	================================
uint8_t GetStatus(SPI_Connection_t *_spi, nRF24_dev *dev);

uint8_t GetConfig(SPI_Connection_t *_spi, nRF24_dev *dev);

uint8_t SetConfig(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t mask);

uint8_t SetAutoAck(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t mask);

uint8_t SetActiveLines(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t mask);

uint8_t SetAddrWidth(SPI_Connection_t *_spi, nRF24_dev *dev, AddrWidth_t a_width);

uint8_t SetRetries(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t delay, uint8_t count);

uint8_t SetChannel(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t channel);

uint8_t SetRF(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t mask);

uint8_t ClearIRQ(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t irq);

uint8_t SetAddrTX(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *addr, uint8_t width);

uint8_t SetAddrRX(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *addr, uint8_t width, Line_t line);

uint8_t SetDataSize(SPI_Connection_t *_spi, nRF24_dev *dev, Line_t line, uint8_t size);

uint8_t GetStatusFIFO(SPI_Connection_t *_spi, nRF24_dev *dev);

uint8_t SetDynamicDataLen(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t mask);

uint8_t GetFeature(SPI_Connection_t *_spi, nRF24_dev *dev);

uint8_t SetFeature(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t mask);

uint8_t ActivateButt(SPI_Connection_t *_spi, nRF24_dev *dev);


//========================		HI LEVEL		================================
uint8_t ReadPayload(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data, uint8_t len);

uint8_t WritePayload(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data, uint8_t len);

uint8_t FlushTX(SPI_Connection_t *_spi, nRF24_dev *dev);

uint8_t FlushRX(SPI_Connection_t *_spi, nRF24_dev *dev);

uint8_t RepeatLastTransfer(SPI_Connection_t *_spi, nRF24_dev *dev);

uint8_t ReadReceiveSize(SPI_Connection_t *_spi, nRF24_dev *dev);

uint8_t AddDataForAsk(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data, uint8_t len, Line_t line);

uint8_t WritePayloadNOASK(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data, uint8_t len);

uint8_t SwitchModeTX(SPI_Connection_t *_spi, nRF24_dev *dev);

uint8_t SwitchModeRX(SPI_Connection_t *_spi, nRF24_dev *dev);

uint8_t RF24_Init (SPI_Connection_t *_spi, nRF24_dev *dev, nRF24_MainConfig_t *cfg);

uint8_t RF24_Processing(SPI_Connection_t *_spi, nRF24_dev *dev);

uint8_t RF24_SendData (SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data, uint8_t len);

uint8_t RF24_ReceiveData (SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data);
//====================================================================================================
#ifdef __cplusplus
}
#endif
#endif /* _NRF24L01_H_ */

