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


typedef enum Pipes {
	pipe0 = 0x00,
	pipe1 = 0x01,
	pipe2 = 0x02,
	pipe3 = 0x03,
	pipe4 = 0x04,
	pipe5 = 0x05,
	none 	= 0x06,
	empty = 0x07
} Pipes_t;

//common data struct for module
typedef struct nRF24_dev_t {
	GPIO_TypeDef *CE_Port;	//CE port for CE tx|rx pinout
	uint32_t CE_Pin;				//CE pin for CE tx|rx pinout
	uint8_t internalFunctionStep;			//step for internal function
	uint8_t state;		//step for procedure processing
	DeviceStatus_t status;	//device status
	uint8_t CHIP_STATUS_REG;	//chip STATUS reg
	uint8_t FIFO_status;	//FIFO status reg
	Pipes_t pipeNumber;//line number receive
	uint8_t rxSize;		//receive data size
	uint32_t txCount;	//transmit count
	uint32_t rxCount;	//receive count
	uint32_t failCount;//error transmit count
	uint8_t pipeTx[5];
	uint8_t pipeRx0[5];
	uint8_t pipeRx1[5];
	uint8_t pipeRx2[5];
	uint8_t pipeRx3[5];
	uint8_t pipeRx4[5];
	uint8_t pipeRx5[5];
} nRF24_dev;

//========================		HI LEVEL		================================
uint8_t ReadPayloadFromChip(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data, uint8_t len);

uint8_t WritePayloadToChip(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data, uint8_t len);

uint8_t FlushTX(SPI_Connection_t *_spi, nRF24_dev *dev);

uint8_t FlushRX(SPI_Connection_t *_spi, nRF24_dev *dev);

uint8_t RepeatLastTransfer(SPI_Connection_t *_spi, nRF24_dev *dev);

uint8_t ReadReceiveSize(SPI_Connection_t *_spi, nRF24_dev *dev);

uint8_t AddDataForAsk(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data, uint8_t len, Pipes_t line);

uint8_t WritePayloadNOASK(SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data, uint8_t len);

uint8_t SwitchModeTX(SPI_Connection_t *_spi, nRF24_dev *dev);

uint8_t SwitchModeRX(SPI_Connection_t *_spi, nRF24_dev *dev);

uint8_t RF24_Init (SPI_Connection_t *_spi, nRF24_dev *dev);

uint8_t RF24_Processing(SPI_Connection_t *_spi, nRF24_dev *dev);

uint8_t RF24_SendData (SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data, uint8_t len);

uint8_t RF24_ReceiveData (SPI_Connection_t *_spi, nRF24_dev *dev, uint8_t *data);
//====================================================================================================
#ifdef __cplusplus
}
#endif
#endif /* _NRF24L01_H_ */

