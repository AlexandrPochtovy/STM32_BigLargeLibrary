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

 * CommandProcessing.c
 * Created on: Nov 30, 2023
 ********************************************************************************/

 /*packet data format:
 byte[0]     synchro-byte
 byte[1]     command
 byte[2]     packet number
 byte[3]     packet length
 byte[4..n]  valid data, no bytes if HELLO command use
 byte[n-4]   CRC32
 */

#include "CommandProcessing.h"

static ConnectionMode_t step = BEGIN;
static uint8_t buffer[255];

uint8_t CheckCRC32(uint8_t* data, uint8_t len) {
	uint32_t crc32Calc = F4xx_HW_CRC32(CRC1, ( uint32_t* )&data, len - 4);
	uint32_t crcShift = ( uint32_t* )&buffer[len - 4];
	return crc32Calc == crcShift;
	}

uint8_t AddCRC32(uint8_t* data, uint8_t len) {//TODO check pointers!
	uint32_t crc32Calc = F4xx_HW_CRC32(CRC1, ( uint32_t* )&data, len - 4);
	data[len - 4] = crc32Calc;
	return 1;
	}

void RequestProcessing(USART_FullDuplex_t* usart) {
	uint8_t len;
	if (step == BEGIN) { //start communications
		USART_ProcessingEnable(usart);
		step = LISTENING;
		}
	if (step == LISTENING) { //usart port listening
		len = USART_Receive(usart, &buffer);
		if (len) {
			step = REQUEST_PROCESSING;
			}
		}
	if (step == REQUEST_PROCESSING) {
		if ((buffer[0] == COMMAND_START_BYTE) && (CheckCRC32(&buffer, len))) {
			switch (buffer[1]) {
				case COMMAND_HELLO:
					;
					break;
				case COMMAND_READ_SENSOR:
					;
					break;
				case READ_DRIVE_DATA:

					break;
				case READ_OTHER_DATA:

					break;
				case READ_SETTINGS:

					break;
				case WRITE_SETTINGS:

					break;
				case COMMAND_NOOP:

					break;
				default:
					break;
				}
			}
		else {//send NOOP if wrong start byte or bad crc32
			buffer[1] = COMMAND_NOOP;
			len = 4;
			}
		buffer[0] = COMMAND_ACCEPT_BYTE;
		buffer[2] += 1;
		buffer[3] = len;
		AddCRC32(&buffer, len);
		step = SENDING;
		}
	if (step == SENDING) {
		if (USART_Transmit(usart, &buffer, len)) {
			step = LISTENING;
			}
		}
	}