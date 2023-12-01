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

#include "CommandProcessing.h"

static ConnectionMode_t step = BEGIN;
static uint8_t buffer[255];

void RequestProcessing(USART_FullDuplex_t *usart) {
	uint8_t len;
	union crc32check {
		uint32_t crc32;
		uint8_t value[4];
		} crcData;
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
		if (buffer[0] == COMMAND_START_BYTE) {
			crcData.crc32 = F4xx_HW_CRC32(CRC1, (uint32_t *)&buffer, len - 4);
			uint32_t crc = (uint32_t *)&buffer[len - 4];
			if (crc == crcData.crc32) {//crc valid
				buffer[0] = COMMAND_ACCEPT_BYTE;
				buffer[2] += 1;
				switch (buffer[1]) {
					case /* constant-expression */:
						/* code */
						break;

					default:
						break;
					}
				}
			else {//TODO add send "crc invalid"
				step = LISTENING;
				}
			}
		else {//TODO add send "wrong start byte"
			step = LISTENING;
			}
		}
	if (step == PACKET_CREATE) {

		}
	if (step == SENDING) {
		if (USART_Transmit(usart, &buffer, len)) {
			step = LISTENING;
			}
		}
	}