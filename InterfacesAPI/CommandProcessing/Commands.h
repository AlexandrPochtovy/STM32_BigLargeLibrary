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

 * Commands.h
 * Created on: Nov 29, 2023
 ********************************************************************************/
#include <stdint.h>

 /*	Commands
		HELLO((byte) 0x00),
		READ_SENSOR((byte) 0x01),
		READ_DRIVE_DATA((byte) 0x02),
		READ_OTHER_DATA((byte) 0x03),
		READ_SETTINGS((byte) 0x04),
		WRITE_SETTINGS((byte) 0x05);
 */

#define COMMAND_START_BYTE (uint8_t)0xAA
#define COMMAND_ACCEPT_BYTE (uint8_t)0x55

typedef enum Commands {
	COMMAND_HELLO = (uint8_t)0x00,
	COMMAND_READ_SENSOR = (uint8_t)0x01,
	READ_DRIVE_DATA = (uint8_t)0x02,
	READ_OTHER_DATA = (uint8_t)0x03,
	READ_SETTINGS = (uint8_t)0x04,
	WRITE_SETTINGS = (uint8_t)0x05,
	COMMAND_NOOP = (uint8_t)0xFF
	} Commands_t;