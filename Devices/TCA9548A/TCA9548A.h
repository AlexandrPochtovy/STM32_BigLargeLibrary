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
 
   Created on: Aug 1, 2022
 ********************************************************************************/

#ifndef TCA9548A_H_
#define TCA9548A_H_

#include "Peripherals/I2C/MyI2C.h"

enum TCA9548A_ADDRESS {
	TCA9548A_ADDR = 0x70//Assumes ALT address pin low
};

typedef enum TCA9548A_channel {
	channel_0 = 0x01,
	channel_1 = 0x02,
	channel_2 = 0x04,
	channel_3 = 0x08,
	channel_4 = 0x10,
	channel_5 = 0x20,
	channel_6 = 0x40,
	channel_7 = 0x80
} TCA9548A_ch_t;

//common data struct for sensor
typedef struct TCA9548A {
	uint8_t addr;
	uint8_t step;
	DeviceStatus_t status;
	uint8_t port;
} TCA9548A_t;

uint8_t TCA9548A_Init(I2C_IRQ_Conn_t *_i2c, TCA9548A_t *dev);
uint8_t TCA9548A_OnChannel(I2C_IRQ_Conn_t *_i2c, TCA9548A_t *dev, TCA9548A_ch_t ch);
uint8_t TCA9548A_OffChannel(I2C_IRQ_Conn_t *_i2c, TCA9548A_t *dev, TCA9548A_ch_t ch);

#endif /* TCA9548A_H_ */
