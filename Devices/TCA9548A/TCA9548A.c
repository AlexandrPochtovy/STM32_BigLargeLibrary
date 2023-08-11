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


#include "TCA9548A.h"

static const uint8_t TCA9548A_PORT = 0x00;

uint8_t TCA9548A_Init(I2C_IRQ_Connection_t *_i2c, TCA9548A_t *dev) {
	dev->status = DEVICE_NOT_INIT;
	if (WriteOneRegByte(_i2c, dev->addr, TCA9548A_PORT, 0x00)) {
		dev->status = DEVICE_INIT;
		dev->step = 0;
		return 1;
	}
	return 0;
}

uint8_t TCA9548A_OnChannel(I2C_IRQ_Connection_t *_i2c, TCA9548A_t *dev, TCA9548A_ch_t ch) {
	switch (dev->step) {
		case 0://read port actual
			if (ReadOneRegByte(_i2c, dev->addr, TCA9548A_PORT, &dev->port)) {
				dev->step = 1;
			}
			break;
		case 1://apply channels
			if (WriteOneRegByte(_i2c, dev->addr, TCA9548A_PORT, dev->port | ch)) {
				dev->status = DEVICE_DONE;
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

uint8_t TCA9548A_OffChannel(I2C_IRQ_Connection_t *_i2c, TCA9548A_t *dev, TCA9548A_ch_t ch) {
	switch (dev->step) {
		case 0://read port actual
			if (ReadOneRegByte(_i2c, dev->addr, TCA9548A_PORT, &dev->port)) {
				dev->step = 1;
			}
			break;
		case 1://apply channels
			if (WriteOneRegByte(_i2c, dev->addr, TCA9548A_PORT, dev->port & ~ch)) {
				dev->status = DEVICE_DONE;
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
