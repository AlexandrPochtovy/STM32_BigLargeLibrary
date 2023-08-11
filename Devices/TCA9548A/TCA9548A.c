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
	if (_i2c->status == PORT_FREE) {
		switch (dev->step) {
		case 0://set addr and clear port
			_i2c->addr = dev->addr;
			dev->status = DEVICE_NOT_INIT;
			FIFO_PutOne(_i2c->buffer, TCA9548A_PORT);
			FIFO_PutOne(_i2c->buffer, 0x00);
			_i2c->len = 1;
			_i2c->mode = I2C_MODE_WRITE;
			dev->step = 1;
			break;
		case 1://exit
			dev->status = DEVICE_INIT;
			dev->step = 0;
			return 1;
		default:
			dev->step = 0;
			return 0;
		}
		I2C_Start_IRQ(_i2c);
	}
	return 0;
}

uint8_t TCA9548A_OnChannel(I2C_IRQ_Connection_t *_i2c, TCA9548A_t *dev, TCA9548A_ch_t ch) {
	if (_i2c->status == PORT_FREE) {
		switch (dev->step) {
		case 0://read port actual
			_i2c->addr = dev->addr;
			FIFO_PutOne(_i2c->buffer, TCA9548A_PORT);
			_i2c->len = 1;
			_i2c->mode = I2C_MODE_READ;
			dev->step = 1;
			break;
		case 1://apply channels
			_i2c->addr = dev->addr;
			FIFO_GetOne(_i2c->buffer, &dev->port);
			FIFO_PutOne(_i2c->buffer, TCA9548A_PORT);
			dev->port |= ch;
			FIFO_PutOne(_i2c->buffer, dev->port);
			_i2c->len = 1;
			_i2c->mode = I2C_MODE_WRITE;
			dev->step = 2;
			break;
		case 2://exit
			dev->status = DEVICE_DONE;
			dev->step = 0;
			return 1;
		default:
			dev->step = 0;
			return 0;
		}
		I2C_Start_IRQ(_i2c);
	}
	return 0;
}

uint8_t TCA9548A_OffChannel(I2C_IRQ_Connection_t *_i2c, TCA9548A_t *dev, TCA9548A_ch_t ch) {
	if (_i2c->status == PORT_FREE) {
		switch (dev->step) {
		case 0://read port actual
			_i2c->addr = dev->addr;
			FIFO_PutOne(_i2c->buffer, TCA9548A_PORT);
			_i2c->len = 1;
			_i2c->mode = I2C_MODE_READ;
			dev->step = 1;
			break;
		case 1://apply channels
			_i2c->addr = dev->addr;
			FIFO_GetOne(_i2c->buffer, &dev->port);
			dev->port &= ~ch;
			FIFO_PutOne(_i2c->buffer, TCA9548A_PORT);
			FIFO_PutOne(_i2c->buffer, dev->port);
			_i2c->len = 1;
			_i2c->mode = I2C_MODE_WRITE;
			dev->step = 2;
			break;
		case 2://exit
			dev->status = DEVICE_DONE;
			dev->step = 0;
			return 1;
		default:
			dev->step = 0;
			return 0;
		}
		I2C_Start_IRQ(_i2c);
	}
	return 0;
}
