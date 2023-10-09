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

	 Created on: Aug 1, 2022
 ********************************************************************************/


#include "TCA9548A.h"

uint8_t TCA9548A_Init(I2C_IRQ_Conn_t *_i2c, TCA9548A_t *dev) {
	PortStatus_t st;
	if (dev->status == DEVICE_FAULTH) {
		return 1;
		}
	else if ((dev->status == DEVICE_READY) && (_i2c->status == PORT_FREE)) {
		_i2c->status = PORT_BUSY;
		dev->status = DEVICE_PROCESSING;
		dev->port = 0x00;
		}
	if (dev->status == DEVICE_PROCESSING) {
		st = I2C_WriteOne(_i2c, dev->addr, dev->port);
		if (st == PORT_DONE) {
			_i2c->status = PORT_FREE;
			dev->status = DEVICE_READY;
			return 1;
			}
		}
	if ((st == PORT_ERROR) && (++dev->errCount >= dev->errLimit)) {
		dev->status = DEVICE_FAULTH;
		_i2c->status = PORT_FREE;
		return 1;
		}
	return 0;
	}

uint8_t TCA9548A_OnChannels(I2C_IRQ_Conn_t *_i2c, TCA9548A_t *dev, uint8_t channelMask) {
	PortStatus_t st;
	if (dev->status == DEVICE_FAULTH) {
		return 1;
		}
	else if ((dev->status == DEVICE_READY) && (_i2c->status == PORT_FREE)) {
		_i2c->status = PORT_BUSY;
		dev->status = DEVICE_PROCESSING;
		dev->port = channelMask;
		}
	if (dev->status == DEVICE_PROCESSING) {
		st = I2C_WriteOne(_i2c, dev->addr, dev->port);
		if (st == PORT_DONE) {
			_i2c->status = PORT_FREE;
			dev->status = DEVICE_READY;
			return 1;
			}
		}
	if ((st == PORT_ERROR) && (++dev->errCount >= dev->errLimit)) {
		dev->status = DEVICE_FAULTH;
		_i2c->status = PORT_FREE;
		return 1;
		}
	return 0;
	}

uint8_t TCA9548A_OffChannels(I2C_IRQ_Conn_t *_i2c, TCA9548A_t *dev, uint8_t channelMask) {
	PortStatus_t st;
	if (dev->status == DEVICE_FAULTH) {
		return 1;
		}
	else if ((dev->status == DEVICE_READY) && (_i2c->status == PORT_FREE)) {
		_i2c->status = PORT_BUSY;
		dev->status = DEVICE_PROCESSING;
		dev->port &= ~channelMask;
		}
	if (dev->status == DEVICE_PROCESSING) {
		st = I2C_WriteOne(_i2c, dev->addr, dev->port);
		if (st == PORT_DONE) {
			_i2c->status = PORT_FREE;
			dev->status = DEVICE_READY;
			return 1;
			}
		}
	if ((st == PORT_ERROR) && (++dev->errCount >= dev->errLimit)) {
		dev->status = DEVICE_FAULTH;
		_i2c->status = PORT_FREE;
		return 1;
		}
	return 0;
	}
