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
   
 	HMC5883L.с
	Created on: 31.01.2022
 ********************************************************************************/

#include "HMC5883L.h"

static const uint32_t HMC5883L_CHIP_ID = 0x00483433; //< Assumes ALT address pin low

static inline uint16_t CONCAT_BYTES(uint8_t msb, uint8_t lsb) {
    return (uint16_t)(((uint16_t)msb << 8) | (uint16_t)lsb);
}	

uint8_t HMC5883L_Init(I2C_IRQ_Connection_t *_i2c, HMC5883L_dev *dev) {
	if (_i2c->status == PORT_FREE)	{ //send setup
		dev->status = DEVICE_NOT_INIT;
		switch (dev->step) {
		case 0: {//setup sensor
			_i2c->addr = dev->addr;
			FIFO_PutOne(_i2c->buffer, HMC5883L_REG_CONFIG_A);
			_i2c->len = 3;
			_i2c->mode = I2C_MODE_WRITE;
			uint8_t dt[3];
			dt[0] = HMC5883L_SAMPLES_1 | HMC5883L_DATARATE_15HZ | HMC5883L_NORMAL;
			dt[1] = HMC5883L_GAIN_1_3GA;
			dt[2] = HMC5883L_CONTINOUS;
			FIFO_PutMulti(_i2c->buffer, dt, 3);
			dev->step = 1;
			break; }
		case 1:
			dev->status = DEVICE_INIT;
			dev->step = 0;
			return 1;
		default:
			dev->step = 0;
			break;
		}
		I2C_Start_IRQ(_i2c);
	}
	return 0;
}

uint8_t HMC5883L_GetData(I2C_IRQ_Connection_t *_i2c, HMC5883L_dev *dev) {
	if (_i2c->status == PORT_FREE) {
		switch (dev->step) {
		case 0: //read data
			_i2c->addr = dev->addr;
			FIFO_PutOne(_i2c->buffer, HMC5883L_REG_OUT_X_M);
			_i2c->len = 6;
			_i2c->mode = I2C_MODE_READ;
			dev->step = 1;
			break;
		case 1: {//calculate data
			uint8_t dt[6];
			FIFO_GetMulti(_i2c->buffer, dt, 6);
			dev->raw.X = CONCAT_BYTES(dt[0], dt[1]);
			dev->raw.Z = CONCAT_BYTES(dt[2], dt[3]);
			dev->raw.Y = CONCAT_BYTES(dt[4], dt[5]);
			dev->status = DEVICE_DONE;
			dev->step = 0;
			return 1; }
		default:
			dev->step = 0;
			break;
		}
		I2C_Start_IRQ(_i2c);
	}
	return 0;
}
