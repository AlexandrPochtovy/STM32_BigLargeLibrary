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

 * 	ADXL345.c
 * 	Created on: 31.01.2022
 */

#include "ADXL345.h"

const float kRatio2g = (float) (2 * 2) / 1024.0f;
const float kRatio4g = (float) (4 * 2) / 1024.0f;
const float kRatio8g = (float) (8 * 2) / 1024.0f;
const float kRatio16g = (float) (16 * 2) / 1024.0f;
const float my_gravity = 9.80665; // m/s^2

static inline uint16_t CONCAT_BYTES(uint8_t msb, uint8_t lsb) {
	return (((uint16_t) msb << 8) | (uint16_t) lsb);
}

uint8_t ADXL345_Init(I2C_IRQ_Connection_t *_i2c, ADXL345_t *dev) {
	if (_i2c->status == PORT_FREE) {//send setup
		_i2c->addr = dev->addr;
		switch (dev->step) {
			case 0: { //setup TAP FreeFall Threshold and Offsets 0x1D..0x2A registers
				dev->status = DEVICE_NOT_INIT;
				FIFO_PutOne(_i2c->buffer, ADXL345_THRESH_TAP_REG);
				_i2c->len = 14;
				_i2c->mode = I2C_MODE_WRITE;
				FIFO_PutOne(_i2c->buffer, 0xFF); //0x1D  threshold value 62.5mg/bit = 16g
				FIFO_PutOne(_i2c->buffer, 0x00); //0x1E  offset X = 0
				FIFO_PutOne(_i2c->buffer, 0x00); //0x1F  offset Y = 0
				FIFO_PutOne(_i2c->buffer, 0x00); //0x20  offset Z = 0
				FIFO_PutOne(_i2c->buffer, 0x00); //0x21  duration tap|dtap value 625 mcs/bit
				FIFO_PutOne(_i2c->buffer, 0x00); //0x22  pause between dtap value 1.25 ms/bit
				FIFO_PutOne(_i2c->buffer, 0x00); //0x23  window dtap value 1.25 ms/bit
				FIFO_PutOne(_i2c->buffer, 0xFF); //0x24  threshold act value 62.5mg/bit = 16g
				FIFO_PutOne(_i2c->buffer, 0x0F); //0x25  threshold inact value 62.5mg/bit = 16g
				FIFO_PutOne(_i2c->buffer, 0x0F); //0x26  time inact value 1sec/bit = 15sec
				FIFO_PutOne(_i2c->buffer, 0x00); //0x27 axis act|inact control enable
				FIFO_PutOne(_i2c->buffer, 0x07); //0x28 free fall value detect 62.5mg/bit
				FIFO_PutOne(_i2c->buffer, 0x28); //0x29 free fall time 5 msec/bit = 200msec
				FIFO_PutOne(_i2c->buffer, 0x00); //0x2A disable TAP detection
				dev->step = 1;
				break;
			}
			case 1: { //setup power samplerate interrupt etc
				FIFO_PutOne(_i2c->buffer, ADXL345_BW_RATE_REG);
				_i2c->len = 4;
				_i2c->mode = I2C_MODE_WRITE;
				FIFO_PutOne(_i2c->buffer, ADXL345_BW_100); //0x2C
				FIFO_PutOne(_i2c->buffer, ADXL345_POWER_CTL_MEASURE
																| ADXL345_POWER_CTL_WAKEUP_8Hz); //0x2D
				FIFO_PutOne(_i2c->buffer, ADXL345_INT_ENABLE_DATA_READY); //0x2E
				FIFO_PutOne(_i2c->buffer, (uint8_t)~ADXL345_INT_MAP_DATA_READY); //0x2F
				dev->step = 2;
				break;
			}
			case 2: //setup data format
				FIFO_PutOne(_i2c->buffer, ADXL345_DATA_FORMAT_REG);
				_i2c->len = 1;
				_i2c->mode = I2C_MODE_WRITE;
				FIFO_PutOne(_i2c->buffer, 0x00);
				dev->step = 3;
				break;
			case 3: //setup FIFO
				FIFO_PutOne(_i2c->buffer, ADXL345_FIFO_CTL_REG);
				_i2c->len = 1;
				_i2c->mode = I2C_MODE_WRITE;
				FIFO_PutOne(_i2c->buffer, 0x00);
				dev->step = 4;
				break;
			case 4:
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

float ADXL345_ConvertData(int16_t raw) {
	uint8_t range = ADXL345_DATA_FORMAT_RANGE_2G;
	switch (range) {
		case ADXL345_DATA_FORMAT_RANGE_2G:
			return (float) raw * my_gravity * kRatio2g;
			break;
		case ADXL345_DATA_FORMAT_RANGE_4G:
			return (float) raw * my_gravity * kRatio4g;
			break;
		case ADXL345_DATA_FORMAT_RANGE_8G:
			return (float) raw * my_gravity * kRatio8g;
			break;
		case ADXL345_DATA_FORMAT_RANGE_16G:
			return (float) raw * my_gravity * kRatio16g;
			break;
		default:
			return 0;
			break;
	}
}

uint8_t ADXL345_GetData(I2C_IRQ_Connection_t *_i2c, ADXL345_t *dev) {
	uint8_t val[ADXL345_DATA_LENGHT];
	if (_i2c->status == PORT_FREE) {//
		_i2c->addr = dev->addr;
		switch (dev->step) {
			case 0: //get data
				FIFO_PutOne(_i2c->buffer, ADXL345_DATAX0_REG);
				_i2c->len = ADXL345_DATA_LENGHT;
				_i2c->mode = I2C_MODE_READ;
				dev->step = 1;
				break;
			case 1: { //convert data
				FIFO_GetMulti(_i2c->buffer, val, ADXL345_DATA_LENGHT);
				dev->raw.X = (int16_t)CONCAT_BYTES(val[1], val[0]);
				dev->data.X = ADXL345_ConvertData(dev->raw.X);
				dev->raw.Y = (int16_t)CONCAT_BYTES(val[3], val[2]);
				dev->data.Y = ADXL345_ConvertData(dev->raw.Y);
				dev->raw.Z = (int16_t)CONCAT_BYTES(val[5], val[4]);
				dev->data.Z = ADXL345_ConvertData(dev->raw.Z);
				dev->status = DEVICE_DONE;
				dev->step = 0;
				return 1;
			}
			default:
				dev->step = 0;
				break;
		}
		I2C_Start_IRQ(_i2c);
	}
	return 0;
}

