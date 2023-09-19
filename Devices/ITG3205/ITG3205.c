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
   
 * 	ITG3205.с
 *  Created on: Jan 30, 2022
 ********************************************************************************/

#include "ITG3205.h"

static const float Chip_GYRO_LSB = 14.375;
static const uint16_t Chip_TEMP_LSB = 280;
static const uint8_t ITG3205_DATA_LEN = 8;

static inline uint16_t CONCAT_BYTES(uint8_t msb, uint8_t lsb) {
	return (uint16_t)(((uint16_t)msb << 8) | (uint16_t)lsb);
}

uint8_t ITG3205_Init(I2C_IRQ_Conn_t *_i2c, ITG3205_t *dev) {
	dev->status = DEVICE_NOT_INIT;
	switch (dev->step) {
		case 0: //reset first
			if (I2C_WriteOneByte(_i2c, dev->addr, ITG3205_PWR_MGM, ITG3205_PWR_MGM_RESET)) {
				dev->step = 1;
			}
			break;
		case 1: //setup clock
			if (I2C_WriteOneByte(_i2c, dev->addr, ITG3205_PWR_MGM, ITG3205_PWR_CLOCK_INTERNAL)) {
				dev->step = 2;
			}
			break;
		case 2:{ //setup sample rate, interrupt
			uint8_t dt[3];
			dt[0] = 0xFF; //ITG3205_SMPLRT_DIV
			dt[1] = ITG3205_DLPF_FS_SEL | ITG3205_DLPF_CFG_256Hz;
			dt[2] = ITG3205_INT_CFG_INT_ANYRD_2CLEAR;
			FIFO_PutMulti(_i2c->buffer, dt, 3);
			if (I2C_WriteBytes(_i2c, dev->addr, ITG3205_SMPLRT_DIV, dt, 3)) {
				dev->status = DEVICE_INIT;
				dev->step = 0;
				return 1;
			}
			break;}
		default:
			dev->step = 0;
			break;
		}
	return 0;
}

uint8_t ITG3205_GetData(I2C_IRQ_Conn_t *_i2c, ITG3205_t *dev) {
	uint8_t dt[ITG3205_DATA_LEN];
	if (I2C_ReadBytes(_i2c, dev->addr, ITG3205_TEMP_OUT_H, dt, ITG3205_DATA_LEN)) {
		dev->raw.temp = (int16_t)CONCAT_BYTES(dt[0], dt[1]);
		dev->raw.X = (int16_t)CONCAT_BYTES(dt[2], dt[3]);
		dev->raw.Y = (int16_t)CONCAT_BYTES(dt[4], dt[5]);
		dev->raw.Z = (int16_t)CONCAT_BYTES(dt[6], dt[7]);
		dev->data.temp = 35 + ((float) (dev->raw.temp + 13200)) / Chip_TEMP_LSB;
		dev->data.X = (float) dev->raw.X / Chip_GYRO_LSB * M_PI / 180;
		dev->data.Y = (float) dev->raw.Y / Chip_GYRO_LSB * M_PI / 180;
		dev->data.Z = (float) dev->raw.Z / Chip_GYRO_LSB * M_PI / 180;
		dev->status = DEVICE_DONE;
		return 1;
	}
	return 0;
}
