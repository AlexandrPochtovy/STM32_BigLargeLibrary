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

#define Chip_GYRO_LSB 14.375F
#define Chip_TEMP_LSB  280
#define ITG3205_DATA_LEN  8

static inline uint16_t CONCAT_BYTES(uint8_t msb, uint8_t lsb) {
	return (uint16_t)(((uint16_t)msb << 8) | (uint16_t)lsb);
}

uint8_t ITG3205_Init(I2C_IRQ_Conn_t *_i2c, ITG3205_t *dev) {
	dev->status = DEVICE_PROCESSING;
	switch (dev->step) {
	case 0: {//read who i am register check connection
		uint8_t ID = 0;
		if (I2C_ReadOneByte(_i2c, dev->addr, ITG3205_WHOAMI, &ID)) {
			if (((ID & 0xFE) << 1) == ITG3205_ADDR) {
				dev->step = 1;
			} else {
				dev->step = 0;
				dev->status = DEVICE_FAULTH;
				return 1;
			}
		}
		break;}
		case 1: //setup clock
			if (I2C_WriteOneByte(_i2c, dev->addr, ITG3205_PWR_MGM, ITG3205_PWR_CLOCK_INTERNAL)) {
				dev->step = 2;
			}
			break;
		case 2: //setup sample rate
			if (I2C_WriteOneByte(_i2c, dev->addr, ITG3205_SMPLRT_DIV, 0x07)) {
				dev->step = 3;
			}
			break;
		case 3: //setup low-pass filter
			if (I2C_WriteOneByte(_i2c, dev->addr, ITG3205_DLPF_FS, ITG3205_DLPF_FS_SEL | ITG3205_DLPF_CFG_5Hz)) {
				dev->step = 4;
			}
			break;
		case 4:
			if (I2C_WriteOneByte(_i2c, dev->addr, ITG3205_INT_CFG, 0x00)) {
				dev->status = DEVICE_READY;
				dev->step = 0;
				return 1;
			}
			break;
		default:
			dev->step = 0;
			break;
		}
	return 0;
}

uint8_t ITG3205_GetData(I2C_IRQ_Conn_t *_i2c, ITG3205_t *dev) {
	dev->status = DEVICE_PROCESSING;
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
