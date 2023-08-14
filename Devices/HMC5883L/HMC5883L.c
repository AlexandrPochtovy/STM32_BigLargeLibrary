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

#define HMC5883L_CHIP_ID  (uint32_t)0x00483433 //< Assumes ALT address pin low
#define HMC5883L_DATA_LEN 6

static inline uint16_t CONCAT_BYTES(uint8_t msb, uint8_t lsb) {
    return (uint16_t)(((uint16_t)msb << 8) | (uint16_t)lsb);
}	

uint8_t HMC5883L_Init(I2C_IRQ_Conn_t *_i2c, HMC5883L_dev *dev) {
	dev->status = DEVICE_NOT_INIT;
	//setup sensor
	uint8_t data[3];
	data[0] = HMC5883L_SAMPLES_1 | HMC5883L_DATARATE_15HZ | HMC5883L_NORMAL;
	data[1] = HMC5883L_GAIN_1_3GA;
	data[2] = HMC5883L_CONTINOUS;
	if (WriteRegBytes(_i2c, dev->addr, HMC5883L_REG_CONFIG_A, data, 3)) {
		dev->status = DEVICE_INIT;
		dev->step = 0;
		return 1;
	}
	return 0;
}

uint8_t HMC5883L_GetData(I2C_IRQ_Conn_t *_i2c, HMC5883L_dev *dev) {
	uint8_t data[HMC5883L_DATA_LEN];
	if (ReadRegBytes(_i2c, dev->addr, HMC5883L_REG_OUT_X_M, data, HMC5883L_DATA_LEN)) {
		dev->raw.X = CONCAT_BYTES(data[0], data[1]);
		dev->raw.Z = CONCAT_BYTES(data[2], data[3]);
		dev->raw.Y = CONCAT_BYTES(data[4], data[5]);
		dev->status = DEVICE_DONE;
		return 1;
	}
	return 0;
}
