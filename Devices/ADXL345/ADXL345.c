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

 * 	ADXL345.c
 * 	Created on: 31.01.2022
 */

#include "ADXL345.h"

#define EarthGravity 9.80665 // m/s^2
#define ADXL345_DATA_LENGHT 6
#define ADXL345_DEVICE_ID 0xE5

#ifndef _FUNCTION_H_
static inline uint16_t CONCAT_BYTES(uint8_t msb, uint8_t lsb) {
	return (((uint16_t)msb << 8) | (uint16_t)lsb);
	}
#endif
/* TODO
 add calibration when init
 */
uint8_t ADXL345_Init(I2C_IRQ_Conn_t *_i2c, ADXL345_t *dev) {
	switch (dev->status) {
		case DEVICE_READY:
			if (_i2c->status == PORT_FREE) {
				_i2c->status = PORT_BUSY;
				dev->status = DEVICE_PROCESSING;
				dev->step = 0;
				}
			break;
		case DEVICE_PROCESSING:
			switch (dev->step) {
				case 0:
					{  //check connection read chip's ID
					uint8_t ID = 0;
					if (I2C_ReadOneByte(_i2c, dev->addr, ADXL345_DEVID_REG, &ID) && (_i2c->status == PORT_BUSY)) {
						if (ID == ADXL345_DEVICE_ID) {
							dev->step = 1;
							}
						else {
							dev->status = DEVICE_FAULTH;
							_i2c->status = PORT_FREE;
							return 1;
							}
						}
					break;
					}
				case 1:
					{ // setup TAP FreeFall Threshold and Offsets 0x1D..0x2A registers
					uint8_t data[14];
					data[0] = 0xFF;	 // 0x1D  threshold value 62.5mg/bit = 16g
					data[1] = 0x00;	 // 0x1E  offset X = 0
					data[2] = 0x00;	 // 0x1F  offset Y = 0
					data[3] = 0x00;	 // 0x20  offset Z = 0
					data[4] = 0x00;	 // 0x21  duration tap|dtap value 625 mcs/bit
					data[5] = 0x00;	 // 0x22  pause between double-tap value 1.25 ms/bit
					data[6] = 0x00;	 // 0x23  window double-tap value 1.25 ms/bit
					data[7] = 0xFF;	 // 0x24  threshold act value 62.5mg/bit = 16g
					data[8] = 0x0F;	 // 0x25  threshold inact value 62.5mg/bit = 16g
					data[9] = 0xFF;	 // 0x26  time inact value 1sec/bit = 15sec
					data[10] = 0x00; // 0x27 axis act|inact control enable
					data[11] = 0x07; // 0x28 free fall value detect 62.5mg/bit
					data[12] = 0x20; // 0x29 free fall time 5 msec/bit = 200msec
					data[13] = 0x00; // 0x2A disable TAP detection
					if (I2C_WriteBytes(_i2c, dev->addr, ADXL345_THRESH_TAP_REG, data, 14) && (_i2c->status == PORT_BUSY)) {
						dev->step = 2;
						}
					break;
					}
				case 2:
					{ // setup power samplerate interrupt etc
					uint8_t data[4] = { 0 };
					data[0] = ADXL345_BW_25;																						// 0x2C
					data[1] = ADXL345_POWER_CTL_MEASURE | ADXL345_POWER_CTL_WAKEUP_8Hz; // 0x2D
					data[2] = ADXL345_INT_ENABLE_DATA_READY;														// 0x2E
					data[3] = (uint8_t)~ADXL345_INT_MAP_DATA_READY;											// 0x2F
					if (I2C_WriteBytes(_i2c, dev->addr, ADXL345_BW_RATE_REG, data, 4) && (_i2c->status == PORT_BUSY)) {
						dev->step = 3;
						}
					break;
					}
				case 3: // setup data format
					if (I2C_WriteOneByte(_i2c, dev->addr, ADXL345_DATA_FORMAT_REG, 0x00) && (_i2c->status == PORT_BUSY)) {
						dev->step = 4;
						}
					break;
				case 4: // setup FIFO
					if (I2C_WriteOneByte(_i2c, dev->addr, ADXL345_FIFO_CTL_REG, 0x00) && (_i2c->status == PORT_BUSY)) {
						dev->status = DEVICE_DONE;
						}
					break;
				default:
					break;
				}
			if (_i2c->status == PORT_ERROR) {
				dev->status = DEVICE_ERROR;
				}
			break;
		case DEVICE_DONE:
			dev->status = DEVICE_READY;
			_i2c->status = PORT_FREE;
			return 1;
		case DEVICE_ERROR:
			if (++dev->errCount >= dev->errLimit) {
				dev->status = DEVICE_FAULTH;
				_i2c->status = PORT_FREE;
				}
			else {
				dev->status = DEVICE_READY;
				_i2c->status = PORT_FREE;
				dev->step = 0;
				}
			break;
		case DEVICE_FAULTH:
			return 1;
		default:
			break;
		}
	return 0;
	}

/*****************************************************************
 * @brief convert gyroscope raw data to normal float data and store in main gyroscope structure
 * internal use only
 * @param _i2c - pointer to I2C bus connection structure
 * @param dev - pointer to gyroscope main structure
 * @retval 1 when end
 */
float ADXL345_ConvertData(int16_t raw, enum ratio factor) {
	return ((2 * raw * factor) * EarthGravity) / 1000.0f;
	}

uint8_t ADXL345_GetData(I2C_IRQ_Conn_t *_i2c, ADXL345_t *dev) {
	switch (dev->status) {
		case DEVICE_READY:
			if (_i2c->status == PORT_FREE) {
				_i2c->status = PORT_BUSY;
				dev->status = DEVICE_PROCESSING;
				dev->step = 0;
				}
			break;
		case DEVICE_PROCESSING:
			{
			uint8_t val[ADXL345_DATA_LENGHT] = { 0 };
			if (I2C_ReadBytes(_i2c, dev->addr, ADXL345_DATAX0_REG, val, ADXL345_DATA_LENGHT) && (_i2c->status == PORT_BUSY)) {
				dev->raw.X = (int16_t)CONCAT_BYTES(val[1], val[0]);
				dev->data.X = ADXL345_ConvertData(dev->raw.X, RATIO_2G);
				dev->raw.Y = (int16_t)CONCAT_BYTES(val[3], val[2]);
				dev->data.Y = ADXL345_ConvertData(dev->raw.Y, RATIO_2G);
				dev->raw.Z = (int16_t)CONCAT_BYTES(val[5], val[4]);
				dev->data.Z = ADXL345_ConvertData(dev->raw.Z, RATIO_2G);
				dev->status = DEVICE_DONE;
				}
			else if (_i2c->status == PORT_ERROR) {
				dev->status = DEVICE_ERROR;
				}
			break;
			}
		case DEVICE_DONE:
			dev->status = DEVICE_READY;
			_i2c->status = PORT_FREE;
			return 1;
		case DEVICE_ERROR:
			if (++dev->errCount >= dev->errLimit) {
				dev->status = DEVICE_FAULTH;
				_i2c->status = PORT_FREE;
				}
			else {
				dev->status = DEVICE_READY;
				_i2c->status = PORT_FREE;
				dev->step = 0;
				}
			break;
		case DEVICE_FAULTH:
			return 1;
		default:
			break;
		}
	return 0;
	}
