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

#ifndef _FUNCTION_H_
static inline uint16_t CONCAT_BYTES(uint8_t msb, uint8_t lsb) {
	return (uint16_t)(((uint16_t)msb << 8) | (uint16_t)lsb);
	}
#endif

uint8_t ITG3205_Init(I2C_IRQ_Conn_t *port, ITG3205_t *dev) {
	PortStatus_t st;
	if (dev->status == DEVICE_FAULTH) {
		return 1;
		}
	else if ((dev->status == DEVICE_READY) && (port->status == PORT_FREE)) {
		port->status = PORT_BUSY;
		dev->status = DEVICE_PROCESSING;
		dev->step = 0;
		}
	switch (dev->step) {
		case 0: //read who i am register check connection
			if (dev->status == DEVICE_PROCESSING) {
				uint8_t ID = 0;
				st = I2C_ReadOneByte(port, dev->addr, ITG3205_WHOAMI, &ID);
				if (st == PORT_DONE) {
					if (((ID & 0xFE) << 1) == ITG3205_ADDR) {
						port->status = PORT_BUSY;
						dev->step = 1;
						}
					else {
						dev->status = DEVICE_FAULTH;
						port->status = PORT_FREE;
						return 1;
						}
					}
				}
			break;
		case 1: //setup clock
			if (dev->status == DEVICE_PROCESSING) {
				st = I2C_WriteOneByte(port, dev->addr, ITG3205_PWR_MGM, ITG3205_PWR_CLOCK_INTERNAL);
				if (st == PORT_DONE) {
					port->status = PORT_BUSY;
					dev->step = 2;
					}
				}
			break;
		case 2: //setup sample rate
			if (dev->status == DEVICE_PROCESSING) {
				st = I2C_WriteOneByte(port, dev->addr, ITG3205_SMPLRT_DIV, 0x07);
				if (st == PORT_DONE) {
					port->status = PORT_BUSY;
					dev->step = 3;
					}
				}
			break;
		case 3: //setup low-pass filter
			if (dev->status == DEVICE_PROCESSING) {
				st = I2C_WriteOneByte(port, dev->addr, ITG3205_DLPF_FS, ITG3205_DLPF_FS_SEL | ITG3205_DLPF_CFG_5Hz);
				if (st == PORT_DONE) {
					port->status = PORT_BUSY;
					dev->step = 4;
					}
				}
			break;
		case 4:
			if (dev->status == DEVICE_PROCESSING) {
				st = I2C_WriteOneByte(port, dev->addr, ITG3205_INT_CFG, 0x00);
				if (st == PORT_DONE) {
					dev->status = DEVICE_DONE;
					dev->step = 0;
					}
				}
			break;
		default:
			break;
		}
	if (dev->status == DEVICE_DONE) {
		port->status = PORT_FREE;
		dev->status = DEVICE_READY;
		return 1;
		}
	else if ((st == PORT_ERROR) && (++dev->errCount >= dev->errLimit)) {
		dev->status = DEVICE_FAULTH;
		port->status = PORT_FREE;
		return 1;
		}
	return 0;
	}

uint8_t ITG3205_GetData(I2C_IRQ_Conn_t *port, ITG3205_t *dev) {
	PortStatus_t st;
	if (dev->status == DEVICE_FAULTH) {
		return 1;
		}
	else if ((dev->status == DEVICE_READY) && (port->status == PORT_FREE)) {
		port->status = PORT_BUSY;
		dev->status = DEVICE_PROCESSING;
		dev->step = 0;
		}
	else if (dev->status == DEVICE_PROCESSING) {
		uint8_t dt[ITG3205_DATA_LEN];
		st = I2C_ReadBytes(port, dev->addr, ITG3205_TEMP_OUT_H, dt, ITG3205_DATA_LEN);
		if (st == PORT_DONE) {
			dev->raw.temp = (int16_t)CONCAT_BYTES(dt[0], dt[1]);
			dev->raw.X = (int16_t)CONCAT_BYTES(dt[2], dt[3]);
			dev->raw.Y = (int16_t)CONCAT_BYTES(dt[4], dt[5]);
			dev->raw.Z = (int16_t)CONCAT_BYTES(dt[6], dt[7]);
			dev->data.temp = 35 + ((float)(dev->raw.temp + 13200)) / Chip_TEMP_LSB;
			dev->data.X = (float)dev->raw.X / Chip_GYRO_LSB * M_PI / 180;
			dev->data.Y = (float)dev->raw.Y / Chip_GYRO_LSB * M_PI / 180;
			dev->data.Z = (float)dev->raw.Z / Chip_GYRO_LSB * M_PI / 180;
			dev->status = DEVICE_DONE;
			}
		}
	if (dev->status == DEVICE_DONE) {
		port->status = PORT_FREE;
		dev->status = DEVICE_READY;
		return 1;
		}
	else if ((st == PORT_ERROR) && (++dev->errCount >= dev->errLimit)) {
		dev->status = DEVICE_FAULTH;
		port->status = PORT_FREE;
		return 1;
		}
	return 0;
	}
