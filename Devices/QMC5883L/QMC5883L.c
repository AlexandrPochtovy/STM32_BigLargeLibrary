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

 * 	QMC5883L.c
 *  Created on: Jan 31, 2022
 ********************************************************************************/

#include "QMC5883L.h"


#define QMC5883L_DATA_LEN 6

static inline uint16_t CONCAT_BYTES(uint8_t msb, uint8_t lsb) {
	return (((uint16_t)msb << 8) | (uint16_t)lsb);
	}

uint8_t QMC5883L_Init(I2C_IRQ_Conn_t *_i2c, QMC5883L_t *dev) {
	PortStatus_t st;
	if (dev->status == DEVICE_FAULTH) {
		return 1;
		}
	else if ((dev->status == DEVICE_READY) && (_i2c->status == PORT_FREE)) {
		_i2c->status = PORT_BUSY;
		dev->status = DEVICE_PROCESSING;
		dev->step = 0;
		}
	switch (dev->step) {
		case 0: //set reset period don't give a fuck recommended magic number 0x01 RTFM OMG!!!
			if (dev->status == DEVICE_PROCESSING) {
				st = I2C_WriteOneByte(_i2c, dev->addr, QMC5883L_REG_PERIOD, 0x01);
				if (st == PORT_DONE) {
					_i2c->status = PORT_BUSY;
					dev->step = 1;
					}
				}
			break;
		case 1: //setup sensor gain and sample rate interrupt
			if (dev->status == DEVICE_PROCESSING) {
				uint8_t data[2];
				data[0] = QMC5883L_MODE_CONTINUOUS | QMC5883L_ODR_50HZ | QMC5883L_RNG_2G | QMC5883L_SAMPLES_512;
				data[1] = 0x00;
				st = I2C_WriteBytes(_i2c, dev->addr, QMC5883L_REG_CFG_A, data, 2);
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
		_i2c->status = PORT_FREE;
		dev->status = DEVICE_READY;
		return 1;
		}
	else if ((st == PORT_ERROR) && (++dev->errCount >= dev->errLimit)) {
		dev->status = DEVICE_FAULTH;
		_i2c->status = PORT_FREE;
		return 1;
		}
	return 0;
	}

uint8_t QMC5883L_GetData(I2C_IRQ_Conn_t *_i2c, QMC5883L_t *dev) {
	PortStatus_t st;
	if (dev->status == DEVICE_FAULTH) {
		return 1;
		}
	else if ((dev->status == DEVICE_READY) && (_i2c->status == PORT_FREE)) {
		_i2c->status = PORT_BUSY;
		dev->status = DEVICE_PROCESSING;
		}
	if (dev->status == DEVICE_PROCESSING) {
		uint8_t dt[QMC5883L_DATA_LEN];
		st = I2C_ReadBytes(_i2c, dev->addr, QMC5883L_REG_OUT_X_L, dt, QMC5883L_DATA_LEN);
		if (st == PORT_DONE) {
			dev->raw.X = (int16_t)CONCAT_BYTES(dt[1], dt[0]);
			dev->raw.Y = (int16_t)CONCAT_BYTES(dt[3], dt[2]);
			dev->raw.Z = (int16_t)CONCAT_BYTES(dt[5], dt[4]);
			dev->data.X = (float)dev->raw.X * QMC5883L_LSB_2G / 32768;
			dev->data.Y = (float)dev->raw.Y * QMC5883L_LSB_2G / 32768;
			dev->data.Z = (float)dev->raw.Z * QMC5883L_LSB_2G / 32768;
			dev->status = DEVICE_DONE;
			}
		}
	if (dev->status == DEVICE_DONE) {
		_i2c->status = PORT_FREE;
		dev->status = DEVICE_READY;
		return 1;
		}
	else if ((st == PORT_ERROR) && (++dev->errCount >= dev->errLimit)) {
		dev->status = DEVICE_FAULTH;
		_i2c->status = PORT_FREE;
		return 1;
		}
	return 0;
	}
