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

	INA219.с
	Created on: 27.01.2022
 ********************************************************************************/

#include "INA219.h"

#define INA219_REG_LEN 2									//register length in byte
#define INA219_MaxCurrentmA 3200U					//maximum curent in mA
#define INA219_ShuntResistance_mOmh 100U	//shunt resistance in milliOmh
#define INA219_CalibrationVal	0x08000000U / ((uint32_t)(INA219_MaxCurrentmA * INA219_ShuntResistance_mOmh) / 10)
#define INA219_Current_LSB_mkA	INA219_MaxCurrentmA * 1000 / 0x8000
#define INA219_Power_LSB_mkV		(uint32_t)(20 * INA219_Current_LSB_mkA)

static inline uint16_t CONCAT_BYTES(uint8_t msb, uint8_t lsb) {
	return (uint16_t)(((uint16_t)msb << 8) | (uint16_t)lsb);
	}

//Init & setup	=============================================================================================
uint8_t INA219_Init(I2C_IRQ_Conn_t *_i2c, INA219_t *dev) {
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
					uint16_t cfg = INA219_CFG_MODE_SHBV_CONTINUOUS |	// shunt and bus voltage continuous	defaulth
						INA219_CFG_SADC_12BIT_128S |	// 128 x 12-bit shunt samples averaged together
						INA219_CFG_BADC_12BIT_128S |	// 128 x 12-bit bus samples averaged together
						INA219_CFG_GAIN_8_320MV |	// Gain 8, 320mV Range defaulth
						INA219_CFG_BVRANGE_16V;				// 0-16V Range
					if (I2C_WriteBytes(_i2c, dev->addr, INA219_REG_CONFIG, (uint8_t *)&cfg, INA219_REG_LEN)) {
						if (_i2c->status == PORT_DONE) {
							_i2c->status = PORT_BUSY;
							dev->step = 1;
							}
						else if (_i2c->status == PORT_ERROR) {
							dev->status = DEVICE_ERROR;
							}
						}
					break;
				case 1:
					uint16_t data = INA219_CalibrationVal;
					if (I2C_WriteBytes(_i2c, dev->addr, INA219_REG_CALIBRATION, (uint8_t *)&data, INA219_REG_LEN)) {
						if (_i2c->status == PORT_DONE) {
							_i2c->status = PORT_BUSY;
							dev->status = DEVICE_DONE;
							}
						else if (_i2c->status == PORT_ERROR) {
							dev->status = DEVICE_ERROR;
							}
						}
					break;
				default:
					break;
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

uint8_t INA219_GetData(I2C_IRQ_Conn_t *_i2c, INA219_t *dev) {
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
				case 0://read  voltage
					uint8_t dt[INA219_REG_LEN];
					if (I2C_ReadBytes(_i2c, dev->addr, INA219_REG_BUSVOLTAGE, dt, INA219_REG_LEN)) {
						if (_i2c->status == PORT_DONE) {
							dev->raw.voltage = (uint16_t)CONCAT_BYTES(dt[0], dt[1]);
							_i2c->status = PORT_BUSY;
							dev->step = 1;
							}
						else if (_i2c->status == PORT_ERROR) {
							dev->status = DEVICE_ERROR;
							}
						}
					break;
				case 1://read power
					uint8_t dt[INA219_REG_LEN];
					if (I2C_ReadBytes(_i2c, dev->addr, INA219_REG_POWER, dt, INA219_REG_LEN)) {
						if (_i2c->status == PORT_DONE) {
							dev->raw.power = (uint16_t)CONCAT_BYTES(dt[0], dt[1]);
							_i2c->status = PORT_BUSY;
							dev->step = 2;
							}
						else if (_i2c->status == PORT_ERROR) {
							dev->status = DEVICE_ERROR;
							}
						}
					break;
				case 2://read current
					uint8_t dt[INA219_REG_LEN];
					if (I2C_ReadBytes(_i2c, dev->addr, INA219_REG_CURRENT, dt, INA219_REG_LEN)) {
						if (_i2c->status == PORT_DONE) {
							dev->raw.current = (uint16_t)CONCAT_BYTES(dt[0], dt[1]);
							_i2c->status = PORT_BUSY;
							dev->step = 3;
							}
						else if (_i2c->status == PORT_ERROR) {
							dev->status = DEVICE_ERROR;
							}
						}
					break;
				case 3://read shunt  voltage
					uint8_t dt[INA219_REG_LEN];
					if (I2C_ReadBytes(_i2c, dev->addr, INA219_REG_SHUNTVOLTAGE, dt, INA219_REG_LEN)) {
						if (_i2c->status == PORT_DONE) {
							dev->raw.shuntV = (uint16_t)CONCAT_BYTES(dt[0], dt[1]);
							dev->status = DEVICE_DONE;
							}
						else if (_i2c->status == PORT_ERROR) {
							dev->status = DEVICE_ERROR;
							}
						}
					break;
				default:
					break;
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
//Get & conversion raw data	=================================================================================
uint16_t INA219_GetVoltageInt(INA219_t *dev, enum INA219_DIVIDERS divider) {
	return (uint16_t)(((uint32_t)(dev->raw.voltage >> 3) * 4) / divider);
	}

float INA219_GetVoltageFloat(INA219_t *dev, enum INA219_DIVIDERS divider) {
	return ((float)((uint32_t)(dev->raw.voltage >> 3) * 4)) / divider;
	}

uint16_t INA219_GetCurrentInt(INA219_t *dev, enum INA219_DIVIDERS divider) {
	return (uint16_t)(((uint32_t)dev->raw.current * INA219_Current_LSB_mkA) / divider);
	}

float INA219_GetCurrentFloat(INA219_t *dev, enum INA219_DIVIDERS divider) {
	return ((float)((uint32_t)dev->raw.current * INA219_Current_LSB_mkA)) / divider;
	}

uint16_t INA219_GetPowerInt(INA219_t *dev, enum INA219_DIVIDERS divider) {
	return (uint16_t)(((uint32_t)dev->raw.power * INA219_Power_LSB_mkV) / divider);
	}

float INA219_GetPowerFloat(INA219_t *dev, enum INA219_DIVIDERS divider) {
	return (float)(((uint32_t)dev->raw.power * INA219_Power_LSB_mkV)) / divider;
	}

//#endif
