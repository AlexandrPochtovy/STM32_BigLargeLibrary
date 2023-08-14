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
   
	INA219.с
 	Created on: 27.01.2022
 ********************************************************************************/

#include "INA219.h"

static const uint8_t INA219_REG_LEN = 2;
static const uint32_t  INA219_MaxCurrentmA = 3200;//maximum curent in mA
static const uint32_t  INA219_ShuntResistance_mOmh = 100;//shunt resistance in milliOmh

#define INA219_CalibrationVal		(uint32_t)0x08000000 / ((uint32_t)(INA219_MaxCurrentmA * INA219_ShuntResistance_mOmh) / 10)
#define INA219_Current_LSB_mkA	(uint32_t)(INA219_MaxCurrentmA * 1000 / 0x8000)
#define INA219_Power_LSB_mkV		(uint32_t)(20 * INA219_Current_LSB_mkA)

static inline uint16_t CONCAT_BYTES(uint8_t msb, uint8_t lsb) {
    return (uint16_t)(((uint16_t)msb << 8) | (uint16_t)lsb);
}

//Init & setup	=============================================================================================
uint8_t INA219_Init(I2C_IRQ_Conn_t *_i2c, INA219_t *dev) {
	dev->status = DEVICE_NOT_INIT;
	uint16_t cfg;
	uint8_t data[INA219_REG_LEN];
	switch (dev->step) {
		case 0:
			cfg = 	INA219_CFG_MODE_SHBV_CONTINUOUS |	// shunt and bus voltage continuous	defaulth
									INA219_CFG_SADC_12BIT_128S 	|	// 128 x 12-bit shunt samples averaged together
									INA219_CFG_BADC_12BIT_128S 	|	// 128 x 12-bit bus samples averaged together
									INA219_CFG_GAIN_8_320MV 		|	// Gain 8, 320mV Range defaulth
									INA219_CFG_BVRANGE_16V;				// 0-16V Range
			data[0] = (uint8_t)cfg;
			data[1] = (uint8_t)(cfg >> 8);
			if (WriteRegBytes(_i2c, dev->addr, INA219_REG_CONFIG, data, INA219_REG_LEN)) {
				dev->step = 1;
			}
			break;
		case 1:
			data[0] = (uint8_t)INA219_CalibrationVal;
			data[1] = (uint8_t)(INA219_CalibrationVal >> 8);
			if (WriteRegBytes(_i2c, dev->addr, INA219_REG_CALIBRATION, data, INA219_REG_LEN)) {
				dev->step = 0;
				dev->status = DEVICE_INIT;
				return 1;
			}
			break;
		default:
			dev->step = 0;
			break;
		}
	return 0;
}

uint8_t INA219_GetRawData(I2C_IRQ_Conn_t *_i2c, INA219_t *dev) {
		uint8_t dt[INA219_REG_LEN];
		uint8_t reg;
		_i2c->addr = dev->addr;
		_i2c->len = INA219_REG_LEN;
		_i2c->mode = I2C_MODE_READ;
		switch (dev->step) {
		case 0://select reg
		case 1:
		case 2:
		case 3:
			switch (dev->step) {
					case 0://select reg
								reg = INA219_REG_BUSVOLTAGE;//read  voltage
								break;
					case 1:
								reg = INA219_REG_POWER;//read power
								break;
					case 2:
								reg = INA219_REG_CURRENT;//read current
								break;
					case 3:
								reg = INA219_REG_SHUNTVOLTAGE;//read shunt voltage
								break;
					default:
						reg = INA219_REG_BUSVOLTAGE;//read voltage
						break;
			}
			if (ReadRegBytes(_i2c, dev->addr, reg, dt, INA219_REG_LEN)) {
				switch (dev->step) {
					case 0://select reg
						dev->raw.voltage = (uint16_t)CONCAT_BYTES(dt[0], dt[1]);//read voltage
						dev->step = 1;
						break;
					case 1:
						dev->raw.power = (uint16_t)CONCAT_BYTES(dt[0], dt[1]);//read power
						dev->step = 2;
						break;
					case 2:
						dev->raw.current = (uint16_t)CONCAT_BYTES(dt[0], dt[1]);//read current
						dev->step = 3;
						break;
					case 3:
						dev->raw.shuntV = (int16_t)CONCAT_BYTES(dt[0], dt[1]);//read shunt voltage
						dev->step = 0;
						break;
					default:
						dev->step = 0;
						break;
					}
				dev->status = DEVICE_DONE;
				return 1;
			}
			break;
		default:
			dev->step = 0;
			break;
		}
	return 0;
}
//Get & conversion raw data	=================================================================================
uint16_t INA219_GetVoltageInt(INA219_t *dev, uint16_t divider) {
	return (uint16_t)(((uint32_t)(dev->raw.voltage >> 3  ) * 4) / divider);
}

float INA219_GetVoltageFloat(INA219_t *dev, uint16_t divider) {
	return ((float)((uint32_t)(dev->raw.voltage >> 3) * 4)) / divider;
}

uint16_t INA219_GetCurrentInt(INA219_t *dev, uint16_t divider) {
	return (uint16_t)(((uint32_t)dev->raw.current * INA219_Current_LSB_mkA) / divider);
}

float INA219_GetCurrentFloat(INA219_t *dev, uint16_t divider) {
	return ((float)((uint32_t)dev->raw.current * INA219_Current_LSB_mkA)) / divider;
}

uint16_t INA219_GetPowerInt(INA219_t *dev, uint16_t divider) {
	return (uint16_t)(((uint32_t)dev->raw.power * INA219_Power_LSB_mkV) / divider);
}

float INA219_GetPowerFloat(INA219_t *dev, uint16_t divider) {
	return (float)(((uint32_t)dev->raw.power * INA219_Power_LSB_mkV)) / divider;
}

//#endif
