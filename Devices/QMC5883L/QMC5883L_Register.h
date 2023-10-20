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

 * 	QMC5883L_Register.h
 *  Created on: Jan 31, 2022
 ********************************************************************************/

#ifndef QMC5883L_REGISTER_H_
#define QMC5883L_REGISTER_H_

enum QMC5883L_Registers {
	QMC5883L_REG_OUT_X_L = 0x00,
	QMC5883L_REG_OUT_X_M = 0x01,
	QMC5883L_REG_OUT_Y_L = 0x02,
	QMC5883L_REG_OUT_Y_M = 0x03,
	QMC5883L_REG_OUT_Z_L = 0x04,
	QMC5883L_REG_OUT_Z_M = 0x05,
	QMC5883L_REG_STATUS = 0x06,
	QMC5883L_REG_TEMP_L = 0x07,
	QMC5883L_REG_TEMP_M = 0x08,
	QMC5883L_REG_CFG_A = 0x09,
	QMC5883L_REG_CFG_B = 0x0A,
	QMC5883L_REG_PERIOD = 0x0B,
	};

enum QMC5883L_STATUS {
	QMC5883L_STATUS_DRDY = 0x01,
	QMC5883L_STATUS_OVL = 0x02,
	QMC5883L_STATUS_DOR = 0x04
	};

enum QMC5883L_REG_CFG_A_MODE {
	QMC5883L_MODE_STANDBY = 0x00,
	QMC5883L_MODE_CONTINUOUS = 0x01,
	};

enum QMC5883L_REG_CFG_A_ODR {
	QMC5883L_ODR_10HZ = 0x00,
	QMC5883L_ODR_50HZ = 0x04,
	QMC5883L_ODR_100HZ = 0x08,
	QMC5883L_ODR_200HZ = 0x0C
	};

enum QMC5883L_REG_CFG_A_RNG {
	QMC5883L_RNG_2G = 0x00,
	QMC5883L_RNG_8G = 0x10
	};

enum QMC5883L_REG_CFG_A_OSR {
	QMC5883L_SAMPLES_512 = 0x00,
	QMC5883L_SAMPLES_256 = 0x40,
	QMC5883L_SAMPLES_128 = 0x80,
	QMC5883L_SAMPLES_64 = 0xC0
	};

enum QMC5883L_REG_CFG_B_CTRL {
	QMC5883L_INT_EN = 0x01,
	QMC5883L_ROL_PNT = 0x40,
	QMC5883L_RESET = 0x80
	};

#endif /* QMC5883L_REGISTER_H_ */
