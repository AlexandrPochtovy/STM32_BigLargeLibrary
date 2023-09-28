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

	BME280.h
	Created on: 30.05.2021
 ********************************************************************************/

#ifndef _BME280_H_
#define _BME280_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "BME280_Registers.h"
#include "I2C_API.h"

	/**********************************************************************
	*                       TYPEDEF & ENUM                                *
	***********************************************************************/
	enum BME280_ADDRESS {
		BME280_ADDR1 = (uint8_t)0xEC,	//address 1 chip 0x76 shifted
		BME280_ADDR2 = (uint8_t)0xED	//address 2 chip 0x77 shifted
		};

	/*****************************************************************
		* @brief bme280 temperature's calibration coefficients, must be read and store for
		* temperature calculating
		*/
	typedef struct bme280_calib_data {
		uint16_t dig_t1;	// Calibration coefficient for the temperature sensor
		int16_t dig_t2; 	// Calibration coefficient for the temperature sensor
		int16_t dig_t3;		// Calibration coefficient for the temperature sensor
		uint16_t dig_p1;	// Calibration coefficient for the pressure sensor
		int16_t dig_p2;		// Calibration coefficient for the pressure sensor
		int16_t dig_p3;		// Calibration coefficient for the pressure sensor
		int16_t dig_p4;		// Calibration coefficient for the pressure sensor
		int16_t dig_p5;		// Calibration coefficient for the pressure sensor
		int16_t dig_p6;		// Calibration coefficient for the pressure sensor
		int16_t dig_p7;		// Calibration coefficient for the pressure sensor
		int16_t dig_p8;		// Calibration coefficient for the pressure sensor
		int16_t dig_p9;		// Calibration coefficient for the pressure sensor
		uint8_t dig_h1;		// Calibration coefficient for the humidity sensor
		int16_t dig_h2;		// Calibration coefficient for the humidity sensor
		uint8_t dig_h3;		// Calibration coefficient for the humidity sensor
		int16_t dig_h4;		// Calibration coefficient for the humidity sensor
		int16_t dig_h5;		// Calibration coefficient for the humidity sensor
		int8_t dig_h6;		// Calibration coefficient for the humidity sensor
		int32_t t_fine;		// Variable to store the intermediate temperature coefficient
		} bme280_calib_data_t;

	/*****************************************************************
	 * @brief bme280 sensor structure of uncompensated temperature, pressure and humidity data
	 */
	typedef struct bme280_uncomp_data {
		uint32_t pressure;		// un-compensated pressure
		uint32_t temperature;	// un-compensated temperature
		uint32_t humidity; 		// un-compensated humidity
		} bme280_uncomp_data_t;

	/*****************************************************************
	 * @brief bme280 sensor structure of compensated temperature, pressure
	 * and humidity data in integer datatype
	 */
	typedef struct bme280_dataInt {
		uint32_t pressure; 		// Compensated pressure in Pa
		int32_t temperature;	// Compensated temperature
		uint32_t humidity;		// Compensated humidity in xxx.xx %
		} bme280_dataInt_t;

	/*****************************************************************
	 * @brief bme280 sensor structure of compensated temperature, pressure
	 * and humidity data in float datatype
	 */
	typedef struct bme280_dataFloat {
		float pressure; 		// Compensated pressure
		float temperature;		// Compensated temperature
		float humidity;			// Compensated humidity
		} bme280_dataFloat_t;

	/*****************************************************************
	 * @brief bme280 sensor main structure: store calibration data, uncompensated data,
	 * compensated data
	 */
	typedef struct bme280_dev {
		const enum BME280_ADDRESS addr;
		DeviceStatus_t status;
		const uint8_t errLimit;
		uint8_t errCount;
		uint8_t step;
		bme280_calib_data_t calib_data;
		bme280_uncomp_data_t uncomp_data;
		bme280_dataInt_t data_int;
		bme280_dataFloat_t data_float;
		} BME280_t;

	/*****************************************************************
		* @brief init bme280: send settings, read calibration data
		* @param _i2c - pointer to I2C bus connection structure
		* @param dev - pointer to bme280 main structure
		* @retval 1 when end
		*/
	uint8_t BME280_Init(I2C_IRQ_Conn_t *_i2c, BME280_t *dev);

	/*****************************************************************
		* @brief get data from bme280 receives data in raw format, converts it to normal values
		* @param _i2c - pointer to I2C bus connection structure
		* @param dev - pointer to bme280 main structure
		* @retval 1 when end
		*/
	uint8_t BME280_GetData(I2C_IRQ_Conn_t *_i2c, BME280_t *dev);

#ifdef __cplusplus
	}
#endif
#endif /* BME280_BME_H_ */
