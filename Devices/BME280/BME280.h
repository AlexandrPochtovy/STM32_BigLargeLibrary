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
   
 	BME280.h
	Created on: 30.05.2021
 ********************************************************************************/

#ifndef _BME280_H_
#define _BME280_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "BME280_Registers.h"
#include "Peripherals/I2C/MyI2C.h"

//===========================================================================================
enum BME280_ADDRESS {
	BME280_ADDR1 = 0xEC,	//address 1 chip 0x76
	BME280_ADDR2 = 0xED		//address 2 chip 0x77
};
/*!
 * @brief Calibration data
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

/*!
 * @brief bme280 sensor structure which comprises of uncompensated temperature,
 * pressure and humidity data
 */
typedef struct bme280_uncomp_data {
	uint32_t pressure;		// un-compensated pressure
	uint32_t temperature;	// un-compensated temperature
	uint32_t humidity; 		// un-compensated humidity
} bme280_uncomp_data_t;

typedef struct bme280_dataInt {
		uint32_t pressure; 		// Compensated pressure
		int32_t temperature;	// Compensated temperature
		uint32_t humidity;		// Compensated humidity
} bme280_dataInt_t;

/*!
 * @brief bme280 sensor structure which comprises of temperature, pressure and
 * humidity data
 */
typedef struct bme280_dataFloat {
		float pressure; 		// Compensated pressure
		float temperature;	// Compensated temperature
		float humidity;			// Compensated humidity
} bme280_dataFloat_t;

//common data struct for sensor
typedef struct bme280_dev {
		const uint8_t addr;
		uint8_t step;
		DeviceStatus_t status;
		bme280_calib_data_t calib_data;
		bme280_uncomp_data_t uncomp_data;
		bme280_dataInt_t data_int;
		bme280_dataFloat_t data_float;
} BME280_t;

//INITIALIZATION	================================================================
uint8_t BME280_Init(I2C_IRQ_Conn_t *_i2c, BME280_t *dev);
uint8_t BME280_GetData(I2C_IRQ_Conn_t *_i2c, BME280_t *dev);
//CALCULATING	==========================================================================
int32_t compensate_temperature_int(BME280_t *dev);
float compensate_temperature_float(BME280_t *dev);
uint32_t compensate_pressure_int(BME280_t *dev);
float compensate_pressure_float(BME280_t *dev);
uint32_t compensate_humidity_int(BME280_t *dev);
float compensate_humidity_float(BME280_t *dev);

void bme280_calculate_data_int(BME280_t *dev);
void bme280_calculate_data_float(BME280_t *dev);

#ifdef __cplusplus
}
#endif
#endif /* BME280_BME_H_ */
