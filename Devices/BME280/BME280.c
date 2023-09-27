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

 BME280.c
 Created on: 30.05.2021
 ********************************************************************************/

#include "BME280.h"

static inline uint16_t CONCAT_BYTES(uint8_t msb, uint8_t lsb) {
	return ( uint16_t )((( uint16_t )msb << 8) | ( uint16_t )lsb);
	}

void bme280_parse_sensor_data(BME280_t* dev, uint8_t* data) {
	/* Variables to store the sensor data */
	uint32_t data_xlsb;
	uint32_t data_lsb;
	uint32_t data_msb;
	/* Store the parsed register values for pressure data */
	data_msb = ( uint32_t )data[0] << 12;
	data_lsb = ( uint32_t )data[1] << 4;
	data_xlsb = ( uint32_t )data[2] >> 4;
	dev->uncomp_data.pressure = data_msb | data_lsb | data_xlsb;
	/* Store the parsed register values for temperature data */
	data_msb = ( uint32_t )data[3] << 12;
	data_lsb = ( uint32_t )data[4] << 4;
	data_xlsb = ( uint32_t )data[5] >> 4;
	dev->uncomp_data.temperature = data_msb | data_lsb | data_xlsb;
	/* Store the parsed register values for humidity data */
	data_msb = ( uint32_t )data[6] << 8;
	data_lsb = ( uint32_t )data[7];
	dev->uncomp_data.humidity = data_msb | data_lsb;
	}

void parse_temp_press_calib_data(BME280_t* dev, uint8_t* data) {
	dev->calib_data.dig_t1 = CONCAT_BYTES(data[1], data[0]);
	dev->calib_data.dig_t2 = ( int16_t )CONCAT_BYTES(data[3], data[2]);
	dev->calib_data.dig_t3 = ( int16_t )CONCAT_BYTES(data[5], data[4]);
	dev->calib_data.dig_p1 = CONCAT_BYTES(data[7], data[6]);
	dev->calib_data.dig_p2 = ( int16_t )CONCAT_BYTES(data[9], data[8]);
	dev->calib_data.dig_p3 = ( int16_t )CONCAT_BYTES(data[11], data[10]);
	dev->calib_data.dig_p4 = ( int16_t )CONCAT_BYTES(data[13], data[12]);
	dev->calib_data.dig_p5 = ( int16_t )CONCAT_BYTES(data[15], data[14]);
	dev->calib_data.dig_p6 = ( int16_t )CONCAT_BYTES(data[17], data[16]);
	dev->calib_data.dig_p7 = ( int16_t )CONCAT_BYTES(data[19], data[18]);
	dev->calib_data.dig_p8 = ( int16_t )CONCAT_BYTES(data[21], data[20]);
	dev->calib_data.dig_p9 = ( int16_t )CONCAT_BYTES(data[23], data[22]);
	dev->calib_data.dig_h1 = data[25];
	}

void parse_humidity_calib_data(BME280_t* dev, uint8_t* data) {
	int16_t dig_h4_lsb;
	int16_t dig_h4_msb;
	int16_t dig_h5_lsb;
	int16_t dig_h5_msb;
	dev->calib_data.dig_h2 = ( int16_t )CONCAT_BYTES(data[1], data[0]);
	dev->calib_data.dig_h3 = data[2];
	dig_h4_msb = ( int16_t )( int8_t )data[3] * 16;
	dig_h4_lsb = ( int16_t )(data[4] & 0x0F);
	dev->calib_data.dig_h4 = dig_h4_msb | dig_h4_lsb;
	dig_h5_msb = ( int16_t )( int8_t )data[5] * 16;
	dig_h5_lsb = ( int16_t )(data[4] >> 4);
	dev->calib_data.dig_h5 = dig_h5_msb | dig_h5_lsb;
	dev->calib_data.dig_h6 = ( int8_t )data[6];
	}

int32_t compensate_temperature_int(BME280_t* dev) {
	int32_t var1;
	int32_t var2;
	int32_t temperature;
	int32_t temperature_min = -4000;
	int32_t temperature_max = 8500;

	var1 = ( int32_t )((dev->uncomp_data.temperature / 8) - (( int32_t )dev->calib_data.dig_t1 * 2));
	var1 = (var1 * (( int32_t )dev->calib_data.dig_t2)) / 2048;
	var2 = ( int32_t )((dev->uncomp_data.temperature / 16) - (( int32_t )dev->calib_data.dig_t1));
	var2 = (((var2 * var2) / 4096) * (( int32_t )dev->calib_data.dig_t3)) / 16384;
	dev->calib_data.t_fine = var1 + var2;
	temperature = (dev->calib_data.t_fine * 5 + 128) / 256;
	if (temperature < temperature_min) {
		temperature = temperature_min;
		}
	else if (temperature > temperature_max) {
		temperature = temperature_max;
		}
	return temperature;
	}

float compensate_temperature_float(BME280_t* dev) {
	float var1;
	float var2;
	float temperature;
	float temperature_min = -40;
	float temperature_max = 85;

	var1 = (( float )dev->uncomp_data.temperature) / 16384.0 - (( float )dev->calib_data.dig_t1) / 1024.0;
	var1 = var1 * (( float )dev->calib_data.dig_t2);
	var2 = ((( float )dev->uncomp_data.temperature) / 131072.0 - (( float )dev->calib_data.dig_t1) / 8192.0);
	var2 = (var2 * var2) * (( double )dev->calib_data.dig_t3);
	dev->calib_data.t_fine = ( int32_t )(var1 + var2);
	temperature = (var1 + var2) / 5120.0;

	if (temperature < temperature_min) {
		temperature = temperature_min;
		}
	else if (temperature > temperature_max) {
		temperature = temperature_max;
		}
	return temperature;
	}

uint32_t compensate_pressure_int(BME280_t* dev) {
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	uint32_t var5;
	uint32_t pressure;
	uint32_t pressure_min = 30000;
	uint32_t pressure_max = 110000;

	var1 = ((( int32_t )dev->calib_data.t_fine) / 2) - ( int32_t )64000;
	var2 = (((var1 / 4) * (var1 / 4)) / 2048) * (( int32_t )dev->calib_data.dig_p6);
	var2 = var2 + ((var1 * (( int32_t )dev->calib_data.dig_p5)) * 2);
	var2 = (var2 / 4) + ((( int32_t )dev->calib_data.dig_p4) * 65536);
	var3 = (dev->calib_data.dig_p3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
	var4 = ((( int32_t )dev->calib_data.dig_p2) * var1) / 2;
	var1 = (var3 + var4) / 262144;
	var1 = (((32768 + var1)) * (( int32_t )dev->calib_data.dig_p1)) / 32768;
	/* avoid exception caused by division by zero */
	if (var1) {
		var5 = ( uint32_t )(( uint32_t )1048576) - dev->uncomp_data.pressure;
		pressure = (( uint32_t )(var5 - ( uint32_t )(var2 / 4096))) * 3125;
		if (pressure < 0x80000000) {
			pressure = (pressure << 1) / (( uint32_t )var1);
			}
		else {
			pressure = (pressure / ( uint32_t )var1) * 2;
			}
		var1 = ((( int32_t )dev->calib_data.dig_p9) * (( int32_t )(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
		var2 = ((( int32_t )(pressure / 4)) * (( int32_t )dev->calib_data.dig_p8)) / 8192;
		pressure = ( uint32_t )(( int32_t )pressure + ((var1 + var2 + dev->calib_data.dig_p7) / 16));
		if (pressure < pressure_min) {
			pressure = pressure_min;
			}
		else if (pressure > pressure_max) {
			pressure = pressure_max;
			}
		}
	else {
		pressure = pressure_min;
		}
	return pressure;
	}

float compensate_pressure_float(BME280_t* dev) {
	float var1;
	float var2;
	float var3;
	float pressure;
	float pressure_min = 30000.0;
	float pressure_max = 110000.0;

	var1 = (( float )dev->calib_data.t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * (( float )dev->calib_data.dig_p6) / 32768.0;
	var2 = var2 + var1 * (( float )dev->calib_data.dig_p5) * 2.0;
	var2 = (var2 / 4.0) + ((( float )dev->calib_data.dig_p4) * 65536.0);
	var3 = (( float )dev->calib_data.dig_p3) * var1 * var1 / 524288.0;
	var1 = (var3 + (( float )dev->calib_data.dig_p2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * (( float )dev->calib_data.dig_p1);

	/* avoid exception caused by division by zero */
	if (var1 > (0.0)) {
		pressure = 1048576.0 - ( float )dev->uncomp_data.pressure;
		pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
		var1 = (( float )dev->calib_data.dig_p9) * pressure * pressure / 2147483648.0;
		var2 = pressure * (( float )dev->calib_data.dig_p8) / 32768.0;
		pressure = pressure + (var1 + var2 + (( float )dev->calib_data.dig_p7)) / 16.0;
		if (pressure < pressure_min) {
			pressure = pressure_min;
			}
		else if (pressure > pressure_max) {
			pressure = pressure_max;
			}
		}
	else /* Invalid case */
		{
		pressure = pressure_min;
		}
	return pressure;
	}

uint32_t compensate_humidity_int(BME280_t* dev) {
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	uint32_t humidity;
	uint32_t humidity_max = 102400;

	var1 = dev->calib_data.t_fine - (( int32_t )76800);
	var2 = ( int32_t )(dev->uncomp_data.humidity * 16384);
	var3 = ( int32_t )((( int32_t )dev->calib_data.dig_h4) * 1048576);
	var4 = (( int32_t )dev->calib_data.dig_h5) * var1;
	var5 = (((var2 - var3) - var4) + ( int32_t )16384) / 32768;
	var2 = (var1 * (( int32_t )dev->calib_data.dig_h6)) / 1024;
	var3 = (var1 * (( int32_t )dev->calib_data.dig_h3)) / 2048;
	var4 = ((var2 * (var3 + ( int32_t )32768)) / 1024) + ( int32_t )2097152;
	var2 = ((var4 * (( int32_t )dev->calib_data.dig_h2)) + 8192) / 16384;
	var3 = var5 * var2;
	var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
	var5 = var3 - ((var4 * (( int32_t )dev->calib_data.dig_h1)) / 16);
	var5 = (var5 < 0 ? 0 : var5);
	var5 = (var5 > 419430400 ? 419430400 : var5);
	humidity = ( uint32_t )(var5 / 4096);
	if (humidity > humidity_max) {
		humidity = humidity_max;
		}
	return humidity;
	}

float compensate_humidity_float(BME280_t* dev) {
	float humidity;
	float humidity_min = 0.0;
	float humidity_max = 100.0;
	float var1;
	float var2;
	float var3;
	float var4;
	float var5;
	float var6;

	var1 = (( float )dev->calib_data.t_fine) - 76800.0;
	var2 = ((( float )dev->calib_data.dig_h4) * 64.0 + ((( float )dev->calib_data.dig_h5) / 16384.0) * var1);
	var3 = dev->uncomp_data.humidity - var2;
	var4 = (( float )dev->calib_data.dig_h2) / 65536.0;
	var5 = (1.0 + ((( float )dev->calib_data.dig_h3) / 67108864.0) * var1);
	var6 = 1.0 + ((( float )dev->calib_data.dig_h6) / 67108864.0) * var1 * var5;
	var6 = var3 * var4 * (var5 * var6);
	humidity = var6 * (1.0 - (( float )dev->calib_data.dig_h1) * var6 / 524288.0);
	if (humidity > humidity_max) {
		humidity = humidity_max;
		}
	else if (humidity < humidity_min) {
		humidity = humidity_min;
		}
	return humidity;
	}

uint8_t BME280_Init(I2C_IRQ_Conn_t* port, BME280_t* dev) {
	PortStatus_t st;
	if (dev->status == DEVICE_FAULTH) {
		return 1;
		}
	else if ((dev->status == DEVICE_READY) && (port->status == PORT_FREE)) {
		port->status = PORT_BUSY;
		dev->status = DEVICE_PROCESSING;
		dev->step = 0;
		}
	uint8_t data[BME280_T_P_CALIB_DATA_LEN];
	switch (dev->step) {
		case 0: // setup humidity
			st = I2C_WriteOneByte(port, dev->addr, BME280_REG_CTRL_HUM, BME280_HUM_OVERSAMPLING_16X);
			if (st == PORT_DONE) {
				port->status = PORT_BUSY;
				dev->step = 1;
				}
			break;
		case 1: // setup mode temp pressure
			data[0] = BME280_NORMAL_MODE | BME280_PRESS_OVERSAMPLING_16X | BME280_TEMP_OVERSAMPLING_16X;
			data[1] = BME280_SPI_3WIRE_MODE_OFF | BME280_FILTER_COEFF_16 | BME280_STANDBY_TIME_20_MS;
			st = I2C_WriteBytes(port, dev->addr, BME280_REG_CTRL_MEAS_PWR, data, 2);
			if (st == PORT_DONE) {
				port->status = PORT_BUSY;
				dev->step = 2;
				}
			break;
		case 2: // read calib temp data
			st = I2C_ReadBytes(port, dev->addr, BME280_REG_T_P_CALIB_DATA, data, BME280_T_P_CALIB_DATA_LEN);
			if (st == PORT_DONE) {
				parse_temp_press_calib_data(dev, data);
				port->status = PORT_BUSY;
				dev->step = 3;
				}
			break;
		case 3: // read calib pressure data
			st = I2C_ReadBytes(port, dev->addr, BME280_REG_HUM_CALIB_DATA, data, BME280_HUM_CALIB_DATA_LEN);
			if (st == PORT_DONE) {
				parse_humidity_calib_data(dev, data);
				dev->status = DEVICE_DONE;
				}
			break;
		default:
			break;
		}
	if (st == PORT_ERROR) {
		++dev->errCount;
		port->status = PORT_FREE;
		return 1;
		}
	if (dev->status == DEVICE_DONE) {
		port->status = PORT_FREE;
		dev->status = DEVICE_READY;
		return 1;
		}
	return 0;
	}

uint8_t BME280_GetData(I2C_IRQ_Conn_t* port, BME280_t* dev) {
	PortStatus_t st;
	switch (dev->status) {
		case DEVICE_FAULTH:
			return 1;
		case DEVICE_READY:
			if (port->status == PORT_FREE) {
				port->status = PORT_BUSY;
				dev->status = DEVICE_PROCESSING;
				}
			break;
		case DEVICE_PROCESSING:
			{
			uint8_t data[BME280_DATA_LEN];
			st = I2C_ReadBytes(port, dev->addr, BME280_REG_DATA, data, BME280_DATA_LEN);
			if (st == PORT_DONE) {
				bme280_parse_sensor_data(dev, data);
				dev->data_int.temperature = compensate_temperature_int(dev);		 /* Compensate the temperature data */
				dev->data_int.pressure = compensate_pressure_int(dev);					 /* Compensate the pressure data */
				dev->data_int.humidity = compensate_humidity_int(dev);					 /* Compensate the humidity data */
				dev->data_float.temperature = compensate_temperature_float(dev); /* Compensate the temperature data */
				dev->data_float.pressure = compensate_pressure_float(dev);			 /* Compensate the pressure data */
				dev->data_float.humidity = compensate_humidity_float(dev);			 /* Compensate the humidity data */
				dev->status = DEVICE_DONE;
				}
			else if (st == PORT_ERROR) {
				dev->status = DEVICE_ERROR;
				}
			break;
			}
		case DEVICE_DONE:
			port->status = PORT_FREE;
			dev->status = DEVICE_READY;
			return 1;
		case DEVICE_ERROR:
			dev->status = ++dev->errCount < dev->errLimit ? DEVICE_READY : DEVICE_FAULTH;
			break;
		default:
			break;
		}
	return 0;
	}
