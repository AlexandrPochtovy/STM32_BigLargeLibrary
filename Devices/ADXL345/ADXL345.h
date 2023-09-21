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

 * 	ADXL345.h
 * 	Created on: 31.01.2022
 */

#ifndef ADXL345_H_
#define ADXL345_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "ADXL345_Register.h"
#include "I2C_API.h"

/**********************************************************************
*                       TYPEDEF & ENUM                                * 
***********************************************************************/
enum ADXL345_ADDRESS {
	ADXL345_ADDR = 0xA6//Assumes ALT address pin low
};

typedef struct ADXL345_RAW {
	int16_t X;
	int16_t Y;
	int16_t Z;
} ADXL345_RAW_t;

typedef struct ADXL345_data {
	float X;
	float Y;
	float Z;
} ADXL345_data_t;

//common data struct for sensor
typedef struct ADXL345 {
	const enum ADXL345_ADDRESS addr;
	DeviceStatus_t status;
	uint8_t step;
	ADXL345_RAW_t raw;
	ADXL345_data_t data;
} ADXL345_t;

/*****************************************************************
  * @brief init gyroscope: send settings
  * @param _i2c - pointer to I2C bus connection structure
  * @param dev - pointer to gyroscope main structure
  * @retval 1 when end
  */
uint8_t ADXL345_Init(I2C_IRQ_Conn_t *_i2c, ADXL345_t *dev);

/*****************************************************************
  * @brief get all axis data from gyroscope and store in main gyroscope structure in RAW format
  * @param _i2c - pointer to I2C bus connection structure
  * @param dev - pointer to gyroscope main structure
  * @retval 1 when end
  */
uint8_t ADXL345_GetData(I2C_IRQ_Conn_t *_i2c, ADXL345_t *dev);

#ifdef __cplusplus
}
#endif
#endif /* ADXL345_H_ */
