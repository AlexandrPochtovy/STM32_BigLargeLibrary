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
   
 * 	QMC5883L.h
 *  Created on: Jan 31, 2022
 ********************************************************************************/

#ifndef QMC5883L_H_
#define QMC5883L_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "QMC5883L_Register.h"
#include "I2C_API.h"

/**********************************************************************
*                       TYPEDEF & ENUM                                * 
***********************************************************************/
enum QMC5883L_ADDRESS {
	QMC5883L_ADDR = 0x1A
};

typedef struct QMC5883L_raw_data_t {
	int16_t X;
	int16_t Y;
	int16_t Z;
} QMC5883L_raw_data;

typedef struct QMC5883L_data_t {
	float X;
	float Y;
	float Z;
} QMC5883L_data;
//common data struct for sensor
typedef struct QMC5883L {
	const enum QMC5883L_ADDRESS addr;
	DeviceStatus_t status;
  uint8_t errCount;
	uint8_t step;
	QMC5883L_raw_data raw;
	QMC5883L_data data;
} QMC5883L_t;

/*****************************************************************
  * @brief init magnetometer: send settings
  * @param _i2c - pointer to I2C bus connection structure
  * @param dev - pointer to magnetometer main structure
  * @retval 1 when end
  */
uint8_t QMC5883L_Init(I2C_IRQ_Conn_t *_i2c, QMC5883L_t *dev);

/*****************************************************************
  * @brief get all axis data from magnetometer and store in main magnetometer structure in RAW format
  * and normalisation axis values
  * @param _i2c - pointer to I2C bus connection structure
  * @param dev - pointer to magnetometer main structure
  * @retval 1 when end
  */
uint8_t QMC5883L_GetData(I2C_IRQ_Conn_t *_i2c, QMC5883L_t *dev);

#ifdef __cplusplus
}
#endif
#endif /* QMC5883L_H_ */
