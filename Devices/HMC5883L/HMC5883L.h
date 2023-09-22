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
   
 	HMC5883L.h
	Created on: 31.01.2022
 ********************************************************************************/

#ifndef _HMC5883L_H_
#define _HMC5883L_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "HMC5883L_Register.h"
#include "I2C_API.h"

/**********************************************************************
*                       TYPEDEF & ENUM                                * 
***********************************************************************/
enum HMC5883L_ADDRESS {
	HMC5883L_ADDR = 0x3C
};

typedef struct HMC5883L_raw_data_t {
	uint16_t X;
	uint16_t Y;
	uint16_t Z;
	uint8_t status;
	uint32_t ID;
} HMC5883L_raw_data;

typedef struct HMC5883L_data_t {
	float X;
	float Y;
	float Z;
} HMC5883L_data;
//common data struct for sensor
typedef struct HMC5883L_dev_t {
	const enum HMC5883L_ADDRESS addr;
	DeviceStatus_t status;
	uint8_t step;
	HMC5883L_raw_data raw;
	HMC5883L_data data;
} HMC5883L_dev;

/*****************************************************************
  * @brief init magnetometer: send settings
  * @param _i2c - pointer to I2C bus connection structure
  * @param dev - pointer to magnetometer main structure
  * @retval 1 when end
  */
uint8_t HMC5883L_Init(I2C_IRQ_Conn_t *_i2c, HMC5883L_dev *dev);

/*****************************************************************
  * @brief get all axis data from magnetometer and store in main magnetometer structure in RAW format
  * and normalisation axis values
  * @param _i2c - pointer to I2C bus connection structure
  * @param dev - pointer to magnetometer main structure
  * @retval 1 when end
  */
uint8_t HMC5883L_GetData(I2C_IRQ_Conn_t *_i2c, HMC5883L_dev *dev);

#ifdef __cplusplus
}
#endif
#endif /* HMC5883L_H_ */
