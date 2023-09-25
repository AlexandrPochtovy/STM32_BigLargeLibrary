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
   
 * 	ITG3205.h
 *  Created on: Jan 30, 2022
 ********************************************************************************/

#ifndef ITG3205_H_
#define ITG3205_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "math.h"
#include "ITG3205_Register.h"
#include "I2C_API.h"
//TODO DEVICE DO NOt ANSHWER IF RECONFIG OR RECONNECT
/**********************************************************************
*                       TYPEDEF & ENUM                                * 
***********************************************************************/
#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif

enum ITG3205_ADDRESS {
	ITG3205_ADDR = 0xD0,
	ITG3205_ADDR2 = 0xD2
};

typedef struct ITG3205_RAW_t {
	int16_t temp;
	int16_t X;
	int16_t Y;
	int16_t Z;
} ITG3205_RAW;

typedef struct ITG3205_data_t {
	float temp;
	float X;
	float Y;
	float Z;
} ITG3205_data;
//common data struct for sensor
typedef struct ITG3205 {
	const enum ITG3205_ADDRESS addr;
	DeviceStatus_t status;
  uint8_t errCount;
	uint8_t step;
	ITG3205_RAW raw;
	ITG3205_data data;
} ITG3205_t;

/*****************************************************************
  * @brief init gyroscope: send settings
  * @param _i2c - pointer to I2C bus connection structure
  * @param dev - pointer to gyroscope main structure
  * @retval 1 when end
  */
uint8_t ITG3205_Init(I2C_IRQ_Conn_t *_i2c, ITG3205_t *dev);

/*****************************************************************
  * @brief get all axis data from gyroscope and store in main gyroscope structure in RAW format
  * and normalization values
  * @param _i2c - pointer to I2C bus connection structure
  * @param dev - pointer to gyroscope main structure
  * @retval 1 when end
  */
uint8_t ITG3205_GetData(I2C_IRQ_Conn_t *_i2c, ITG3205_t *dev);

#ifdef __cplusplus
}
#endif
#endif /* ITG3205_H_ */
