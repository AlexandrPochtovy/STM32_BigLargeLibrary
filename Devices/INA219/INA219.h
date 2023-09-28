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

  INA219.h
  Created on: 27.01.2022
 ********************************************************************************/

#ifndef _INA219_H_
#define _INA219_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "INA219_Register.h"
#include "I2C_API.h"

  /**********************************************************************
  *                       TYPEDEF & ENUM                                *
  ***********************************************************************/
  enum INA219_ADDRESS {
    INA219_ADDR = 0x88
    };

  enum INA219_DIVIDERS {
    div1 = 1,
    div10 = 10,
    div100 = 100,
    div1000 = 1000
    };

  typedef struct INA219_RawData_t {
    int16_t shuntV;
    uint16_t voltage;
    uint16_t current;
    uint16_t power;
    } INA219_RawData;

  struct INA219_data {
    float voltage;
    float current;
    float power;
    };


  typedef struct INA219 {
    const enum INA219_ADDRESS addr;
    DeviceStatus_t status;
    const uint8_t errLimit;
    uint8_t errCount;
    uint8_t step;
    INA219_RawData raw;
    struct INA219_data data;
    } INA219_t;

  /*****************************************************************
    * @brief init power meter: send settings
    * @param _i2c - pointer to I2C bus connection structure
    * @param dev - pointer to power meter main structure
    * @retval 1 when end
    */
  uint8_t INA219_Init(I2C_IRQ_Conn_t *_i2c, INA219_t *dev);

  /*****************************************************************
    * @brief get all power data from meter and store in main structure
    * in RAW format and normalization
    * @param _i2c - pointer to I2C bus connection structure
    * @param dev - pointer to meter main structure
    * @retval 1 when end
    */
  uint8_t INA219_GetData(I2C_IRQ_Conn_t *_i2c, INA219_t *dev);
  //Get & conversion raw data	==================================================================
  uint16_t INA219_GetVoltageInt(INA219_t *dev, uint16_t divider);
  float INA219_GetVoltageFloat(INA219_t *dev, uint16_t divider);
  uint16_t INA219_GetCurrentInt(INA219_t *dev, uint16_t divider);
  float INA219_GetCurrentFloat(INA219_t *dev, uint16_t divider);
  uint16_t INA219_GetPowerInt(INA219_t *dev, uint16_t divider);
  float INA219_GetPowerFloat(INA219_t *dev, uint16_t divider);

#ifdef __cplusplus
  }
#endif

#endif /* INA219_H_ */
