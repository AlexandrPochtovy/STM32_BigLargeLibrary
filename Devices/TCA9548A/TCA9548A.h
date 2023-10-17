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

   Created on: Aug 1, 2022
 ********************************************************************************/

#ifndef TCA9548A_H_
#define TCA9548A_H_

#include "I2C_API.h"

 /**********************************************************************
 *                       TYPEDEF & ENUM                                *
 ***********************************************************************/
enum TCA9548A_ADDRESS {
  TCA9548A_ADDR = 0xE0//Assumes ALT address pin low
  };

enum TCA9548A_channel {
	TCA9548A_CH_OFF = 0x00,
  TCA9548A_CH0 = 0x01,
  TCA9548A_CH1 = 0x02,
  TCA9548A_CH2 = 0x04,
  TCA9548A_CH3 = 0x08,
  TCA9548A_CH4 = 0x10,
  TCA9548A_CH5 = 0x20,
  TCA9548A_CH6 = 0x40,
  TCA9548A_CH7 = 0x80
  };

//common data struct for sensor
typedef struct TCA9548A {
  enum TCA9548A_ADDRESS addr;
  volatile DeviceStatus_t status;
  const uint8_t errLimit;
  uint8_t errCount;
  uint8_t port;
  } TCA9548A_t;

/*****************************************************************
  * @brief set i2c multiplexer's port's channels as mask
  * @param _i2c - pointer to I2C bus connection structure
  * @param dev - pointer to multiplexer main structure
  * @param channelMask - mask for enable channels
  * @retval 1 when end
  */
uint8_t TCA9548A_SetChannels(I2C_IRQ_Conn_t *_i2c, TCA9548A_t *dev, uint8_t channelMask);


#endif /* TCA9548A_H_ */
