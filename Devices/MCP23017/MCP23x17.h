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
   
 * 	MCP23_17.h
 *  Created on: 12 jan 2021
 ********************************************************************************/

#ifndef INC_MCP23017_H_
#define INC_MCP23017_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "MCP23x17_Register.h"
#include "I2C_API.h"

/**********************************************************************
*                       TYPEDEF & ENUM                                * 
***********************************************************************/
enum MCP23017_ADDRESS {
	MCP23017_ADDR = 0x40
};

typedef struct MCP23_port {
		uint8_t A;
		uint8_t B;
} MCP23_port_t;

typedef struct MCP23 {
		const enum MCP23017_ADDRESS addr;
    uint8_t errCount;
		DeviceStatus_t status;
		MCP23_port_t data;
} MCP23_t;

/*****************************************************************
  * @brief init MCP IO expander: send settings
  * @param _i2c - pointer to I2C bus connection structure
  * @param dev - pointer to MCP IO expander main structure
  * @retval 1 when end
  */
uint8_t MCP23_17_Init(I2C_IRQ_Conn_t *_i2c, MCP23_t *dev);

/*****************************************************************
  * @brief read IO port value from MCP IO expander
  * @param _i2c - pointer to I2C bus connection structure
  * @param dev - pointer to MCP IO expander main structure
  * @param port - select port A or port B
  * @param value - pointer for store port value
  * @retval 1 when end
  */
uint8_t MCP23_17_ReadPort(I2C_IRQ_Conn_t *_i2c, MCP23_t *dev, uint8_t port, uint8_t *value);

/*****************************************************************
  * @brief write IO port value to MCP IO expander
  * @param _i2c - pointer to I2C bus connection structure
  * @param dev - pointer to MCP IO expander main structure
  * @param port - select port A or port B
  * @param value - value for write
  * @retval 1 when end
  */
uint8_t MCP23_17_WritePort(I2C_IRQ_Conn_t *_i2c, MCP23_t *dev, uint8_t port, uint8_t value);

/*****************************************************************
  * @brief read both IO ports A and B value from MCP IO expander
  * @param _i2c - pointer to I2C bus connection structure
  * @param dev - pointer to MCP IO expander main structure
  * @param value - pointer for store ports value
  * @retval 1 when end
  */
uint8_t MCP23_17_ReadAB(I2C_IRQ_Conn_t *_i2c, MCP23_t *dev, uint8_t *value);

/*****************************************************************
  * @brief write both IO ports A and B value to MCP IO expander
  * @param _i2c - pointer to I2C bus connection structure
  * @param dev - pointer to MCP IO expander main structure
  * @param value - pointer to value for write
  * @retval 1 when end
  */
uint8_t MCP23_17_WriteAB(I2C_IRQ_Conn_t *_i2c, MCP23_t *dev, uint8_t *value);

#ifdef __cplusplus
}
#endif

#endif /* INC_MCP23_17_H_ */
