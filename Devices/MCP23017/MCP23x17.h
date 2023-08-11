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
   
 * 	MCP23_17.h
 *  Created on: 12 jan 2021
 ********************************************************************************/

#ifndef INC_MCP23017_H_
#define INC_MCP23017_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "MCP23x17_Register.h"
#include "Peripherals/I2C/MyI2C.h"

enum MCP23017_ADDRESS {
	MCP23017_ADDR = 0x40
};



//=======================================================================
typedef struct MCP23_port {
		uint8_t portA;
		uint8_t portB;
} MCP23_port_t;

typedef struct MCP23 {
		const uint8_t addr;
		DeviceStatus_t status;
		MCP23_port_t data;
} MCP23_t;
//function	==================================================================
uint8_t MCP23_17_Init(I2C_IRQ_Connection_t *_i2c, MCP23_t *dev);
uint8_t MCP23_17_ReadPort(I2C_IRQ_Connection_t *_i2c, MCP23_t *dev, uint8_t port, uint8_t *value);
uint8_t MCP23_17_WritePort(I2C_IRQ_Connection_t *_i2c, MCP23_t *dev, uint8_t port, uint8_t value);


#ifdef __cplusplus
}
#endif

#endif /* INC_MCP23_17_H_ */
