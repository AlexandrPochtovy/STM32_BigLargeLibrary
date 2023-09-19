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
   
 * MCP23x17_Register.h
 * Created on: Jul 26, 2023
 ********************************************************************************/

#ifndef MCP23017_MCP23X17_REGISTER_H_
#define MCP23017_MCP23X17_REGISTER_H_

//	registers map	============================================================================
#define MCP23017_IODIRA 	(uint8_t)0x00
#define MCP23017_IODIRB 	(uint8_t)0x01
#define MCP23017_IPOLA 		(uint8_t)0x02
#define MCP23017_IPOLB 		(uint8_t)0x03
#define MCP23017_GPINTENA (uint8_t)0x04
#define MCP23017_GPINTENB (uint8_t)0x05
#define MCP23017_DEFVALA 	(uint8_t)0x06
#define MCP23017_DEFVALB 	(uint8_t)0x07
#define MCP23017_INTCONA 	(uint8_t)0x08
#define MCP23017_INTCONB 	(uint8_t)0x09
#define MCP23017_IOCONA 	(uint8_t)0x0A
#define MCP23017_IOCONB 	(uint8_t)0x0B
#define MCP23017_GPPUA 		(uint8_t)0x0C
#define MCP23017_GPPUB 		(uint8_t)0x0D
#define MCP23017_INTFA 		(uint8_t)0x0E
#define MCP23017_INTFB 		(uint8_t)0x0F
#define MCP23017_INTCAPA 	(uint8_t)0x10
#define MCP23017_INTCAPB 	(uint8_t)0x11
#define MCP23017_GPIOA 		(uint8_t)0x12
#define MCP23017_GPIOB 		(uint8_t)0x13
#define MCP23017_OLATA		(uint8_t)0x14
#define MCP23017_OLATB		(uint8_t)0x15
// I/O Direction Default state: MCP23017_IODIR_ALL_INPUT	===============================================
#define MCP23017_IODIR_ALL_OUTPUT	(uint8_t)0x00
#define MCP23017_IODIR_ALL_INPUT	(uint8_t)0xFF
#define MCP23017_IODIR_IO0_INPUT	(uint8_t)0x01
#define MCP23017_IODIR_IO1_INPUT	(uint8_t)0x02
#define MCP23017_IODIR_IO2_INPUT	(uint8_t)0x04
#define MCP23017_IODIR_IO3_INPUT	(uint8_t)0x08
#define MCP23017_IODIR_IO4_INPUT	(uint8_t)0x10
#define MCP23017_IODIR_IO5_INPUT	(uint8_t)0x20
#define MCP23017_IODIR_IO6_INPUT	(uint8_t)0x40
#define MCP23017_IODIR_IO7_INPUT	(uint8_t)0x80
// Input Polarity Default state: MCP23017_IPOL_ALL_NORMAL	==============================================
#define MCP23017_IPOL_ALL_NORMAL		(uint8_t)0x00
#define MCP23017_IPOL_ALL_INVERTED	(uint8_t)0xFF
#define MCP23017_IPOL_IO0_INVERTED	(uint8_t)0x01
#define MCP23017_IPOL_IO1_INVERTED	(uint8_t)0x02
#define MCP23017_IPOL_IO2_INVERTED	(uint8_t)0x04
#define MCP23017_IPOL_IO3_INVERTED	(uint8_t)0x08
#define MCP23017_IPOL_IO4_INVERTED	(uint8_t)0x10
#define MCP23017_IPOL_IO5_INVERTED	(uint8_t)0x20
#define MCP23017_IPOL_IO6_INVERTED	(uint8_t)0x40
#define MCP23017_IPOL_IO7_INVERTED	(uint8_t)0x80
// Pull-Up Resistor Default state: MCP23017_GPPU_ALL_DISABLED	=========================================
#define MCP23017_GPPU_ALL_DISABLED	(uint8_t)0x00
#define MCP23017_GPPU_ALL_ENABLED	(uint8_t)0xFF
#define MCP23017_GPPU_IO0_ENABLED	(uint8_t)0x01
#define MCP23017_GPPU_IO1_ENABLED	(uint8_t)0x02
#define MCP23017_GPPU_IO2_ENABLED	(uint8_t)0x04
#define MCP23017_GPPU_IO3_ENABLED	(uint8_t)0x08
#define MCP23017_GPPU_IO4_ENABLED	(uint8_t)0x10
#define MCP23017_GPPU_IO5_ENABLED	(uint8_t)0x20
#define MCP23017_GPPU_IO6_ENABLED	(uint8_t)0x40
#define MCP23017_GPPU_IO7_ENABLED	(uint8_t)0x80
//	Bit definition pins for PORT A	================================================
#define MCP23017_GPA0		(uint8_t)0x01
#define MCP23017_GPA1		(uint8_t)0x02
#define MCP23017_GPA2		(uint8_t)0x04
#define MCP23_17_GPA3		(uint8_t)0x08
#define MCP23017_GPA4		(uint8_t)0x10
#define MCP23017_GPA5		(uint8_t)0x20
#define MCP23017_GPA6		(uint8_t)0x40
#define MCP23017_GPA7		(uint8_t)0x80
//	Bit definition pins for PORT B	================================================
#define MCP23017_GPB0		(uint8_t)0x01
#define MCP23017_GPB1		(uint8_t)0x02
#define MCP23017_GPB2		(uint8_t)0x04
#define MCP23_17_GPB3		(uint8_t)0x08
#define MCP23017_GPB4		(uint8_t)0x10
#define MCP23017_GPB5		(uint8_t)0x20
#define MCP23017_GPB6		(uint8_t)0x40
#define MCP23017_GPB7		(uint8_t)0x80
//======================================================================



#endif /* MCP23017_MCP23X17_REGISTER_H_ */
