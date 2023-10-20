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

 * 	MCP23_17.c
 *  Created on: 12 jan 2021
 ********************************************************************************/

#include "MCP23x17.h"

#define MCP23017_CFG_LENGHT 14

uint8_t MCP23_17_Init(I2C_IRQ_Conn_t *_i2c, MCP23_t *dev) {
	switch (dev->status) {
		case DEVICE_READY:
			if (_i2c->status == PORT_FREE) {
				_i2c->status = PORT_BUSY;
				dev->status = DEVICE_PROCESSING;
				}
			break;
		case DEVICE_PROCESSING: {
			uint8_t data[MCP23017_CFG_LENGHT];
			data[0] = MCP23017_IODIR_ALL_OUTPUT;  //reg 0x00 IODIRA 	RW set all pins portA as output
			data[1] = MCP23017_IODIR_ALL_OUTPUT;  //reg 0x01 IODIRB 	RW set all pins portB as input
			data[2] = MCP23017_IPOL_ALL_NORMAL;  //reg 0x02 IPOLA 		RW set all pins portA normal polarity
			data[3] = MCP23017_IPOL_ALL_NORMAL;  //reg 0x03 IPOLB		RW set all pins portB normal polarity
			data[4] = 0x00;  //reg 0x04 GPINTENA	RW disable interrupts for all pins portA
			data[5] = 0x00;  //reg 0x05 GPINTENB	RW disable interrupts for all pins portB
			data[6] = 0x00;  //reg 0x06 DEFVALA	RW default value of port A pins
			data[7] = 0x00;  //reg 0x07 DEFVALB	RW default value of port B pins
			data[8] = 0x00;  //reg 0x08 INTCONA	RW compare value for interrupt port A
			data[9] = 0x00;  //reg 0x09 INTCONB	RW compare value for interrupt port B
			data[10] = 0x00;  //reg 0x0A IOCONA		RW setup byte
			data[11] = 0x00;  //reg 0x0B IOCONB		RW setup byte
			data[12] = 0x00;  //reg 0x0C GPPUA		RW setup pull-up port A pins disabled for output
			data[13] = 0x00;  //reg 0x0D GPPUB		RW setup pull-up port B pins enabled for input
			if (I2C_WriteBytes(_i2c, dev->addr, MCP23017_IODIRA, data, MCP23017_CFG_LENGHT) && (_i2c->status == PORT_BUSY)) {
				dev->status = DEVICE_DONE;
				}
			else if (_i2c->status == PORT_ERROR) {
				dev->status = DEVICE_ERROR;
				}
			break;}
		case DEVICE_DONE:
			dev->status = DEVICE_READY;
			_i2c->status = PORT_FREE;
			return 1;
		case DEVICE_ERROR:
			if (++dev->errCount >= dev->errLimit) {
				dev->status = DEVICE_FAULTH;
				_i2c->status = PORT_FREE;
				}
			else {
				dev->status = DEVICE_READY;
				_i2c->status = PORT_FREE;
				}
			break;
		case DEVICE_FAULTH:
			return 1;
		default:
			break;
		}
	return 0;
	}

uint8_t MCP23_17_ReadPort(I2C_IRQ_Conn_t *_i2c, MCP23_t *dev, uint8_t port, uint8_t *value) {
	switch (dev->status) {
		case DEVICE_READY:
			if (_i2c->status == PORT_FREE) {
				_i2c->status = PORT_BUSY;
				dev->status = DEVICE_PROCESSING;
				}
			break;
		case DEVICE_PROCESSING:
			if (I2C_ReadOneByte(_i2c, dev->addr, port, value) && (_i2c->status == PORT_BUSY)) {
				dev->status = DEVICE_DONE;
				}
			else if (_i2c->status == PORT_ERROR) {
				dev->status = DEVICE_ERROR;
				}
			break;
		case DEVICE_DONE:
			dev->status = DEVICE_READY;
			_i2c->status = PORT_FREE;
			return 1;
		case DEVICE_ERROR:
			if (++dev->errCount >= dev->errLimit) {
				dev->status = DEVICE_FAULTH;
				_i2c->status = PORT_FREE;
				}
			else {
				dev->status = DEVICE_READY;
				_i2c->status = PORT_FREE;
				}
			break;
		case DEVICE_FAULTH:
			return 1;
		default:
			break;
		}
	return 0;
	}

uint8_t MCP23_17_WritePort(I2C_IRQ_Conn_t *_i2c, MCP23_t *dev, uint8_t port, uint8_t value) {
	switch (dev->status) {
		case DEVICE_READY:
			if (_i2c->status == PORT_FREE) {
				_i2c->status = PORT_BUSY;
				dev->status = DEVICE_PROCESSING;
				}
			break;
		case DEVICE_PROCESSING:
			if (I2C_WriteOneByte(_i2c, dev->addr, port, value) && (_i2c->status == PORT_BUSY)) {
				dev->status = DEVICE_DONE;
				}
			else if (_i2c->status == PORT_ERROR) {
				dev->status = DEVICE_ERROR;
				}
			break;
		case DEVICE_DONE:
			dev->status = DEVICE_READY;
			_i2c->status = PORT_FREE;
			return 1;
		case DEVICE_ERROR:
			if (++dev->errCount >= dev->errLimit) {
				dev->status = DEVICE_FAULTH;
				_i2c->status = PORT_FREE;
				}
			else {
				dev->status = DEVICE_READY;
				_i2c->status = PORT_FREE;
				}
			break;
		case DEVICE_FAULTH:
			return 1;
		default:
			break;
		}
	return 0;
	}

uint8_t MCP23_17_ReadAB(I2C_IRQ_Conn_t *_i2c, MCP23_t *dev, uint8_t *value) {
	switch (dev->status) {
		case DEVICE_READY:
			if (_i2c->status == PORT_FREE) {
				_i2c->status = PORT_BUSY;
				dev->status = DEVICE_PROCESSING;
				}
			break;
		case DEVICE_PROCESSING:
			if (I2C_ReadBytes(_i2c, dev->addr, MCP23017_GPIOA, value, 2) && (_i2c->status == PORT_BUSY)) {
				dev->status = DEVICE_DONE;
				}
			else if (_i2c->status == PORT_ERROR) {
				dev->status = DEVICE_ERROR;
				}
			break;
		case DEVICE_DONE:
			dev->status = DEVICE_READY;
			_i2c->status = PORT_FREE;
			return 1;
		case DEVICE_ERROR:
			if (++dev->errCount >= dev->errLimit) {
				dev->status = DEVICE_FAULTH;
				_i2c->status = PORT_FREE;
				}
			else {
				dev->status = DEVICE_READY;
				_i2c->status = PORT_FREE;
				}
			break;
		case DEVICE_FAULTH:
			return 1;
		default:
			break;
		}
	return 0;
	}

uint8_t MCP23_17_WriteAB(I2C_IRQ_Conn_t *_i2c, MCP23_t *dev, uint8_t *value) {
	switch (dev->status) {
		case DEVICE_READY:
			if (_i2c->status == PORT_FREE) {
				_i2c->status = PORT_BUSY;
				dev->status = DEVICE_PROCESSING;
				}
			break;
		case DEVICE_PROCESSING:
			if (I2C_WriteBytes(_i2c, dev->addr, MCP23017_GPIOA, value, 2) && (_i2c->status == PORT_BUSY)) {
				dev->status = DEVICE_DONE;
				}
			else if (_i2c->status == PORT_ERROR) {
				dev->status = DEVICE_ERROR;
				}
			break;
		case DEVICE_DONE:
			dev->status = DEVICE_READY;
			_i2c->status = PORT_FREE;
			return 1;
		case DEVICE_ERROR:
			if (++dev->errCount >= dev->errLimit) {
				dev->status = DEVICE_FAULTH;
				_i2c->status = PORT_FREE;
				}
			else {
				dev->status = DEVICE_READY;
				_i2c->status = PORT_FREE;
				}
			break;
		case DEVICE_FAULTH:
			return 1;
		default:
			break;
		}
	return 0;
	}

uint8_t MCP23_17_Check(I2C_IRQ_Conn_t *_i2c, MCP23_t *dev, uint8_t reg) {
	switch (dev->status) {
		case DEVICE_READY:
			if (_i2c->status == PORT_FREE) {
				_i2c->status = PORT_BUSY;
				dev->status = DEVICE_PROCESSING;
				}
			break;
		case DEVICE_PROCESSING:
			if (I2C_WriteOne(_i2c, dev->addr, MCP23017_GPIOA) && (_i2c->status == PORT_BUSY)) {
				dev->status = DEVICE_DONE;
				}
			else if (_i2c->status == PORT_ERROR) {
				dev->status = DEVICE_ERROR;
				}
			break;
		case DEVICE_DONE:
			dev->status = DEVICE_READY;
			_i2c->status = PORT_FREE;
			return 1;
		case DEVICE_ERROR:
			if (++dev->errCount >= dev->errLimit) {
				dev->status = DEVICE_FAULTH;
				_i2c->status = PORT_FREE;
				}
			else {
				dev->status = DEVICE_READY;
				_i2c->status = PORT_FREE;
				}
			break;
		case DEVICE_FAULTH:
			return 1;
		default:
			break;
		}
	return 0;
	}
