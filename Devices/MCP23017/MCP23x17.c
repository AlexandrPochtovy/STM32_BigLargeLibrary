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

 * 	MCP23_17.c
 *  Created on: 12 jan 2021
 ********************************************************************************/

#include "MCP23x17.h"

static const uint8_t MCP23017_CFG_LENGHT = 22;
static const uint8_t MCP23017_PORT_LENGHT = 1;

uint8_t MCP23_17_Init(I2C_IRQ_Connection_t *_i2c, MCP23_t *dev) {
	if (_i2c->status == PORT_FREE) {
		switch (_i2c->step) {
		case 0:
			dev->status = DEVICE_NOT_INIT;
			_i2c->addr = dev->addr;
			FIFO_PutOne(_i2c->buffer, MCP23017_IODIRA);  //_i2c->reg = MCP23017_IODIRA;
			FIFO_PutOne(_i2c->buffer, MCP23017_IODIR_ALL_INPUT);  //reg 0x00 IODIRA 	RW set all pins portA as output
			FIFO_PutOne(_i2c->buffer, MCP23017_IODIR_ALL_OUTPUT);
			//reg 0x01 IODIRB 	RW set all pins portB as input
			FIFO_PutOne(_i2c->buffer, MCP23017_IPOL_ALL_NORMAL);  //reg 0x02 IPOLA 		RW set all pins portA normal polarity
			FIFO_PutOne(_i2c->buffer, MCP23017_IPOL_ALL_NORMAL);  //reg 0x03 IPOLB		RW set all pins portB normal polarity
			FIFO_PutOne(_i2c->buffer, 0xF0);  //reg 0x04 GPINTENA	RW disable interrupts for all pins portA
			FIFO_PutOne(_i2c->buffer, 0x0F);  //reg 0x05 GPINTENA	RW disable interrupts for all pins portB
			FIFO_PutOne(_i2c->buffer, 0x00);  //reg 0x06 DEFVALA	RW default value of port A pins
			FIFO_PutOne(_i2c->buffer, 0x00);  //reg 0x07 DEFVALB	RW default value of port B pins
			FIFO_PutOne(_i2c->buffer, 0x00);  //reg 0x08 INTCONA	RW compare value for interrupt port A
			FIFO_PutOne(_i2c->buffer, 0x00);  //reg 0x09 INTCONB	RW compare value for interrupt port B
			FIFO_PutOne(_i2c->buffer, 0x00);  //reg 0x0A IOCONA		RW setup byte
			FIFO_PutOne(_i2c->buffer, 0x00);  //reg 0x0B IOCONB		RW setup byte
			FIFO_PutOne(_i2c->buffer, 0xFF);  //reg 0x0C GPPUA		RW setup pull-up port A pins disabled for output
			FIFO_PutOne(_i2c->buffer, 0xFF);  //reg 0x0D GPPUB		RW setup pull-up port B pins enabled for input
			FIFO_PutOne(_i2c->buffer, 0xDE);  //reg 0x0E INTFA		R interrupt flag register port A
			FIFO_PutOne(_i2c->buffer, 0xAD);  //reg 0x0F INTFB		R interrupt flag register port B
			FIFO_PutOne(_i2c->buffer, 0xBE);  //reg 0x10 INTCAPA	R interrupt flag register port A
			FIFO_PutOne(_i2c->buffer, 0xAF);  //reg 0x11 INTCAPB	R interrupt flag register port B
			FIFO_PutOne(_i2c->buffer, 0xDE);  //reg 0x12 GPIOA		RW main port data register port A
			FIFO_PutOne(_i2c->buffer, 0xAD);  //reg 0x13 GPIOB		RW main port data register port B
			FIFO_PutOne(_i2c->buffer, 0xBE);  //reg 0x14 OLATA		RW output port data register port A
			FIFO_PutOne(_i2c->buffer, 0xAF);  //reg 0x15 OLATB		RW output port data register port B;
			_i2c->len = MCP23017_CFG_LENGHT;
			_i2c->mode = I2C_MODE_WRITE;
			_i2c->step = 1;
			I2C_Start_IRQ(_i2c);
			break;
		case 1:
			dev->status = DEVICE_INIT;
			_i2c->step = 0;
			return 1;
			break;
		default:
			_i2c->step = 0;
			break;
		}
	}
	return 0;
}

uint8_t MCP23_17_Write(I2C_IRQ_Connection_t *_i2c, MCP23_t *dev, uint8_t reg, uint8_t *data,
    uint8_t len) {
	if (_i2c->status == PORT_FREE) {
		switch (_i2c->step) {
		case 0:
			_i2c->addr = dev->addr;
			FIFO_PutOne(_i2c->buffer, reg);
			FIFO_PutMulti(_i2c->buffer, data, len);
			_i2c->len = len;
			_i2c->mode = I2C_MODE_WRITE;
			_i2c->step = 1;
			I2C_Start_IRQ(_i2c);
			break;
		case 1:
			dev->status = DEVICE_DONE;
			_i2c->step = 0;
			return 1;
		default:
			_i2c->step = 0;
			break;
		}
	}
	return 0;
}

uint8_t MCP23_17_Read(I2C_IRQ_Connection_t *_i2c, MCP23_t *dev, uint8_t reg, uint8_t *data,
    uint8_t len) {
	if (_i2c->status == PORT_FREE) {
		switch (_i2c->step) {
		case 0:
			_i2c->addr = dev->addr;
			FIFO_PutOne(_i2c->buffer, reg);
			_i2c->len = len;
			_i2c->mode = I2C_MODE_READ;
			_i2c->step = 1;
			I2C_Start_IRQ(_i2c);
			break;
		case 1:
			FIFO_GetMulti(_i2c->buffer, data, len);
			dev->status = DEVICE_DONE;
			_i2c->step = 0;
			return 1;
		default:
			_i2c->step = 0;
			break;
		}
	}
	return 0;
}

uint8_t MCP23_17_ReadPortA(I2C_IRQ_Connection_t *_i2c, MCP23_t *dev) {
	if (_i2c->status == PORT_FREE) {
		switch (_i2c->step) {
		case 0:
			_i2c->addr = dev->addr;
			FIFO_PutOne(_i2c->buffer, MCP23017_GPIOA);
			_i2c->len = MCP23017_PORT_LENGHT;
			_i2c->step = 1;
			_i2c->mode = I2C_MODE_READ;
			I2C_Start_IRQ(_i2c);
			break;
		case 1:
			FIFO_GetOne(_i2c->buffer, &dev->data.portA);
			dev->status = DEVICE_DONE;
			_i2c->step = 0;
			return 1;
			break;
		default:
			_i2c->step = 0;
			break;
		}
	}
	return 0;
}

uint8_t MCP23_17_WritePortA(I2C_IRQ_Connection_t *_i2c, MCP23_t *dev, uint8_t value) {
	if (_i2c->status == PORT_FREE) {
		switch (_i2c->step) {
		case 0:
			_i2c->addr = dev->addr;
			FIFO_PutOne(_i2c->buffer, MCP23017_OLATA);
			FIFO_PutOne(_i2c->buffer, value);
			_i2c->len = MCP23017_PORT_LENGHT;
			_i2c->mode = I2C_MODE_WRITE;
			_i2c->step = 1;
			I2C_Start_IRQ(_i2c);
			break;
		case 1:
			dev->status = DEVICE_DONE;
			_i2c->step = 0;
			return 1;
		default:
			_i2c->step = 0;
			break;
		}
	}
	return 0;
}

uint8_t MCP23_17_ReadPortB(I2C_IRQ_Connection_t *_i2c, MCP23_t *dev) {
	if (_i2c->status == PORT_FREE) {
		switch (_i2c->step) {
		case 0:
			_i2c->addr = dev->addr;
			FIFO_PutOne(_i2c->buffer, MCP23017_GPIOB);
			_i2c->len = MCP23017_PORT_LENGHT;
			_i2c->mode = I2C_MODE_READ;
			_i2c->step = 1;
			I2C_Start_IRQ(_i2c);
			break;
		case 1:
			FIFO_GetOne(_i2c->buffer, &dev->data.portB);
			dev->status = DEVICE_DONE;
			_i2c->step = 0;
			return 1;
			break;
		default:
			_i2c->step = 0;
			break;
		}
	}
	return 0;
}

uint8_t MCP23_17_WritePortB(I2C_IRQ_Connection_t *_i2c, MCP23_t *dev, uint8_t value) {
	if (_i2c->status == PORT_FREE) {
		switch (_i2c->step) {
		case 0:
			_i2c->addr = dev->addr;
			FIFO_PutOne(_i2c->buffer, MCP23017_OLATB);
			FIFO_PutOne(_i2c->buffer, value);
			_i2c->len = MCP23017_PORT_LENGHT;
			_i2c->mode = I2C_MODE_WRITE;
			_i2c->step = 1;
			I2C_Start_IRQ(_i2c);
			break;
		case 1:
			dev->status = DEVICE_DONE;
			_i2c->step = 0;
			return 1;
		default:
			_i2c->step = 0;
			break;
		}
	}
	return 0;
}
