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
   
 * I2C_API.c
 * Created on: Aug 30, 2023
 ********************************************************************************/

#include "I2C_API.h"

uint8_t I2C_WriteOneByte(I2C_IRQ_Conn_t *_i2c, uint8_t addr, uint8_t reg, uint8_t value) {
	switch (_i2c->status) {
	case PORT_FREE:
		_i2c->status = PORT_BUSY;
		_i2c->addr = addr;
		FIFO_PutOne(_i2c->buffer, reg);
		FIFO_PutOne(_i2c->buffer, value);
		_i2c->len = 1;
		_i2c->mode = I2C_MODE_WRITE;
		I2C_Start_IRQ(_i2c);
		return 0;
		break;
	case PORT_DONE:
		_i2c->buffer->lockState = BUFFER_FREE;
		_i2c->status = PORT_FREE;
		return 1;// exit
		break;
	case PORT_ERROR://TODO ADD error processing
		_i2c->buffer->lockState = BUFFER_FREE;
		FIFO_Init(_i2c->buffer);
		_i2c->status = PORT_FREE;
		return 1;
		break;
	default:
		return 0;
		break;
	}
}

uint8_t I2C_WriteBytes(I2C_IRQ_Conn_t *_i2c, uint8_t addr, uint8_t reg, void *data, uint8_t size) {
	switch (_i2c->status) {
	case PORT_FREE:
		_i2c->status = PORT_BUSY;
		_i2c->addr = addr;
		FIFO_PutOne(_i2c->buffer, reg);
		FIFO_PutMulti(_i2c->buffer, (uint8_t*)data, size);
		_i2c->len = size;
		_i2c->mode = I2C_MODE_WRITE;
		I2C_Start_IRQ(_i2c);
		return 0;
		break;
	case PORT_DONE:
		_i2c->buffer->lockState = BUFFER_FREE;
		_i2c->status = PORT_FREE;
		return 1;	//exit
		break;
	case PORT_ERROR://TODO ADD error processing
		_i2c->buffer->lockState = BUFFER_FREE;
		FIFO_Init(_i2c->buffer);
		_i2c->status = PORT_FREE;
		return 1;
		break;
	default:
		return 0;
		break;
	}
}

uint8_t I2C_ReadOneByte(I2C_IRQ_Conn_t *_i2c, uint8_t addr, uint8_t reg, uint8_t *value) {
	switch (_i2c->status) {
	case PORT_FREE:
		_i2c->status = PORT_BUSY;
		_i2c->addr = addr;
		FIFO_PutOne(_i2c->buffer, reg);
		_i2c->len = 1;
		_i2c->mode = I2C_MODE_READ;
		I2C_Start_IRQ(_i2c);
		return 0;
		break;
	case PORT_DONE:
		_i2c->buffer->lockState = BUFFER_FREE;
		FIFO_GetOne(_i2c->buffer, value);
		_i2c->status = PORT_FREE;
		return 1;// exit
		break;
	case PORT_ERROR://TODO ADD error processing
		_i2c->buffer->lockState = BUFFER_FREE;
		FIFO_Init(_i2c->buffer);
		_i2c->status = PORT_FREE;
		return 1;
		break;
	default:
		return 0;
		break;
	}
}

uint8_t I2C_ReadBytes(I2C_IRQ_Conn_t *_i2c, uint8_t addr, uint8_t reg, void *data, uint8_t size) {
		switch (_i2c->status) {
	case PORT_FREE:
		_i2c->status = PORT_BUSY;
		_i2c->addr = addr;
		FIFO_PutOne(_i2c->buffer, reg);
		_i2c->len = size;
		_i2c->mode = I2C_MODE_READ;
		I2C_Start_IRQ(_i2c);		
		return 0;
		break;
	case PORT_DONE:
		_i2c->buffer->lockState = BUFFER_FREE;
		FIFO_GetMulti(_i2c->buffer, (uint8_t*)data, size);
		_i2c->status = PORT_FREE;
		return 1;// exit
		break;
	case PORT_ERROR://TODO ADD error processing
		_i2c->buffer->lockState = BUFFER_FREE;
		FIFO_Init(_i2c->buffer);
		_i2c->status = PORT_FREE;
		return 1;
		break;
	default:
		return 0;
		break;
	}
}
