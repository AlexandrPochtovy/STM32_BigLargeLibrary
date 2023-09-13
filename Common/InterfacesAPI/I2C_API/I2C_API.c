/*********************************************************************************
	Original author: user

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

uint8_t WriteOneRegByte(I2C_IRQ_Conn_t *_i2c, uint8_t addr, uint8_t reg, uint8_t value) {
	if ((_i2c->status == PORT_FREE) && (_i2c->step == 0)) {
		_i2c->status == PORT_BUSY;
		_i2c->addr = addr;
		FIFO_PutOne(_i2c->buffer, reg);
		FIFO_PutOne(_i2c->buffer, value);
		_i2c->len = 1;
		_i2c->mode = I2C_MODE_WRITE;
		_i2c->step = 1;
		I2C_Start_IRQ(_i2c);
		return 0;
	}
	else if ((_i2c->status == PORT_DONE) && (_i2c->step == 1)) {
		_i2c->step = 0;
		_i2c->status = PORT_FREE;
		return 1;// exit
	}
	else {//TODO  ADD error processing HERE
		FIFO_Init(_i2c->buffer);
		_i2c->step = 0;
		_i2c->status = PORT_FREE;
		return 0;//repeat
	}
}

uint8_t WriteRegBytes(I2C_IRQ_Conn_t *_i2c, uint8_t addr, uint8_t reg, void *data, uint8_t size) {
	if ((_i2c->status == PORT_FREE) && (_i2c->step == 0)) {
		_i2c->status == PORT_BUSY;
		_i2c->addr = addr;
		FIFO_PutOne(_i2c->buffer, reg);
		FIFO_PutMulti(_i2c->buffer, (uint8_t*)data, size);
		_i2c->len = size;
		_i2c->mode = I2C_MODE_WRITE;
		_i2c->step = 1;
		I2C_Start_IRQ(_i2c);
		return 0;
	}
	else if ((_i2c->status == PORT_DONE) && (_i2c->step == 1)) {
		_i2c->step = 0;
		_i2c->status = PORT_FREE;
		return 1;// exit
	}
	else {//TODO  ADD error processing HERE
		FIFO_Init(_i2c->buffer);
		_i2c->step = 0;
		_i2c->status = PORT_FREE;
		return 0;//repeat
	}
}

uint8_t ReadOneRegByte(I2C_IRQ_Conn_t *_i2c, uint8_t addr, uint8_t reg, uint8_t *value) {
	if ((_i2c->status == PORT_FREE) && (_i2c->step == 0)) {
		_i2c->status = PORT_BUSY;
		_i2c->addr = addr;
		FIFO_PutOne(_i2c->buffer, reg);
		_i2c->len = 1;
		_i2c->mode = I2C_MODE_READ;
		_i2c->step = 1;
		I2C_Start_IRQ(_i2c);
	}
	else if ((_i2c->status == PORT_DONE) && (_i2c->step == 1)) {
		FIFO_GetOne(_i2c->buffer, value);
		_i2c->step = 0;
		_i2c->status = PORT_FREE;
		return 1;// exit
	}
	else {//TODO  ADD error processing HERE
		FIFO_Init(_i2c->buffer);
		_i2c->step = 0;
		_i2c->status = PORT_FREE;
		return 0;//repeat
	}
}

uint8_t ReadRegBytes(I2C_IRQ_Conn_t *_i2c, uint8_t addr, uint8_t reg, void *data, uint8_t size) {
	if ((_i2c->status == PORT_FREE) && (_i2c->step == 0)) {
		_i2c->status = PORT_BUSY;
		_i2c->addr = addr;
		FIFO_PutOne(_i2c->buffer, reg);
		_i2c->len = size;
		_i2c->mode = I2C_MODE_READ;
		_i2c->step = 1;
		I2C_Start_IRQ(_i2c);
	}
	else if ((_i2c->status == PORT_DONE) && (_i2c->step == 1)) {
		FIFO_GetMulti(_i2c->buffer, (uint8_t*)data, size);
		_i2c->step = 0;
		_i2c->status = PORT_FREE;
		return 1;// exit
	}
	else {//TODO  ADD error processing HERE
		FIFO_Init(_i2c->buffer);
		_i2c->step = 0;
		_i2c->status = PORT_FREE;
		return 0;//repeat
	}
}
