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

uint8_t I2C_WriteOneByte(I2C_IRQ_Conn_t *port, uint8_t addr, uint8_t reg, uint8_t value) {
	if (port->status == PORT_BUSY) {
		port->addr = addr;
		FIFO_PutOne(port->buffer, reg);
		FIFO_PutOne(port->buffer, value);
		port->len = 1;
		port->mode = I2C_MODE_WRITE;
		I2C_Start_IRQ(port);
	}
	else if (port->status == PORT_COMPLITE) {
		port->status = PORT_DONE;
		return 1;
	}
	else if (port->status == PORT_ERROR) {
		FIFO_Init(port->buffer);
		return 1;
	}
	return 0;
}

uint8_t I2C_WriteBytes(I2C_IRQ_Conn_t *port, uint8_t addr, uint8_t reg, uint8_t *data,
    uint8_t size) {
	if (port->status == PORT_BUSY) {
		port->addr = addr;
		FIFO_PutOne(port->buffer, reg);
		FIFO_PutMulti(port->buffer, data, size);
		port->len = size;
		port->mode = I2C_MODE_WRITE;
		I2C_Start_IRQ(port);
	}
	else if (port->status == PORT_COMPLITE) {
		port->status = PORT_DONE;
		return 1;
	}
	else if (port->status == PORT_ERROR) {
		FIFO_Init(port->buffer);
		return 1;
	}
	return 0;
}

uint8_t I2C_ReadOneByte(I2C_IRQ_Conn_t *port, uint8_t addr, uint8_t reg, uint8_t *value) {
	if (port->status == PORT_BUSY) {
		port->addr = addr;
		FIFO_PutOne(port->buffer, reg);
		port->len = 1;
		port->mode = I2C_MODE_READ;
		I2C_Start_IRQ(port);
	}
	else if (port->status == PORT_COMPLITE) {
		FIFO_GetOne(port->buffer, value);
		port->status = PORT_DONE;
		return 1;
	}
	else if (port->status == PORT_ERROR) {
		FIFO_Init(port->buffer);
		return 1;
	}
	return 0;
}

uint8_t I2C_ReadBytes(I2C_IRQ_Conn_t *port, uint8_t addr, uint8_t reg, uint8_t *data,
    uint8_t size) {
	if (port->status == PORT_BUSY) {
		port->addr = addr;
		FIFO_PutOne(port->buffer, reg);
		port->len = size;
		port->mode = I2C_MODE_READ;
		I2C_Start_IRQ(port);
	}
	else if (port->status == PORT_COMPLITE) {
		FIFO_GetMulti(port->buffer, data, size);
		port->status = PORT_DONE;
		return 1;
	}
	else if (port->status == PORT_ERROR) {
		FIFO_Init(port->buffer);
		return 1;
	}
	return 0;
}

uint8_t I2C_WriteOne(I2C_IRQ_Conn_t *port, uint8_t addr, uint8_t reg) {
	if (port->status == PORT_BUSY) {
		port->addr = addr;
		FIFO_PutOne(port->buffer, reg);
		port->len = 0;
		port->mode = I2C_MODE_WRITE;
		I2C_Start_IRQ(port);
	}
	else if (port->status == PORT_COMPLITE) {
		port->status = PORT_DONE;
		return 1;
	}
	else if (port->status == PORT_ERROR) {
		FIFO_Init(port->buffer);
		return 1;
	}
	return 0;
}
