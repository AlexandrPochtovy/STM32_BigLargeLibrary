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

 * 	I2_HWC.c
 *	Created on: 30 nov. 2020
 */


#include "I2C_HW.h"
/*запускает обмен с I2C через прерывания, использовать
 * после того как передали в кольцевой буфер все необходимые данные
 */
void I2C_Start_IRQ(I2C_IRQ_Conn_t *_i2c) {
	_i2c->i2c->CR2 |= I2C_CR2_ITBUFEN;//Enable TXE RxNE iterrupt for >1 byte
	if (_i2c->len > 1) {
		LL_I2C_AcknowledgeNextData(_i2c->i2c, LL_I2C_ACK); 	// Ack enable if more one bytes read
	}
	else {
		LL_I2C_AcknowledgeNextData(_i2c->i2c, LL_I2C_NACK);//Ack disable if only one byte read
	}
	_i2c->i2c->CR1 |= I2C_CR1_START;//LL_I2C_GenerateStartCondition(_i2c->i2c);
}
/*запускает обмен по I2C с использованием DMA, не дописано*/
void I2C_Start_DMA(I2C_DMA_Conn_t *_i2c) {
	LL_I2C_DisableDMAReq_TX(_i2c->i2c);
	LL_I2C_DisableIT_BUF(_i2c->i2c);//отключаем прерывания чтобы работало DMA
	if (_i2c->len > 1) {
		LL_I2C_AcknowledgeNextData(_i2c->i2c, LL_I2C_ACK); 	// Ack enable if more one bytes read
	}	else {
		LL_I2C_AcknowledgeNextData(_i2c->i2c, LL_I2C_NACK);//Ack disable if only one byte read
	}
	//DMA_Ch4_Restart(DMA1, _i2c->buffer, (uint16_t)_i2c->len);
	_i2c->i2c->CR1 |= I2C_CR1_START;//LL_I2C_GenerateStartCondition(_i2c->i2c);
}
/* альтернативный основной обработчик прерывания, использовать в блоке обработки прерываний
 * после окончания обработки присваивает соединению статус "свободно"
 * при ошибках будет выполняться обработчик ошибок*/
void I2C_Raw_IRQ_CallBack(I2C_IRQ_Conn_t *_i2c) {
	volatile uint16_t I2C_SR1 = _i2c->i2c->SR1;//LL_I2C_ReadReg(_i2c->i2c, SR1);//Read SR1 first
	//EV5 Start condition generated. Clear: read SR1 and write slave addr to DR final
	if (I2C_SR1 & I2C_SR1_SB) {//start signal send device address
		if (_i2c->mode == I2C_MODE_RW) {// Read mode
			_i2c->i2c->DR = _i2c->addr | 0x01;
		}
		else { // Write mode
			_i2c->i2c->DR = _i2c->addr;
		}
	}
	// EV6 Address sent. Clear: read SR1, read SR2
	else if (I2C_SR1 & I2C_SR1_ADDR) {/* slave found and ASK recive*/
		if (_i2c->mode == I2C_MODE_WRITE) {//write mode
			(void)_i2c->i2c->SR2;	//Read SR2, clear ADDR
			FIFO_GetOne(_i2c->buffer, (uint8_t *)&_i2c->i2c->DR);//It's FIRST byte (reg or value)
			if (_i2c->len == 0) {						//write one byte
				LL_I2C_GenerateStopCondition(_i2c->i2c);//use errata & AN2824
				_i2c->status = PORT_DONE;				//set bus status free
			}
		}
		else if (_i2c->mode == I2C_MODE_RW) {
			(void) _i2c->i2c->SR2;			//Read SR2, clear ADDR
			if (_i2c->len == 1) {
				//_i2c->i2c->CR1 &=~I2C_CR1_ACK;		//Set ACK low (before clear ADDR, during EV6)
				_i2c->i2c->CR2 |= I2C_CR2_ITBUFEN; 	//Enable RxNE
				LL_I2C_GenerateStopCondition(_i2c->i2c);//ReStart after current byte transfer
			}
		}
		else if (_i2c->mode == I2C_MODE_READ) {
			//_i2c->mode = I2C_MODE_RW;	//switch mode to read-write
			(void) _i2c->i2c->SR2;				//Read SR2, clear ADDR
			FIFO_GetOne(_i2c->buffer, ((uint8_t *)&_i2c->i2c->DR));//Send first byte reg address
			LL_I2C_GenerateStartCondition(_i2c->i2c);//ReStart after current byte transfer
		}
		return;
	}
	//byte transref finished
	else if (I2C_SR1 & I2C_SR1_BTF) {
		if ((_i2c->mode == I2C_MODE_WRITE) && (_i2c->len == 0)) {	//if data end and TXE not pass
			_i2c->status = PORT_DONE;//set bus free status
		}
		else if (_i2c->mode == I2C_MODE_READ) {
			_i2c->mode = I2C_MODE_RW;	//switch mode to read-write
			return;
		}
		else if (_i2c->mode == I2C_MODE_RW) {
			if (_i2c->len == 0) {
				//PutOne(&_i2c->buffer, ((uint8_t)_i2c->i2c->DR));//read byte from data reg
				//--_i2c->len;
			}
		}
	}
	//RX buffer is not empty need read byte
	else if ((I2C_SR1 & I2C_SR1_RXNE) && (_i2c->mode == I2C_MODE_RW)) {
		if (_i2c->len == 1) {
			_i2c->i2c->CR2 &= ~I2C_CR2_ITBUFEN; 	//Disable RxNE
			FIFO_PutOne(_i2c->buffer, ((uint8_t)_i2c->i2c->DR));//read byte from data reg
			--_i2c->len;
			_i2c->status = PORT_DONE;//set bus free status
		}
		else if (_i2c->len == 2) {
			LL_I2C_AcknowledgeNextData(_i2c->i2c, LL_I2C_NACK);//Ack disable if only one byte read
			LL_I2C_GenerateStopCondition(_i2c->i2c);//ReStart after current byte transfer
			FIFO_PutOne(_i2c->buffer, ((uint8_t)_i2c->i2c->DR));//read byte from data reg
			--_i2c->len;
		}
		else {
			FIFO_PutOne(_i2c->buffer, ((uint8_t)_i2c->i2c->DR));//read byte from data reg
			--_i2c->len;
		}
	}
	// Data register empty (transmitters). Clear: write byte to DR or after START/STOP
	else if ((I2C_SR1 & I2C_SR1_TXE) && (_i2c->mode == I2C_MODE_WRITE)) {
		if (_i2c->len == 0) {//no more data
			_i2c->i2c->CR2 &= ~I2C_CR2_ITBUFEN; 	//Disable TXE
			LL_I2C_GenerateStopCondition(_i2c->i2c);//send stop
		} else {
			--_i2c->len;
			FIFO_GetOne(_i2c->buffer, ((uint8_t *)&_i2c->i2c->DR));//write byte
		}
	}
}

/* обработчик прерываний при обмене с использованием DMA переделать*/
void I2C_EV_IRQ_DMA_CallBack(I2C_DMA_Conn_t *_i2c) {
	volatile uint16_t I2C_SR1 = LL_I2C_ReadReg(_i2c->i2c, SR1);	//Read SR1
	//EV5 Start condition generated. Clear: read SR1 and write slave addr to DR
	if (I2C_SR1 & I2C_SR1_SB) {//send device address
		if (_i2c->mode == I2C_MODE_READ) {
			_i2c->i2c->DR = _i2c->addr | 0x01;	// Read mode
		}
		else {
			_i2c->i2c->DR = _i2c->addr;					// Write mode
		}
		return;
	}
	// EV6 Address sent. Clear: read SR1, read SR2
	else if (I2C_SR1 & I2C_SR1_ADDR) {// slave found and ASK recive
		if (_i2c->mode == I2C_MODE_WRITE) {//work correct
			(void) _i2c->i2c->SR2;	//Read SR2, clear ADDR
			_i2c->i2c->DR = _i2c->reg;
			LL_I2C_EnableDMAReq_TX(_i2c->i2c);//i2c_conn->I2Cbus->CR2 |= I2C_CR2_DMAEN;
		}
		return;
	}
	else if (I2C_SR1 & I2C_SR1_BTF) {//byte transmitted or recive complite
		//LL_I2C_DisableDMAReq_TX(i2c_conn->I2Cbus);//i2c_conn->I2Cbus->CR2 &= ~I2C_CR2_DMAEN;
		LL_I2C_GenerateStopCondition(_i2c->i2c);//Send STOP before last byte transfer
		_i2c->status = PORT_FREE;			//set status complete
		return;
	}
}
//=============================================================================
void I2C_ERR_IRQ_CallBack(I2C_IRQ_Conn_t *_i2c) {
	volatile uint32_t I2C_SR1 = 0;
	volatile uint32_t I2C_SR2 = 0;
	_i2c->status = PORT_ERROR;
	I2C_SR1 = _i2c->i2c->SR1;/* Read the I2C1 status register */
	I2C_SR2 = _i2c->i2c->SR2;/* Read the I2C1 status register */
	/* 10 bit AF detect */
	if (I2C_SR1 & I2C_SR1_AF) {//+
		LL_I2C_ClearFlag_AF(_i2c->i2c);
	}
		/* 8 bit BERR detect start-stop faulth, full restart bus*/
	else if (I2C_SR1 & I2C_SR1_BERR) {//+
		LL_I2C_ClearFlag_BERR(_i2c->i2c);
	}
	/* 9 bit ARLO detect wrong master found */
	else if (I2C_SR1 & I2C_SR1_ARLO) {//+
		LL_I2C_ClearFlag_ARLO(_i2c->i2c);
	}
	/* 11 bit OVR detect input buffer overload */
	else if (I2C_SR1 & I2C_SR1_OVR) {//+
		LL_I2C_ClearFlag_OVR(_i2c->i2c);
	}
	/* 12 bit PECERR detect */
	else if (I2C_SR1 & I2C_SR1_PECERR) {//+
		LL_I2C_ClearSMBusFlag_PECERR(_i2c->i2c);
	}
	/* 14 bit TIMEOUT detect */
	else if (I2C_SR1 & I2C_SR1_TIMEOUT) {//+
		LL_I2C_ClearSMBusFlag_TIMEOUT(_i2c->i2c);
	}
	/* 15 bit SMBALERT detect */
	else if (I2C_SR1 & I2C_SR1_SMBALERT) {//+
		LL_I2C_ClearSMBusFlag_ALERT(_i2c->i2c);
	}
	else if (I2C_SR2 & I2C_SR2_BUSY) {
		LL_I2C_Disable(I2C1);
		LL_I2C_Enable(I2C1);
	}
	else if (I2C_SR1 & I2C_SR1_STOPF) {//+
		LL_I2C_Disable(I2C1);
		LL_I2C_Enable(I2C1);
		}
	LL_I2C_GenerateStopCondition(_i2c->i2c);
}
//=============================================================================
