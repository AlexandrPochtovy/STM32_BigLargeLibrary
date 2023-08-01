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

 * 	TM1637.c
 *  Created on: Sep 25, 2020
 */

#include "TM1637.h"

//      A
//     ---
//  F |   | B
//     -G-
//  E |   | C
//     ---
//      D
const uint8_t TubeCode[] = {
	0x3F,	//0
	0x06,	//1
	0x5B,	//2
	0x4F,	//3
	0x66,	//4
	0x6D,	//5
	0x7D,	//6
	0x07,	//7
	0x7F,	//8
	0x6F,	//9
	0x00,	//space
	0x40	//-
	};

//==	RAW	===============================================================
//send start signal to TM1637
void TM1637_OpenLine(TM1637_t *led) {
	LL_GPIO_SetOutputPin(led->Clock_Port, led->Clock_Pin);
	LL_GPIO_SetOutputPin(led->Data_Port, led->Data_Pin);
	if (led->delayNeed) {LL_mDelay(led->delay);}
	LL_GPIO_ResetOutputPin(led->Data_Port, led->Data_Pin);
	LL_GPIO_ResetOutputPin(led->Clock_Port, led->Clock_Pin);
}
//End of transmission
void TM1637_CloseLine(TM1637_t *led) {
	LL_GPIO_ResetOutputPin(led->Clock_Port, led->Clock_Pin);
	LL_GPIO_ResetOutputPin(led->Data_Port, led->Data_Pin);
	if (led->delayNeed) {LL_mDelay(led->delay);}
	LL_GPIO_SetOutputPin(led->Clock_Port, led->Clock_Pin);
	LL_GPIO_SetOutputPin(led->Data_Port, led->Data_Pin);
}

uint8_t TM1637_CodeNumber(uint8_t number)
{return TubeCode[number];}

void TM1637_init(TM1637_t *led) {
	LL_GPIO_SetOutputPin(led->Clock_Port, led->Clock_Pin);
	LL_GPIO_SetOutputPin(led->Data_Port, led->Data_Pin);
}

void TM1637_WriteByteLine(TM1637_t *led, int8_t dataByte) {
	for (uint8_t i = 0; i < 8; i++) {//sent 8bit data
		LL_GPIO_ResetOutputPin(led->Clock_Port, led->Clock_Pin);
		if (dataByte & 0x01) {LL_GPIO_SetOutputPin(led->Data_Port, led->Data_Pin);} //LSB first
		else {LL_GPIO_ResetOutputPin(led->Data_Port, led->Data_Pin);}
		dataByte >>= 1;
		if (led->delayNeed) {LL_mDelay(led->delay);}
		LL_GPIO_SetOutputPin(led->Clock_Port, led->Clock_Pin);
	}
	LL_GPIO_ResetOutputPin(led->Clock_Port, led->Clock_Pin);//wait for the ACK
	LL_GPIO_SetOutputPin(led->Data_Port, led->Data_Pin);
	if (led->delayNeed) {LL_mDelay(led->delay);}
	LL_GPIO_SetOutputPin(led->Clock_Port, led->Clock_Pin);
}

//==  DISPLAY  ==========================================================
void TM1637_SendOneByte(TM1637_t *led, uint8_t pos, int8_t dataByte, int8_t en) {
	TM1637_OpenLine(led);//start signal sent to TM1637 from MCU
	TM1637_WriteByteLine(led, ADDR_FIXED);//
	TM1637_CloseLine(led);
	TM1637_OpenLine(led);
	TM1637_WriteByteLine(led, FirstTubeNum | pos);//
	TM1637_WriteByteLine(led, TM1637_CodeNumber(dataByte));//
	TM1637_CloseLine(led);            //
	TM1637_OpenLine(led);          //
	TM1637_WriteByteLine(led, en ? ((DISP_CTRL | (led->brightness & 0x07)) | DISP_ACT) : ((DISP_CTRL | (led->brightness & 0x07)) & ~DISP_ACT));//
	TM1637_CloseLine(led);          //
}

void TM1637_SendArray(TM1637_t *led, uint8_t *sendData, uint8_t en) {
	TM1637_OpenLine(led);          //start signal sent to TM1637 from MCU
	TM1637_WriteByteLine(led,ADDR_AUTO);//
	TM1637_CloseLine(led);           //
	TM1637_OpenLine(led);          //
	TM1637_WriteByteLine(led,FirstTubeNum);//
	for (uint8_t i = 0; i < 4; i++) {
		TM1637_WriteByteLine(led, TM1637_CodeNumber(sendData[i]));        //
	}
	TM1637_CloseLine(led);           //
	TM1637_OpenLine(led);          //
	TM1637_WriteByteLine(led, en ? ((DISP_CTRL | (led->brightness & 0x07)) | DISP_ACT) : ((DISP_CTRL | (led->brightness & 0x07)) & ~DISP_ACT));//
	TM1637_CloseLine(led);           //
}

void TM1637_Clear(TM1637_t *led) {
	uint8_t clr[] = {10,10,10,10};
	TM1637_SendArray(led, clr, 1);
}

void TM1637_DisplayClock(TM1637_t *led, uint8_t hrs, uint8_t mins, uint8_t en) {
	if (hrs < 100 && mins < 60)
	{
		uint8_t disp_time[4];
		if ((hrs / 10) == 0) disp_time[0] = 10;
		else disp_time[0] = (hrs / 10);
		disp_time[1] = hrs % 10;
		disp_time[2] = mins / 10;
		disp_time[3] = mins % 10;
		TM1637_SendArray(led, disp_time, en);
	}
}

void TM1637_DisplayInt(TM1637_t *led, int16_t value, uint8_t en) {
	if (value > 9999 || value < -999) {return;}
	else {
		uint8_t negative = value < 0 ? 1 : 0;
		value = abs(value);

		uint8_t digits[4];
		digits[0] = (uint8_t)(value / 1000);      	// количесто тысяч в числе
		uint16_t b = (uint16_t)(digits[0] * 1000); 	// вспомогательная переменная
		digits[1] = (uint8_t)((value - b) / 100); 	// получем количество сотен
		b += (uint16_t)(digits[1] * 100);               	// суммируем сотни и тысячи
		digits[2] = (uint8_t)((value - b) / 10);  	// получем десятки
		b += (uint16_t)(digits[2] * 10);                	// сумма тысяч, сотен и десятков
		digits[3] = (uint8_t)(value - b);              	// получаем количество единиц
/*переписать этот кусок, неправильно расставляет пустые значения и минус*/
		if (digits[2] == 0) {
			if (negative) digits[2] = 11;
			else digits[2] = 10;
		}
		if (digits[1] == 0) {
			if (negative) digits[1] = 11;
			else digits[1] = 10;
		}
		if (digits[0] == 0) {
			if (negative) digits[0] = 11;
			else digits[0] = 10;
		}
		TM1637_SendArray(led, digits, en);
	}
}
