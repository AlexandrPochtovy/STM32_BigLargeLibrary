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

 * 	TM1637.h
 *  Created on: Sep 25, 2020
 */

#ifndef _TM1637_H_
#define _TM1637_H_

#ifdef __cplusplus
extern "C" {
#endif
/********************************************************************/
#include "main.h"
#include "math.h"
#include "stdlib.h"
/********************************************************************/
typedef struct TM1637 {
	GPIO_TypeDef *Clock_Port;	//Clock port
	uint32_t Clock_Pin;				//Clock pin
	GPIO_TypeDef *Data_Port;	//Clock port
	uint32_t Data_Pin;				//Clock pin
	uint8_t delayNeed;				//if delay need, set 1
	uint8_t delay;						//delay value
	uint8_t brightness;				//brightness value
} TM1637_t;

//************definitions for TM1637*********************
enum command {
 ADDR_AUTO 		= 0x40,	//команда для отправки массива байт с первого индикатора
 ADDR_FIXED	 	= 0x44,	//команда отправки одного байта в определенный индикатор
 DISP_CTRL		=	0x88,	//команда управления индикаторами
 DISP_ACT			= 0x08	//маска бита включения индикаторов
};

enum flags {
PointFlag			= 0x80,		//маска бита точки
FirstTubeNum	= 0xC0,	//адрес первого индикатора
};

enum numbers {		//коды цифр
	_0 = 0x3f,
	_1 = 0x06,
	_2 = 0x5b,
	_3 = 0x4f,
	_4 = 0x66,
	_5 = 0x6d,
	_6 = 0x7d,
	_7 = 0x07,
	_8 = 0x7f,
	_9 = 0x6f
};

/************** БУКВЫ И СИМВОЛЫ *****************/
enum simbols {
_A = 0x77,
_B = 0x7f,
_C = 0x39,
_D = 0x3f,
_E = 0x79,
_F = 0x71,
_G = 0x3d,
_H = 0x76,
_I = 0x06,
_J = 0x1e,
_L = 0x38,
_N = 0x37,
_O = 0x3f,
_P = 0x73,
_S = 0x6d,
_U = 0x3e,
_Y = 0x6e,
_a = 0x5f,
_b = 0x7c,
_c = 0x58,
_d = 0x5e,
_e = 0x7b,
_f = 0x71,
_h = 0x74,
_i = 0x10,
_j = 0x0e,
_l = 0x06,
_n = 0x54,
_o = 0x5c,
_q = 0x67,
_r = 0x50,
_t = 0x78,
_u = 0x1c,
_y = 0x6e,
_dash = 0x40, //тире
_empty = 0x00, //пробел
_degree = 0x63 //градус
};
/********************************************************************/
void TM1637_WriteByteLine(TM1637_t *led, int8_t dataByte);

void TM1637_init(TM1637_t *led);
void TM1637_SendOneByte(TM1637_t *led, uint8_t pos, int8_t dataByte, int8_t en);	//выводит цифру DispData в указанную ячейку дисплея BitAddr
void TM1637_SendArray(TM1637_t *led, uint8_t *sendData, uint8_t en);				//выводит цифры массивом по ячейкам. От 1 до 4 (byte values[] = {3, 5, 9, 0};)
void TM1637_DisplayClock(TM1637_t *led, uint8_t hrs, uint8_t mins, uint8_t en);	//выводит часы и минуты
void TM1637_DisplayInt(TM1637_t *led, int16_t value, uint8_t en);					//выводит число от -999 до 9999 (да, со знаком минус)
void TM1637_Clear(TM1637_t *led);		//очистить дисплей
/********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SRC_TM1637_TM1637_H_ */
