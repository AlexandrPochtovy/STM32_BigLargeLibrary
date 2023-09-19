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
