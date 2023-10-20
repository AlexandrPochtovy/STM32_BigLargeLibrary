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
 
	MicroDelay.h
	Created on: 25/08/2022
*********************************************************************************/

#ifndef MICRODELAY_H_
#define MICRODELAY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f1xx.h"

#define DWT_CYCCNT	*(volatile uint32_t*)0xE0001004
#define DWT_CONTROL *(volatile uint32_t*)0xE0001000
#define SCB_DEMCR	*(volatile uint32_t*)0xE000EDFC

static inline void DWT_Init(void) {
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; //enable debug counter
	DWT->CYCCNT = 0U;								//reset counter
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;   			//start counter
}

static inline void delay_us(uint32_t us) {
	uint32_t us_count_tic =  us * (SystemCoreClock / 1000000U);
	DWT->CYCCNT = 0U;
	while(DWT->CYCCNT < us_count_tic);
}

static inline void DWT_Meas(uint32_t *val) {
	*val = DWT->CYCCNT;
	DWT->CYCCNT = 0;
}

#ifdef __cplusplus
}
#endif

#endif /* MICRODELAY_H_ */
