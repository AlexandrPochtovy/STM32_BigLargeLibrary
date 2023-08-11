/*
 * MicroDelay.h
 *
 *  Created on: 25/08/2022
 *      Author: alexm
 */

#ifndef MICRODELAY_H_
#define MICRODELAY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define DWT_CYCCNT	*(volatile uint32_t*)0xE0001004
#define DWT_CONTROL *(volatile uint32_t*)0xE0001000
#define SCB_DEMCR		*(volatile uint32_t*)0xE000EDFC

__STATIC_INLINE void DWT_Init(void) {
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // разрешаем использовать счётчик
	DWT->CYCCNT = 0U;// обнуляем счётчик
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;   // запускаем счётчик
}

__STATIC_INLINE void delay_us(uint32_t us) {
	uint32_t us_count_tic =  us * (SystemCoreClock / 1000000U);
	DWT->CYCCNT = 0U;
	while(DWT->CYCCNT < us_count_tic);
}

__STATIC_INLINE void DWT_Meas(uint32_t *val) {
	*val = DWT->CYCCNT;
	DWT->CYCCNT = 0;
}

#ifdef __cplusplus
}
#endif

#endif /* MICRODELAY_H_ */
