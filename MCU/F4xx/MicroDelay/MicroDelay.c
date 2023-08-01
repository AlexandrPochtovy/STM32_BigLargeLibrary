/*
 * MicroDelay.c
 *
 *  Created on: 25/08/2022
 *      Author: alexm
 */

#include "MicroDelay.h"

#define DWT_CYCCNT	*(volatile uint32_t*)0xE0001004
#define DWT_CONTROL *(volatile uint32_t*)0xE0001000
#define SCB_DEMCR	*(volatile uint32_t*)0xE000EDFC

//base setup ========================================================
void DWC_Init(void) {
	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;// разрешаем использовать DWT
	DWT_CYCCNT = 0;// обнуляем счётчик
	DWT_CONTROL|= DWT_CTRL_CYCCNTENA_Msk; // включаем счётчик
}

