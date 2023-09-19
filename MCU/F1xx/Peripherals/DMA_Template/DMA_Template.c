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
   
 *  DMA_Template.c
 *  Created on: 03.05.2021
 ********************************************************************************/

#include "DMA_Template.h"

void DMA_Ch1_IRQ_Callback(DMA_TypeDef *DMAnum) {
	volatile uint32_t sr = DMAnum->ISR;
	if (sr & DMA_ISR_GIF2) {
		if (sr & DMA_ISR_HTIF2) {
			LL_DMA_ClearFlag_HT2(DMAnum);
		}
		else if (sr & DMA_ISR_TCIF2) {
			LL_DMA_ClearFlag_TC2(DMAnum);
		}
		else if (sr & DMA_ISR_TEIF2) {
			LL_DMA_ClearFlag_TE2(DMAnum);
			LL_DMA_DisableChannel(DMAnum, LL_DMA_CHANNEL_1);
			LL_DMA_SetMemoryAddress(DMAnum, LL_DMA_CHANNEL_1, (uint32_t)0x00);
			LL_DMA_SetDataLength(DMAnum, LL_DMA_CHANNEL_1, (uint32_t)0x00);
		}
		LL_DMA_ClearFlag_GI2(DMAnum);
	}
}
//--------------------------------------------------------------------------
void DMA_Ch2_IRQ_Callback(DMA_TypeDef *DMAnum) {//SPI1 RX
	volatile uint32_t sr = DMAnum->ISR;
	if (sr & DMA_ISR_GIF2) {
		if (sr & DMA_ISR_HTIF2) {//recive half buffer
			LL_DMA_ClearFlag_HT2(DMAnum);
		}
		else if (sr & DMA_ISR_TCIF2) {//recive all buffer
			LL_DMA_ClearFlag_TC2(DMAnum);
		}
		else if (sr & DMA_ISR_TEIF2) {//recive error
			LL_DMA_ClearFlag_TE2(DMAnum);
			LL_DMA_DisableChannel(DMAnum, LL_DMA_CHANNEL_2);
			LL_DMA_SetMemoryAddress(DMAnum, LL_DMA_CHANNEL_2, (uint32_t)0x00);
			LL_DMA_SetDataLength(DMAnum, LL_DMA_CHANNEL_2, (uint32_t)0x00);
		}
		LL_DMA_ClearFlag_GI2(DMAnum);
	}
}
//--------------------------------------------------------------------------
void DMA_Ch3_IRQ_Callback(DMA_TypeDef *DMAnum) {//SPI1 TX
	volatile uint32_t sr = DMAnum->ISR;
	if (sr & DMA_ISR_GIF3) {
		if (sr & DMA_ISR_HTIF3) {
			LL_DMA_ClearFlag_HT3(DMAnum);
		}
		else if (sr & DMA_ISR_TCIF3) {
			LL_DMA_ClearFlag_TC3(DMAnum);
		}
		else if (sr & DMA_ISR_TEIF3) {
			LL_DMA_ClearFlag_TE3(DMAnum);
			LL_DMA_DisableChannel(DMAnum, LL_DMA_CHANNEL_3);
			LL_DMA_SetMemoryAddress(DMAnum, LL_DMA_CHANNEL_3, (uint32_t)0x00);
			LL_DMA_SetDataLength(DMAnum, LL_DMA_CHANNEL_3, (uint32_t)0x00);
		}
		LL_DMA_ClearFlag_GI3(DMAnum);
	}
}
//--------------------------------------------------------------------------
void DMA_Ch4_IRQ_Callback(DMA_TypeDef *DMAnum) {//I2C2 TX
	volatile uint32_t sr = DMAnum->ISR;
	if (sr & DMA_ISR_GIF4) {
		if (sr & DMA_ISR_HTIF4) {
			LL_DMA_ClearFlag_HT4(DMAnum);
		}
		else if (sr & DMA_ISR_TCIF4) {
			LL_DMA_ClearFlag_TC4(DMAnum);
		}
		else if (sr & DMA_ISR_TEIF4) {
			LL_DMA_ClearFlag_TE4(DMAnum);
			LL_DMA_DisableChannel(DMAnum, LL_DMA_CHANNEL_4);
			LL_DMA_SetMemoryAddress(DMAnum, LL_DMA_CHANNEL_4, (uint32_t)0x00);
			LL_DMA_SetDataLength(DMAnum, LL_DMA_CHANNEL_4, (uint32_t)0x00);
		}
		LL_DMA_ClearFlag_GI4(DMAnum);
	}
}
//--------------------------------------------------------------------------
void DMA_Ch5_IRQ_Callback(DMA_TypeDef *DMAnum) {//I2C2 RX
	volatile uint32_t sr = DMAnum->ISR;
	if (sr & DMA_ISR_GIF5) {//global interrupt DMA
		if (sr & DMA_ISR_HTIF5) {//half transfer complite interrupt
			LL_DMA_ClearFlag_HT5(DMAnum);
		}
		else if (sr & DMA_ISR_TCIF5) {//full transfer complite interrupt
			LL_DMA_ClearFlag_TC5(DMAnum);
		}
		else if (sr & DMA_ISR_TEIF5) {//error transfer interrupt
			LL_DMA_ClearFlag_TE5(DMAnum);
			LL_DMA_DisableChannel(DMAnum, LL_DMA_CHANNEL_5);
			LL_DMA_SetMemoryAddress(DMAnum, LL_DMA_CHANNEL_5, (uint32_t)0x00);
			LL_DMA_SetDataLength(DMAnum, LL_DMA_CHANNEL_5, (uint32_t)0x00);
		}
		LL_DMA_ClearFlag_GI5(DMAnum);
	}
}
//--------------------------------------------------------------------------
void DMA_Ch6_IRQ_Callback(DMA_TypeDef *DMAnum) {//USART2 RX
	volatile uint32_t sr = DMAnum->ISR;
	if (sr & DMA_ISR_GIF6) {
		if (sr & DMA_ISR_HTIF6) {
			LL_DMA_ClearFlag_HT6(DMAnum);
		}
		else if (sr & DMA_ISR_TCIF6) {
			LL_DMA_ClearFlag_TC6(DMAnum);
		}
		else if (sr & DMA_ISR_TEIF6) {
			LL_DMA_ClearFlag_TE6(DMAnum);
			LL_DMA_DisableChannel(DMAnum, LL_DMA_CHANNEL_6);
			LL_DMA_SetMemoryAddress(DMAnum, LL_DMA_CHANNEL_6, (uint32_t)0x00);
			LL_DMA_SetDataLength(DMAnum, LL_DMA_CHANNEL_6, (uint32_t)0x00);
		}
		LL_DMA_ClearFlag_GI6(DMAnum);
	}
}
//--------------------------------------------------------------------------
void DMA_Ch7_IRQ_Callback(DMA_TypeDef *DMAnum) {//USART2 TX
	volatile uint32_t sr = DMAnum->ISR;
	if (sr & DMA_ISR_GIF7) {
		if (sr & DMA_ISR_HTIF7) {
			LL_DMA_ClearFlag_HT7(DMAnum);
		}
		else if (sr & DMA_ISR_TCIF7) {
			LL_DMA_ClearFlag_TC7(DMAnum);
		}
		else if (sr & DMA_ISR_TEIF7) {
			LL_DMA_ClearFlag_TE7(DMAnum);
			LL_DMA_DisableChannel(DMAnum, LL_DMA_CHANNEL_7);
			LL_DMA_SetMemoryAddress(DMAnum, LL_DMA_CHANNEL_7, (uint32_t)0x00);
			LL_DMA_SetDataLength(DMAnum, LL_DMA_CHANNEL_7, (uint32_t)0x00);
		}
		LL_DMA_ClearFlag_GI7(DMAnum);
	}
}
//==========================================================================
void DMA_Ch1_Restart(DMA_TypeDef *DMAnum, uint8_t *data, uint16_t lenght) {
	LL_DMA_DisableChannel(DMAnum, LL_DMA_CHANNEL_1);
	LL_DMA_SetMemoryAddress(DMAnum, LL_DMA_CHANNEL_1, (uint32_t)data);
	LL_DMA_SetDataLength(DMAnum, LL_DMA_CHANNEL_1, (uint32_t)lenght);
	LL_DMA_EnableChannel(DMAnum, LL_DMA_CHANNEL_1);
}

void DMA_Ch2_Restart(DMA_TypeDef *DMAnum, uint8_t *data, uint16_t lenght) {
	LL_DMA_DisableChannel(DMAnum, LL_DMA_CHANNEL_2);
	LL_DMA_SetMemoryAddress(DMAnum, LL_DMA_CHANNEL_2, (uint32_t)data);
	LL_DMA_SetDataLength(DMAnum, LL_DMA_CHANNEL_2, (uint32_t)lenght);
	LL_DMA_EnableChannel(DMAnum, LL_DMA_CHANNEL_2);
}

void DMA_Ch3_Restart(DMA_TypeDef *DMAnum, uint8_t *data, uint16_t lenght) {
	LL_DMA_DisableChannel(DMAnum, LL_DMA_CHANNEL_3);
	LL_DMA_SetMemoryAddress(DMAnum, LL_DMA_CHANNEL_3, (uint32_t)data);
	LL_DMA_SetDataLength(DMAnum, LL_DMA_CHANNEL_3, (uint32_t)lenght);
	LL_DMA_EnableChannel(DMAnum, LL_DMA_CHANNEL_3);
}
//TX I2C2 restart --------------------------------------------------------------------------
void DMA_Ch4_Restart(DMA_TypeDef *DMAnum, uint8_t *data, uint16_t lenght) {
	LL_DMA_DisableChannel(DMAnum, LL_DMA_CHANNEL_4);
	LL_DMA_SetMemoryAddress(DMAnum, LL_DMA_CHANNEL_4, (uint32_t)data);
	LL_DMA_SetDataLength(DMAnum, LL_DMA_CHANNEL_4, (uint32_t)lenght);
	LL_DMA_EnableChannel(DMAnum, LL_DMA_CHANNEL_4);
}

//RX I2C2 restart --------------------------------------------------------------------------
void DMA_Ch5_Restart(DMA_TypeDef *DMAnum, uint8_t *data, uint16_t lenght) {
	LL_DMA_DisableChannel(DMAnum, LL_DMA_CHANNEL_5);
	LL_DMA_SetMemoryAddress(DMAnum, LL_DMA_CHANNEL_5, (uint32_t)data);
	LL_DMA_SetDataLength(DMAnum, LL_DMA_CHANNEL_5, (uint32_t)lenght);
	LL_DMA_EnableChannel(DMAnum, LL_DMA_CHANNEL_5);
}
//RX USART2 restart --------------------------------------------------------------------------
void DMA_Ch6_Restart(DMA_TypeDef *DMAnum, uint8_t *data, uint16_t lenght) {
	LL_DMA_DisableChannel(DMAnum, LL_DMA_CHANNEL_6);
	LL_DMA_SetMemoryAddress(DMAnum, LL_DMA_CHANNEL_6, (uint32_t)data);
	LL_DMA_SetDataLength(DMAnum, LL_DMA_CHANNEL_6, (uint32_t)lenght);
	LL_DMA_EnableChannel(DMAnum, LL_DMA_CHANNEL_6);
}
//TX USART2 restart --------------------------------------------------------------------------
void DMA_Ch7_Restart(DMA_TypeDef *DMAnum, uint8_t *data, uint16_t lenght) {
	LL_DMA_DisableChannel(DMAnum, LL_DMA_CHANNEL_7);
	LL_DMA_SetMemoryAddress(DMAnum, LL_DMA_CHANNEL_7, (uint32_t)data);
	LL_DMA_SetDataLength(DMAnum, LL_DMA_CHANNEL_7, (uint32_t)lenght);
	LL_DMA_EnableChannel(DMAnum, LL_DMA_CHANNEL_7);
}
