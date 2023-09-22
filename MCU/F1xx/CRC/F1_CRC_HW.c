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

  Base on forum
  https://we.easyelectronics.ru/STM32/crc32-na-stm32-kak-na-kompe-ili-na-kompe-kak-na-stm32.html
 
 F1_CRC_HW.c
 Created on: Sep 20, 2020
*********************************************************************************/

#include "F1_CRC_HW.h"

uint32_t HW_CRC32(const uint8_t* pData, size_t count, uint32_t init) {
  uint32_t crc;
  uint32_t *p32 = (uint32_t*) pData;
  size_t count32 = count >> 2;
  if (0xFFFFFFFF == init) {
  	CRC->CR |= CRC_CR_RESET;
  }
  while (count32--) {
  	CRC->DR = __RBIT(*p32++);
  }
  crc = __RBIT(CRC->DR);
  count = count % 4;
  if (count) {
  	CRC->DR = __RBIT(crc);
  	if (count == 1) {
  		CRC->DR = __RBIT((*p32 & 0x000000FF) ^ crc) >> 24;
  		crc = (crc >> 8) ^ __RBIT(CRC->DR);
  	} else if (count == 2) {
  		CRC->DR = (__RBIT((*p32 & 0x0000FFFF) ^ crc) >> 16);
  		crc = (crc >> 16) ^ __RBIT(CRC->DR);
  	} else if (count == 3) {
  		CRC->DR = __RBIT((*p32 & 0x00FFFFFF) ^ crc) >> 8;
  		crc = (crc >> 24) ^ __RBIT(CRC->DR);
  	}
  }
  return ~crc;
}

uint32_t crc32_zlib(const uint32_t *data, size_t cnt) {
  uint32_t i;
  CRC->CR = CRC_CR_RESET;
  for (i = 0; i < (cnt / 4); i++) {
    CRC->DR = __RBIT(data[i]);
  }
  uint32_t result = __RBIT(CRC->DR);
  cnt = (cnt % 4) * 8;
  if (cnt) {
    uint32_t tmp = CRC->DR;
	CRC->DR = tmp;
    CRC->DR = __RBIT((data[i] & (0xFFFFFFFF >> (32 - cnt))) ^ result) >> (32 - cnt);
    result = (result >> cnt) ^ __RBIT(CRC->DR);
  }
  return ~result;
}
