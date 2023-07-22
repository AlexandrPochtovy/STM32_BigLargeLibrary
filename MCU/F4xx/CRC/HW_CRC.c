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
 
 HW_CRC.c
 Created on: Sep 20, 2023
*********************************************************************************/

#include "HW_CRC.h"

uint32_t F4xx_HW_CRC32(CRC_TypeDef *_crc, uint32_t *data, uint32_t len)
{
    // Calculate the CRC32 of the data buffer using the hardware peripheral
    uint32_t crc32 = 0;
    for (uint32_t i = 0; i < len; i++) {
        _crc->DR = data[i];
        crc32 = _crc->DR;
    }
    crc32 ^= 0xFFFFFFFF;// Finalize the CRC32 value by flipping all bits
    _crc->CR = CRC_CR_RESET;// Initialize the hardware CRC peripheral
    return crc32;
}

