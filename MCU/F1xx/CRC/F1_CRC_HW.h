/*********************************************************************************
 Original author: Aliaksandr Pachtovy<alex.mail.prime@gmail.com>

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 
 F1_CRC_HW.h
 Created on: Sep 20, 2020
*********************************************************************************/
#ifndef _F1_CRC_HW_H_
#define _F1_CRC_HW_H_

#include <stddef.h>
#include <stdint.h>
#include "stm32f1xx.h"

uint32_t HW_CRC32(const uint8_t* pData, size_t count, uint32_t init);

uint32_t crc32_zlib(const uint32_t *data, size_t cnt);

#endif /* _F1_CRC_HW_H_ */
