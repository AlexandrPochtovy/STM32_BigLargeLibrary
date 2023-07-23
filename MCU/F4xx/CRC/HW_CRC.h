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
 
 HW_CRC.h
 Created on: Sep 20, 2023
*********************************************************************************/
#ifndef HW_CRC_H_
#define HW_CRC_H_

#include "stm32f4xx.h"
#include "stdlib.h"

uint32_t F4xx_HW_CRC32(CRC_TypeDef *_crc, uint32_t *data, uint32_t len);

#endif /* HW_CRC_H_ */
