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
   
 * FILObuffer.h
 * Created on: Aug 18, 2022
*********************************************************************************/

#ifndef FILOBUFFER_FILOBUFFER_H_
#define FILOBUFFER_FILOBUFFER_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "DataTypes/CommonDataTypes.h"

typedef struct filo {
    int32_t *buffer;
    volatile uint8_t   buffer_size;
    volatile uint8_t   ind;
    volatile uint8_t   bytes_avail;
    volatile BufferStatus_t lockState;
} filo_t;

void FILO_Init(filo_t *q);
uint8_t FILO_PutOne(filo_t *q, int32_t data);
uint8_t FILO_GetOne(filo_t *q, int32_t *data);

#endif /* FILOBUFFER_FILOBUFFER_H_ */
