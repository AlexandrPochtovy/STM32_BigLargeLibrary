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

 * FIFObuffer.h
 * Created on: Aug 17, 2022
*********************************************************************************/
#ifndef SRC_RINGFIFO_H_
#define SRC_RINGFIFO_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "DataTypes/BufferDataTypes.h"

typedef struct fifo {
    uint8_t *buffer;
    volatile size_t   buffer_size;
    volatile size_t   head;
    volatile size_t   tail;
    volatile size_t   bytes_avail;
    volatile BufferStatus_t lockState;
} fifo_t;

/* Initializes the buffer, resets the head and tail indexes. */
void FIFO_Init(fifo_t *q);

/* Put one element to buffer */
uint8_t FIFO_PutOne(fifo_t *q, uint8_t data);

/* Get one element from buffer */
uint8_t FIFO_GetOne(fifo_t *q, uint8_t *data);

/* Put size elements to buffer */
uint8_t FIFO_PutMulti(fifo_t *q, uint8_t *data, size_t size);

/* Get size elements from buffer */
uint8_t FIFO_GetMulti(fifo_t *q, uint8_t *data, size_t size);

#endif /* SRC_RINGBUFFER_H_ */
