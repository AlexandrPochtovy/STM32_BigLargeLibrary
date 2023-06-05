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

 * FIFObuffer.c
 * Created on: Aug 17, 2022
*********************************************************************************/

#include "FIFObuffer.h"

static inline size_t min(size_t a, size_t b) {
    return a < b ? a : b;
}

static inline uint8_t NextCounter(const uint8_t Counter, const uint8_t max) {
   if  ((Counter + 1) > (max - 1)) {
      return 0;
   } else {
      return Counter + 1;
   };
}

//---------------------------------------------------------------------------
void FIFO_Init(fifo_t *q) {
    q->head = 0;
    q->tail = 0;
    q->bytes_avail = 0;
    q->lockState = BUFFER_FREE;
}

uint8_t FIFO_PutOne(fifo_t *q, uint8_t data) {
    if(q->buffer_size == q->bytes_avail){
        return 0;
    }
    q->buffer[q->tail] = data; 
    q->tail = NextCounter(q->tail, q->buffer_size);
    q->bytes_avail += 1;
    return 1;
}

uint8_t FIFO_GetOne(fifo_t *q, uint8_t *data) {
    if(q->bytes_avail == 0){
        return 0;
    }
    *data = q->buffer[q->head];
    q->head = NextCounter(q->head, q->buffer_size);
    q->bytes_avail -= 1;
    return 1;
}

uint8_t FIFO_PutMulti(fifo_t *q, uint8_t *data, size_t size) {
    if(q->buffer_size - q->bytes_avail < size){
        return 0;
    }
    const size_t part1 = min(size, q->buffer_size - q->tail);
    const size_t part2 = size - part1;
    memcpy(q->buffer + q->tail, data, part1);
    memcpy(q->buffer, data + part1, part2);
    q->tail = (q->tail + size) % q->buffer_size;
    q->bytes_avail += size;
    return 1;
}

uint8_t FIFO_GetMulti(fifo_t *q, uint8_t *data, size_t size) {
    if(q->bytes_avail < size){
        return 0;
    }
	const size_t part1 = min(size, q->buffer_size - q->head);
	const size_t part2 = size - part1;
	memcpy(data, q->buffer + q->head, part1);
	memcpy(data + part1, q->buffer, part2);
	q->head = (q->head + size) % q->buffer_size;
	q->bytes_avail -= size;
	return 1;
}

