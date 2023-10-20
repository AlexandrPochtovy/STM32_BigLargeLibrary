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
   
 * FILObuffer.c
 * Created on: Aug 18, 2022
*********************************************************************************/

#include "FILObuffer.h"

//---------------------------------------------------------------------------
void FILO_Init(filo_t *q) {
	q->ind = 0;
	q->bytes_avail = 0;
	q->state = BUFFER_FREE;
}

uint8_t FILO_PutOne(filo_t *q, size_t data) {
	if(q->buffer_size == q->bytes_avail){
		return 0;
	}
	q->buffer[q->ind] = data;
	q->ind += 1;
	q->bytes_avail += 1;
	return 1;
}

uint8_t FILO_GetOne(filo_t *q, size_t *data) {
  if((q->bytes_avail == 0) || (q->ind == 0)) {
    return 0;
  }
  else {
		q->ind -= 1;
		*data = q->buffer[q->ind];
		q->bytes_avail -= 1;
    return 1;
  }
}

uint8_t FILO_PutMulti(filo_t *q, size_t *data, size_t size) {
    if(q->buffer_size - q->bytes_avail < size){
        return 0;
    }
    else {
      memcpy(&q->buffer[q->ind], data, size * sizeof(q->buffer[0]));
      q->ind = (q->ind + size) % q->buffer_size;
      q->bytes_avail += size;
      return 1;
    }
}

uint8_t FILO_GetMulti(filo_t *q, size_t *data, size_t size) {
  if(q->bytes_avail < size){
      return 0;
  }
  else {
    memcpy(data, &q->buffer[q->ind], size * sizeof(q->buffer[0]));
    q->ind = (q->ind + size) % q->buffer_size;
    q->bytes_avail -= size;
    return 1;
  }
}

