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

 * FIFObuffer.h
 * Created on: Aug 17, 2022
*********************************************************************************/
#ifndef _BUFFERS_FIFOBUFFERS_H_
#define _BUFFERS_FIFOBUFFERS_H_

#include <stdint.h>
#include <string.h>
#include "Buffers/BufferDataTypes.h"

/*****************************************************************
 * @brief  buffer datatype
 * @param  *buffer - pointer to array
 *         buffer_size - array length
 *         head - index for put element
 *         tail - index for get element
 *         bytes_avail - quantity data in buffer
 *         lockState - status buffer: free or lock
 * @retval none
 */
typedef struct fifo
{
  uint8_t *buffer;
  size_t buffer_size;
  size_t head;
  size_t tail;
  size_t bytes_avail;
  volatile BufferStatus_t lockState;
} fifo_t;

/*****************************************************************
 * @brief  Initializes the buffer, resets the head and tail indexes to zero.
 * @param  *q - pointer to buffer fifo_t
 * @retval none
 */
void FIFO_Init(fifo_t *q);

/*****************************************************************
 * @brief  Put one element to buffer
 * @param  *q - pointer to buffer fifo_t
 *         data - value for put
 * @retval 1 when end
 */
uint8_t FIFO_PutOne(fifo_t *q, uint8_t data);

/*****************************************************************
 * @brief  Get one element from buffer
 * @param  *q - pointer to buffer fifo_t
 *         *data - pointer to value storage for get
 * @retval 1 when end
 */
uint8_t FIFO_GetOne(fifo_t *q, uint8_t *data);

/*****************************************************************
 * @brief  Put some elements to buffer
 * @param  *q - pointer to buffer fifo_t
 *         *data - array or dataset for put
 *         size - quantity
 * @retval 1 when end
 */
uint8_t FIFO_PutMulti(fifo_t *q, uint8_t *data, size_t size);

/*****************************************************************
 * @brief  Get some elements from buffer
 * @param  *q - pointer to buffer fifo_t
 *         *data - array or dataset for storage
 *         size - quantity
 * @retval 1 when end
 */
uint8_t FIFO_GetMulti(fifo_t *q, uint8_t *data, size_t size);

#endif /* _BUFFERS_FIFOBUFFERS_H_ */
