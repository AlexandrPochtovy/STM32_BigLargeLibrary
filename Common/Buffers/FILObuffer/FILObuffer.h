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
   
 * FILObuffer.h
 * Created on: Aug 18, 2022
*********************************************************************************/

#ifndef FILOBUFFER_FILOBUFFER_H_
#define FILOBUFFER_FILOBUFFER_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "Buffers/BufferDataTypes.h"

/*****************************************************************
  * @brief  buffer datatype
  * @param  *buffer - pointer to array
  *         buffer_size - array length
  *         ind - index for put/get element
  *         bytes_avail - quantity data in buffer
  *         lockState - status buffer: free or lock   
  * @retval none
  */
typedef struct filo {
    size_t *buffer;
    size_t   buffer_size;
    size_t   ind;
    size_t   bytes_avail;
    BufferStatus_t lockState;
} filo_t;

/*****************************************************************
  * @brief  Initializes the buffer, resets the head and tail indexes.
  * @param  *q - pointer to buffer fifo_t
  * @retval none
  */
void FILO_Init(filo_t *q);

/*****************************************************************
  * @brief  Put one element to buffer
  * @param  *q - pointer to buffer fifo_t
  *         data - value for put 
  * @retval 1 when end
  */
uint8_t FILO_PutOne(filo_t *q, size_t data);

/*****************************************************************
  * @brief  Get one element from buffer
  * @param  *q - pointer to buffer fifo_t
  *         *data - pointer to value storage for get 
  * @retval 1 when end
  */
uint8_t FILO_GetOne(filo_t *q, size_t *data);

/*****************************************************************
  * @brief  Put some elements to buffer
  * @param  *q - pointer to buffer fifo_t
  *         *data - array or dataset for put
  *         size - quantity 
  * @retval 1 when end
  */
uint8_t FILO_PutMulti(filo_t *q, size_t *data, size_t size);

/*****************************************************************
  * @brief  Get some elements from buffer
  * @param  *q - pointer to buffer fifo_t
  *         *data - array or dataset for storage
  *         size - quantity 
  * @retval 1 when end
  */
uint8_t FILO_GetMulti(filo_t *q, size_t *data, size_t size);

#endif /* FILOBUFFER_FILOBUFFER_H_ */
