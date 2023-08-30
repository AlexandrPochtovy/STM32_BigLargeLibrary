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
   
 * InterfaceDataTypes.h
 * Created on: 24/02/2022
 ********************************************************************************/

#ifndef INTERFACEDATATYPES_H_
#define INTERFACEDATATYPES_H_

/************************************************************************************
*									COMMON											*
************************************************************************************/
/* MCU peripheral state
* specifies that it is possible to work with peripherals at a low level
* with multiple errors or timeouts, the peripheral is considered faulty
*/
typedef enum PortStatus {	//состояние порта используется внутри функций
	PORT_FREE,				//свободно можно работать
	PORT_BUSY,				//занято
	PORT_DONE,    			//операция завершена успешно
	PORT_ERROR,				//возникли ошибки, повторить последний запрос
	PORT_TIMEOUT,			//таймаут, устройство не отвечает
	PORT_WTF				//произошла НЕХОРОШАЯ МАГИЯ, полный перезапуск с нуля
} PortStatus_t;

/* state of data exchange with the device as with a bus element
 * used to display the status of the process of working with the device for the main code
 */
typedef enum DeviceStatus {//состояние устройства на шине
	DEVICE_NOT_INIT,//устройство не настроено
	DEVICE_INIT,		//устройство настроено
	DEVICE_DONE,		//опрос выполняется без ошибок
	DEVICE_ERROR,		//опрос данных завершился с ошибкой
	DEVICE_FAULTH		//ошибки при опросе устройства, потеря связи и тд
} DeviceStatus_t;

/************************************************************************************
*									I2C												*
************************************************************************************/
typedef enum I2C_Mode {
	I2C_MODE_WRITE,	//for write
	I2C_MODE_READ,	//for read
	I2C_MODE_RW		//for read as write restart internal use only
} I2C_Mode_t;

typedef struct I2C_IRQ_Conn {
	I2C_TypeDef *i2c;	//pointer to HW i2c bus
	PortStatus_t status;//status I2C bus
	uint8_t step;		//step processing
	uint8_t addr;		//device I2C address
	uint8_t len;		//length data
	I2C_Mode_t mode;	//device mode
	fifo_t *buffer;		//pointer circular buffer
} I2C_IRQ_Conn_t;

typedef struct I2C_DMA_Conn {
	I2C_TypeDef *i2c;	//pointer to HW i2c bus
	void *DMAx;			//pointer to DMA MCU peripheral
	uint32_t Channel;	//pointer to dma channel
	PortStatus_t status;//status I2C bus
	uint8_t step;		//step processing
	uint8_t addr;		//device I2C address
	uint8_t reg;		//register I2C
	uint8_t len;		//length data
	I2C_Mode_t mode;	//device mode
	uint8_t *buffer;	//pointer linear buffer
} I2C_DMA_Conn_t;

/************************************************************************************
*									SPI												*
************************************************************************************/
typedef enum SPI_Mode {	//команда работы с устройством: чтение или запись данных
	SPI_MODE_WRITE,	//полудуплекс запись в шину
	SPI_MODE_READ,	//полудуплекс чтение из шины
	SPI_MODE_DUPLEX	//полный дуплекс чтение и запись параллельно
} SPI_Mode_t;
/*
 * общая структура соединения с любым устройством на шине состоит из:
 * структуры запроса по шине: адрес устройства, адрес регистра, длина запроса, режим чтение/запись
 * структуры работы с шиной: аппаратный адрес шины, состояние шины, буфер приема/передачи
 * состояния устройства: не настроено, настроено и готово, ошибка
 */
typedef struct SPI_Conn {
	SPI_TypeDef *SPIbus;	//pointer to HW SPI port
	Port_Status_t status;	//status port
	//SPI_Mode_t mode;			//read write mode
	fifo_t txbuffer;		//pointer circular buffer
	uint8_t txlen;			//length data
	fifo_t rxbuffer;		//pointer circular buffer
	uint8_t rxlen;			//length data
} SPI_Connection_t;

/************************************************************************************
*									USART											*
************************************************************************************/
	/*	общая структура соединения с любым устройством на шине состоит из:
	 * структуры запроса по шине: адрес устройства, адрес регистра, длина запроса, режим чтение/запись
	 * структуры работы с шиной: аппаратный адрес шины, состояние шины, буфер приема/передачи
	 * состояния устройства: не настроено, настроено и готово, ошибка
	 */
	typedef struct USART_Conn {
		USART_TypeDef *USART;		//pointer to HW USART port
		PortStatus_t txStatus;	//status USART port
		fifo_t *txbuffer;				//pointer circular buffer
		uint8_t txlen;					//length data
		PortStatus_t rxStatus;	//status USART port
		fifo_t *rxbuffer;				//pointer circular buffer
		uint8_t rxlen;					//length data
	} USART_Conn_t;


#endif /* INTERFACEDATATYPES_H_ */
