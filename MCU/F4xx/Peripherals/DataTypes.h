/*
 * 
 *CommonDataTypes.h
 *  Created on: Aug 30, 2023
 *      Author: alexm
 */

#ifndef _DATATYPES_H_
#define _DATATYPES_H_

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

#endif /* _DATATYPES_H_ */
