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

 *	F1xx_DataTypes.h
 *  Created on: Aug 30, 2023
 */

#ifndef _F1xx_DATATYPES_H_
#define _F1xx_DATATYPES_H_

/************************************************************************************
 *									COMMON											*
 ************************************************************************************/
/* MCU peripheral state
 * specifies that it is possible to work with peripherals at a low level
 * with multiple errors or timeouts, the peripheral is considered faulty
 */
typedef enum PortStatus
{
  PORT_FREE,        // port free, ready for connect
  PORT_BUSY,        // port busy, data exchange is being prepared
  PORT_IN_PROGRESS, // data exchange is being processing
  PORT_COMPLITE,    // data exchange is being complite
  PORT_DONE,        // data exchange complite without error
  PORT_ERROR        // data exchange complite without error
} PortStatus_t;

/* state of data exchange with the device as with a bus element
 * used to display the status of the process of working with the device for the main code
 */
typedef enum DeviceStatus
{
  DEVICE_ON,         // device is power on, default setting
  DEVICE_READY,      // the device ready for data exchange
  DEVICE_PROCESSING, // data exchange in progress
  DEVICE_DONE,       // data exchange completed successfully
  DEVICE_ERROR,      // communication error: bad data, port/bus problem, etc, increment error count
  DEVICE_FAULTH      // device mark as "lost", process read-write data is off
} DeviceStatus_t;

#endif /* _F1xx_DATATYPES_H_ */
