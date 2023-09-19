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

 * 	ADXL345_Register.h
 *	Created on: 31.01.2022
 */

#ifndef ADXL345_REGISTER_H_
#define ADXL345_REGISTER_H_

#define RATIO_2G						(float) (2 * 2) / 1024.0f
#define RATIO_4G						(float) (4 * 2) / 1024.0f
#define RATIO_8G  					(float) (8 * 2) / 1024.0f
#define RATIO_16G  					(float) (16 * 2) / 1024.0f
#define my_gravity 					9.80665 // m/s^2
#define ADXL345_DATA_LENGHT 6
/*************************** REGISTER MAP ***************************/
enum ADXL345_Register {
	ADXL345_DEVID_REG					= 0x00,	//R		Device ID
	ADXL345_THRESH_TAP_REG		= 0x1D,	//RW	Tap Threshold.
	ADXL345_OFSX_REG					= 0x1E,	//RW	X-Axis Offset.
	ADXL345_OFSY_REG					= 0x1F,	//RW 	Y-Axis Offset.
	ADXL345_OFSZ_REG					= 0x20,	//RW	Z- Axis Offset.
	ADXL345_TAP_DUR_REG				= 0x21,	//RW	Tap Duration.
	ADXL345_TAP_LATENT_REG		= 0x22,	//RW	Tap Latency.
	ADXL345_TAP_WINDOW_REG		= 0x23,	//RW	Tap Window.
	ADXL345_THRESH_ACT_REG		= 0x24,	//RW	Activity Threshold
	ADXL345_THRESH_INACT_REG	= 0x25,	//RW	Inactivity Threshold
	ADXL345_TIME_INACT_REG		= 0x26,	//RW	Inactivity Time
	ADXL345_ACT_INACT_CTL_REG	= 0x27,	//RW	Axis Enable Control for Activity and Inactivity Detection
	ADXL345_THRESH_FF_REG			= 0x28,		//RW	Free-Fall Threshold. recommendation 0x05..0x09
	ADXL345_TIME_FF_REG				= 0x29,		//RW 	Free-Fall Time. recommendation 0x14..0x44 (100-350ms)
	ADXL345_TAP_AXES_REG			= 0x2A,		//RW	Axis Control for Tap/Double Tap.
	ADXL345_ACT_TAP_STATUS_REG= 0x2B,	//R	Status of source Tap/Double Tap
	ADXL345_BW_RATE_REG				= 0x2C,	//RW	Data Rate and Power mode Control
	ADXL345_POWER_CTL_REG			= 0x2D,	//RW	Power-Saving Features Control
	ADXL345_INT_ENABLE_REG		= 0x2E,	//RW Interrupt Enable Control
	ADXL345_INT_MAP_REG				= 0x2F,	//RW	Interrupt Mapping Control
	ADXL345_INT_SOURCE_REG		= 0x30,	//R	Source of Interrupts
	ADXL345_DATA_FORMAT_REG		= 0x31,//RW	Data Format Control
	ADXL345_DATAX0_REG				= 0x32,	//R X-Axis Data 0
	ADXL345_DATAX1_REG				= 0x33,	//R X-Axis Data 1
	ADXL345_DATAY0_REG				= 0x34,	//R Y-Axis Data 0
	ADXL345_DATAY1_REG				= 0x35,	//R Y-Axis Data 1
	ADXL345_DATAZ0_REG				= 0x36,	//R Z-Axis Data 0
	ADXL345_DATAZ1_REG				= 0x37,	//R Z-Axis Data 1
	ADXL345_FIFO_CTL_REG			= 0x38,	//RW	FIFO Control
	ADXL345_FIFO_STATUS_REG		= 0x39		//R	FIFO Status
};

//Register 0x27 ADXL345_ACT_INACT_CTL Axis Enable Control for Activity and Inactivity Detection
enum ADXL345_ACT_INACT {
	ADXL345_INACT_Z_En	= 0b00000001,
	ADXL345_INACT_Y_En	= 0b00000010,
	ADXL345_INACT_X_En	= 0b00000100,
	ADXL345_INACT_ACDC	= 0b00001000,
	ADXL345_ACT_Z_En		= 0b00010000,
	ADXL345_ACT_Y_En		= 0b00100000,
	ADXL345_ACT_X_En		= 0b01000000,
	ADXL345_ACT_ACDC		= 0b10000000,
};
//Register 0x2A ADXL345_TAP_AXES RW Axis Control for Tap/Double Tap.
enum ADXL345_TAP_AXES_EN {
	ADXL345_TAP_Z_EN		= 0b00000001,
	ADXL345_TAP_Y_EN		= 0b00000010,
	ADXL345_TAP_X_EN		= 0b00000100,
	ADXL345_TAP_SUPRESS	= 0b00001000,
};
//Register 0x2B ADXL345_ACT_TAP_STATUS Status of source Tap/Double Tap
enum ADXL345_ACT_TAP_ST {
	ADXL345_TAP_Z_SOURCE = 0b00000001,
	ADXL345_TAP_Y_SOURCE = 0b00000010,
	ADXL345_TAP_X_SOURCE = 0b00000100,
	ADXL345_TAP_ASLEEP	 = 0b00001000,
	ADXL345_ACT_Z_SOURCE = 0b00010000,
	ADXL345_ACT_Y_SOURCE = 0b00100000,
	ADXL345_ACT_X_SOURCE = 0b01000000
};

//Register 0x2C ADXL345_BW_RATE Data Rate and Power mode Control
enum ADXL345_BW {
	ADXL345_BW_LOW_POWER	= 0b00010000,
	ADXL345_BW_1600	= 0b00001111,// 1111		IDD = 40uA
  ADXL345_BW_800	= 0b00001110,// 1110		IDD = 90uA
  ADXL345_BW_400	= 0b00001101,// 1101		IDD = 140uA
  ADXL345_BW_200	 =0b00001100,// 1100		IDD = 140uA
  ADXL345_BW_100	= 0b00001011,// 1011		IDD = 140uA
  ADXL345_BW_50		= 0b00001010,// 1010		IDD = 140uA
  ADXL345_BW_25		= 0b00001001,// 1001		IDD = 90uA
  ADXL345_BW_12_5	= 0b00001000,// 1000		IDD = 60uA
  ADXL345_BW_6_25	= 0b00000111,// 0111		IDD = 50uA
  ADXL345_BW_3_13	= 0b00000110,// 0110		IDD = 45uA
  ADXL345_BW_1_56	= 0b00000101,// 0101		IDD = 40uA
  ADXL345_BW_0_78	= 0b00000100,// 0100		IDD = 34uA
  ADXL345_BW_0_39	= 0b00000011,// 0011		IDD = 23uA
  ADXL345_BW_0_20	= 0b00000010,// 0010		IDD = 23uA
  ADXL345_BW_0_10	= 0b00000001,// 0001		IDD = 23uA
  ADXL345_BW_0_05	= 0b00000000 // 0000		IDD = 23uA
};

enum ADXL345_DATARATE {
    ADXL345_DATARATE_3200HZ    = 0b00001111,
    ADXL345_DATARATE_1600HZ    = 0b00001110,
    ADXL345_DATARATE_800HZ     = 0b00001101,
    ADXL345_DATARATE_400HZ     = 0b00001100,
    ADXL345_DATARATE_200HZ     = 0b00001011,
    ADXL345_DATARATE_100HZ     = 0b00001010,
    ADXL345_DATARATE_50HZ      = 0b00001001,
    ADXL345_DATARATE_25HZ      = 0b00001000,
    ADXL345_DATARATE_12_5HZ    = 0b00000111,
    ADXL345_DATARATE_6_25HZ    = 0b00000110,
    ADXL345_DATARATE_3_13HZ    = 0b00000101,
    ADXL345_DATARATE_1_56HZ    = 0b00000100,
    ADXL345_DATARATE_0_78HZ    = 0b00000011,
    ADXL345_DATARATE_0_39HZ    = 0b00000010,
    ADXL345_DATARATE_0_20HZ    = 0b00000001,
    ADXL345_DATARATE_0_10HZ    = 0b00000000
};

enum ADXL345_POWER_CTL {
	ADXL345_POWER_CTL_WAKEUP_8Hz 	= 0b00000000,
	ADXL345_POWER_CTL_WAKEUP_4Hz 	= 0b00000001,
	ADXL345_POWER_CTL_WAKEUP_2Hz 	= 0b00000010,
	ADXL345_POWER_CTL_WAKEUP_1Hz 	= 0b00000011,
	ADXL345_POWER_CTL_SLEEP				= 0b00000100,
	ADXL345_POWER_CTL_MEASURE			= 0b00001000,
	ADXL345_POWER_CTL_AUTO_SLEEP	= 0b00010000,
	ADXL345_POWER_CTL_LINK				= 0b00100000,
};

enum ADXL345_INT_ENABLE {
	ADXL345_INT_ENABLE_DATA_READY	= 0b10000000,
	ADXL345_INT_ENABLE_SINGLE_TAP	= 0b01000000,
	ADXL345_INT_ENABLE_DOUBLE_TAP	= 0b00100000,
	ADXL345_INT_ENABLE_ACTIVITY		= 0b00010000,
	ADXL345_INT_ENABLE_INACTIVITY	= 0b00001000,
	ADXL345_INT_ENABLE_FREE_FALL	= 0b00000100,
	ADXL345_INT_ENABLE_WATERMARK	= 0b00000010,
	ADXL345_INT_ENABLE_OVERRUN		= 0b00000001
};

enum ADXL345_INT_MAP {
	ADXL345_INT_MAP_DATA_READY		= 0b10000000,
	ADXL345_INT_MAP_SINGLE_TAP		= 0b01000000,
	ADXL345_INT_MAP_DOUBLE_TAP		= 0b00100000,
	ADXL345_INT_MAP_ACTIVITY		= 0b00010000,
	ADXL345_INT_MAP_INACTIVITY		= 0b00001000,
	ADXL345_INT_MAP_FREE_FALL		= 0b00000100,
	ADXL345_INT_MAP_WATERMARK		= 0b00000010,
	ADXL345_INT_MAP_OVERRUN			= 0b00000001
};

enum ADXL345_INT_SOURCE {
	ADXL345_INT_SOURCE_DATA_READY	= 0b10000000,
	ADXL345_INT_SOURCE_SINGLE_TAP	= 0b01000000,
	ADXL345_INT_SOURCE_DOUBLE_TAP	= 0b00100000,
	ADXL345_INT_SOURCE_ACTIVITY		= 0b00010000,
	ADXL345_INT_SOURCE_INACTIVITY	= 0b00001000,
	ADXL345_INT_SOURCE_FREE_FALL	= 0b00000100,
	ADXL345_INT_SOURCE_WATERMARK	= 0b00000010,
	ADXL345_INT_SOURCE_OVERRUN		= 0b00000001
};

enum ADXL345_DATA_FORMAT {
	ADXL345_DATA_FORMAT_RANGE_2G	= 0b00000000,
	ADXL345_DATA_FORMAT_RANGE_4G	= 0b00000001,
	ADXL345_DATA_FORMAT_RANGE_8G	= 0b00000010,
	ADXL345_DATA_FORMAT_RANGE_16G	= 0b00000011,
	ADXL345_DATA_FORMAT_Justify		= 0b00000100,
	ADXL345_DATA_FORMAT_FULL_RES	= 0b00001000,
	ADXL345_DATA_FORMAT_INT_INV		= 0b00100000,
	ADXL345_DATA_FORMAT_SPI				= 0b01000000,
	ADXL345_DATA_FORMAT_SELF_TEST	= 0b10000000,
};

#endif /* ADXL345_REGISTER_H_ */
