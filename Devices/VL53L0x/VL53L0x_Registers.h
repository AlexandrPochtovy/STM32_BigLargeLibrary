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

 * 	VL53L0x_Registers.h
 *	Created on: Jul 25, 2022
 ********************************************************************************/

#ifndef VL53L0X_REGISTERS_H_
#define VL53L0X_REGISTERS_H_

enum VL53L0x_REG {
// SYSTEM CONFIG
	SYSRANGE_START = 0x00,
	  /*	0x00: 0: single shot mode 0x01:
	   0x01: 1: @continuous mode: toggle state @singleshot mode: arm next shot
	   0x02: 1: back-to-back operation mode
	   0x04: timed operation mode
	   0x08: histogram operation mode*/
	  SYSTEM_SEQUENCE_CONFIG = 0x01,
	  SYSTEM_THRESH_HIGH = 0x0C,
	  SYSTEM_THRESH_LOW = 0x0E,
	  SYSTEM_RANGE_CONFIG = 0x09,
	  SYSTEM_INTERMEASUREMENT_PERIOD = 0x04,
// GPIO CONFIG
	  SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A,
	  /*	0x00: Disabled
	   0x01: Low level
	   0x02: High level
	   0x03: Out of window
	   0x04: New sample ready*/
	  GPIO_HV_MUX_ACTIVE_HIGH = 0x84,
	  SYSTEM_INTERRUPT_CLEAR = 0x0B,
	  CHIP_I2C_ADDRESS = 0x8A,
	  I2C_MODE = 0x88,  // 0: i2c standard mode
// Result registers
	  RESULT_INTERRUPT_STATUS = 0x13,
	  RESULT_RANGE_STATUS = 0x14,
//CORE
	  RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC,
	  RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0,
	  RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0,
	  RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4,
	  RESULT_PEAK_SIGNAL_RATE_REF = 0xB6,
// Algo register,
	  ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28,
// Check Limit registers
	  MSRC_CONFIG_CONTROL = 0x60,
	  PRE_RANGE_CONFIG_MIN_SNR = 0x27,
	  PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56,
	  PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57,
	  PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64,
	  FINAL_RANGE_CONFIG_MIN_SNR = 0x67,
	  FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47,
	  FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48,
	  FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,
// PRE RANGE registers
	  PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61,
	  PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62,
	  PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50,
	  PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51,
	  PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52,
// Internal tuning registers,
	  INTERNAL_TUNING_1 = 0x91,
	  INTERNAL_TUNING_2 = 0xFF,
// Unknown registers
	  UNKNOWN_REGISTER_1 = 0x85,
	  UNKNOWN_REGISTER_2 = 0xCD,
	  UNKNOWN_REGISTER_3 = 0xCC,
	  UNKNOWN_REGISTER_4 = 0xBE,
// Other registers
	  SYSTEM_HISTOGRAM_BIN = 0x81,
	  HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33,
	  HISTOGRAM_CONFIG_READOUT_CTRL = 0x55,
	  FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70,
	  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71,
	  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72,
	  CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20,
	  MSRC_CONFIG_TIMEOUT_MACROP = 0x46,
	  SOFT_RESET_GO2_SOFT_RESET_N = 0xBF,
	  IDENTIFICATION_MODEL_ID = 0xC0,
	  IDENTIFICATION_REVISION_ID = 0xC2,
	  OSC_CALIBRATE_VAL = 0xF8,
	  GLOBAL_CONFIG_VCSEL_WIDTH = 0x32,
	  GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0,
	  GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1,
	  GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2,
	  GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3,
	  GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4,
	  GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5,
	  GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6,
	  DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E,
	  DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F,
	  POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80,
	  VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89,
	  ALGO_PHASECAL_LIM = 0x30,
	  ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30,
	  SOMETHING_MAGIC_REG = 0x83  //SOMETHING CONST, DON'T GIVE A FUCK
};

#endif /* VL53L0X_REGISTERS_H_ */
