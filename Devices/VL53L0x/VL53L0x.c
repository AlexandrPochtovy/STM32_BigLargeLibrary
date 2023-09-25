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

 * 	VL53L0x.cpp
 *	Created on: Jul 25, 2022
 ********************************************************************************/

#include "VL53L0x.h"

#define VL53L0x_SHOOTH_DEEP 8

/**********************************************************************
*          		STATIC INLINE FUNCTIONS                               * 
***********************************************************************/
/*****************************************************************
  * @brief  Encode VCSEL pulse period register value from period in PCLKs
  * based on VL53L0X_encode_vcsel_period()
  * @param  uint8_t period_pclks in clks
  * @retval uint8_t period in microsec
  */
static inline uint8_t __encodeVcselPeriod(uint8_t period_pclks) {
	return (((period_pclks) >> 1) - 1);
}

/*****************************************************************
  * @brief Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs from register value
  * based on VL53L0X_decode_vcsel_period()
  * @param
  * @retval
  */
static inline uint8_t __decodeVcselPeriod(uint8_t reg_val) {
	return (((reg_val) + 1) << 1);
}

/*****************************************************************
  * @brief Calculate macro period in nanoseconds from VCSEL period in PCLKs 
  * based on VL53L0X_calc_macro_period_ps()
  * PLL_period_ps = 1655; macro_period_vclks = 2304
  * @param
  * @retval
  */
static inline uint32_t __calcMacroPeriod(uint8_t vcsel_period_pclks) {
	return ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000);
}

/****************************************************************
 * 			VALUE CONVERTERS									*
 ****************************************************************/
/* Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
 * based on VL53L0X_calc_timeout_us()
 */
uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks) {
	uint32_t macro_period_ns = __calcMacroPeriod(vcsel_period_pclks);
	return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

/* Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
 * based on VL53L0X_calc_timeout_mclks()
 */
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks) {
	uint32_t macro_period_ns = __calcMacroPeriod(vcsel_period_pclks);
	return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

/* Decode sequence step timeout in MCLKs from register value
 * based on VL53L0X_decode_timeout()
 * Note: the original function returned a uint32_t, but the return value is
 * always stored in a uint16_t.
 * format: "(LSByte * 2^MSByte) + 1"
 */
uint16_t decodeTimeout(uint16_t reg_val) {
	return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

/* Encode sequence step timeout register value from timeout in MCLKs
 * based on VL53L0X_encode_timeout()
 * Note: the original function took a uint16_t, but the argument passed to it
 * is always a uint16_t.
 * // format: "(LSByte * 2^MSByte) + 1"
 */
uint16_t encodeTimeout(uint16_t timeout_mclks) {
	uint32_t ls_byte = 0;
	uint16_t ms_byte = 0;
	if (timeout_mclks > 0) {
		ls_byte = timeout_mclks - 1;
		while ((ls_byte & 0xFFFFFF00) > 0) {
			ls_byte >>= 1;
			ms_byte++;
		}
		return (ms_byte << 8) | (ls_byte & 0xFF);
	}
	else {
		return 0;
	}
}

void setupVL53L0X(VL53L0x_t *dev, uint16_t timeout) {
	dev->limit_timeout = timeout;
	dev->count_timeout = 0;
	dev->timeoutFlag = 0;
}

/*****************************************************************
  * @brief write to sensor new I2C address and store it in main sensor's structure
  * @note there is a problem, the new address is recorded but the sensor does not respond to the new address
  * do not use!
  * @param _i2c - pointer to I2C bus connection structure
  * @param dev - pointer to sensor's main structure
  * @param new_addr - new I2C address
  * @retval 1 when complite
  */
uint8_t setAddress(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev, const uint8_t new_addr) {
	dev->status = DEVICE_PROCESSING;
	if (I2C_WriteOneByte(_i2c, dev->addr, CHIP_I2C_ADDRESS, new_addr & 0x7F)) {
		dev->addr = new_addr;
		dev->status = DEVICE_DONE;
		return 1;
	}
	return 0;
}

/*****************************************************************
  * @brief read sensor's I2C address from structure
  * @param dev - pointer to sensor's main structure
  * @retval actual i2c address
  */
uint8_t getAddress(VL53L0x_t *dev) {
	return dev->addr;
}

/*****************************************************************
  * @brief read sensor's model ID and store it in main sensor's structure
  * @note use it for connection check
  * @param _i2c - pointer to I2C bus connection structure
  * @param dev - pointer to sensor's main structure
  * @retval 1 when complite
  */
uint8_t getModelId(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev) {
	uint8_t st;
	st = I2C_ReadOneByte(_i2c, dev->addr, IDENTIFICATION_MODEL_ID, &dev->modelID);
	return st;
}

/*****************************************************************
  * @brief read sensor's revision ID and store it in main sensor's structure
  * @note use it for connection check
  * @param _i2c - pointer to I2C bus connection structure
  * @param dev - pointer to sensor's main structure
  * @retval 1 when complite
  */
uint8_t getRevisionId(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev)
{
	uint8_t st;
	st = I2C_ReadOneByte(_i2c, dev->addr, IDENTIFICATION_REVISION_ID, &dev->revisionID);
	return st;
}

/*****************************************************************
  * @brief set new timeout value to main sensor's structure
  * @param dev - pointer to sensor's main structure
  * @param timeout - new timeout value
  * @retval none
  */
void setTimeout(VL53L0x_t *dev, const uint16_t timeout)
{
	dev->limit_timeout = timeout;
}

/*****************************************************************
  * @brief get actual timeout value from main sensor's structure
  * @note useless feature, made to be
  * @param dev - pointer to sensor's main structure
  * @retval timeout value
  */
uint16_t getTimeout(VL53L0x_t *dev)
{
	return dev->count_timeout;
}

/*****************************************************************
  * @brief clear timeout's count and flags in main sensor's structure
  * @param dev - pointer to sensor's main structure
  * @retval timeout value
  */
void startTimeout(VL53L0x_t *dev)
{
	dev->count_timeout = 0;
}

/*****************************************************************
  * @brief Check if timeout is enabled (set to nonzero value) and has expired
  * @param dev - pointer to sensor's main structure
  * @retval timeout flag
  */
uint8_t checkTimeoutExpired(VL53L0x_t *dev)  //+
{
	dev->timeoutFlag = ++dev->count_timeout > dev->limit_timeout;
	return dev->timeoutFlag;
}

/**********************************************************************
*                       INERNAL FUNCTION                              * 
***********************************************************************/
uint8_t setSignalRateLimit(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev, uint32_t limit_Mcps)
{
	uint8_t st;
	uint16_t data = Min(Max((uint16_t)limit_Mcps, 65535), 32);
	st = I2C_WriteBytes(_i2c, dev->addr, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (uint8_t* )&data, 2);
	return st;
}

uint8_t getSpadInfo(I2C_IRQ_Conn_t *_i2c,VL53L0x_t *dev)
{
	dev->status = DEVICE_PROCESSING;
	uint8_t st;
	switch (dev->stepL1) {
	case 0:
		st = I2C_WriteOneByte(_i2c, dev->addr, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
		if (st) {
			dev->stepL1 = 1;
		}
		break;
	case 1:
		st = I2C_WriteOneByte(_i2c, dev->addr, INTERNAL_TUNING_2, 0x01);
		if (st) {
			dev->stepL1 = 2;
		}
		break;
	case 2:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSRANGE_START, 0x00);
		if (st) {
			dev->stepL1 = 3;
		}
		break;
	case 3:
		st = I2C_WriteOneByte(_i2c, dev->addr, INTERNAL_TUNING_2, 0x06);
		if (st) {
			dev->stepL1 = 4;
		}
		break;
	case 4:
		st = I2C_ReadOneByte(_i2c, dev->addr, SOMETHING_MAGIC_REG, &dev->tmp8);
		if (st) {
			dev->tmp8 |= 0x04;
			dev->stepL1 = 5;
		}
		break;
	case 5:
		st = I2C_WriteOneByte(_i2c, dev->addr, SOMETHING_MAGIC_REG, dev->tmp8);
		if (st) {
			dev->stepL1 = 6;
		}
		break;
	case 6:
		st = I2C_WriteOneByte(_i2c, dev->addr, INTERNAL_TUNING_2, 0x07);
		if (st) {
			dev->stepL1 = 7;
		}
		break;
	case 7:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSTEM_HISTOGRAM_BIN, 0x01);
		if (st) {
			dev->stepL1 = 8;
		}
		break;
	case 8:
		st = I2C_WriteOneByte(_i2c, dev->addr, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
		if (st) {
			dev->stepL1 = 9;
		}
		;
		break;
	case 9:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x94, 0x6b);
		if (st) {
			dev->stepL1 = 10;
		}
		break;
	case 10:
		st = I2C_WriteOneByte(_i2c, dev->addr, SOMETHING_MAGIC_REG, 0x00);
		if (st) {
			setTimeout(dev, 100);
			startTimeout(dev);
			dev->stepL1 = 11;
		}
		break;
	case 11:
		st = I2C_ReadOneByte(_i2c, dev->addr, SOMETHING_MAGIC_REG, &dev->tmp8);
		if (st) {
			if (dev->tmp8 != 0x00) {
				dev->stepL1 = 12;
			}
			else if (checkTimeoutExpired(dev)) {
				dev->stepL1 = 0;
				dev->status = DEVICE_FAULTH;
				return 1;
			}
		}
		break;
	case 12:
		st = I2C_WriteOneByte(_i2c, dev->addr, SOMETHING_MAGIC_REG, 0x01);
		if (st) {
			dev->stepL1 = 13;
		}
		break;
	case 13:
		st = I2C_ReadOneByte(_i2c, dev->addr, 0x92, &dev->tmp8);
		if (st) {
			dev->spad_count = dev->tmp8 & 0x7f;
			dev->spad_type_is_aperture = (dev->tmp8 >> 7) & 0x01;
			dev->stepL1 = 14;
		}
		break;
	case 14:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSTEM_HISTOGRAM_BIN, 0x00);
		if (st) {
			dev->stepL1 = 15;
		}
		break;
	case 15:
		st = I2C_WriteOneByte(_i2c, dev->addr, INTERNAL_TUNING_2, 0x06);
		if (st) {
			dev->stepL1 = 16;
		}
		break;
	case 16:
		st = I2C_ReadOneByte(_i2c, dev->addr, SOMETHING_MAGIC_REG, &dev->tmp8);
		if (st) {
			dev->tmp8 &= ~0x04;
			dev->stepL1 = 17;
		}
		break;
	case 17:
		st = I2C_WriteOneByte(_i2c, dev->addr, SOMETHING_MAGIC_REG, dev->tmp8);
		if (st) {
			dev->stepL1 = 18;
		}
		break;
	case 18:
		st = I2C_WriteOneByte(_i2c, dev->addr, INTERNAL_TUNING_2, 0x01);
		if (st) {
			dev->stepL1 = 19;
		}
		break;
	case 19:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSRANGE_START, 0x01);
		if (st) {
			dev->stepL1 = 20;
		}
		break;
	case 20:
		st = I2C_WriteOneByte(_i2c, dev->addr, INTERNAL_TUNING_2, 0x00);
		if (st) {
			dev->stepL1 = 21;
		}
		break;
	case 21:
		st = I2C_WriteOneByte(_i2c, dev->addr, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
		if (st) {
			dev->stepL1 = 0;
			return 1;
		}
		break;
	default:
		dev->stepL1 = 0;
		break;
	}
	return 0;
}

uint8_t getSequenceStepEnables(I2C_IRQ_Conn_t *_i2c,VL53L0x_t *dev)
{
	uint8_t st;
	uint8_t sequence_config;
	st = I2C_ReadOneByte(_i2c, dev->addr, SYSTEM_SEQUENCE_CONFIG, &sequence_config);
	if (st) {
		dev->enables.tcc = (sequence_config >> 4) & 0x1;
		dev->enables.dss = (sequence_config >> 3) & 0x1;
		dev->enables.msrc = (sequence_config >> 2) & 0x1;
		dev->enables.pre_range = (sequence_config >> 6) & 0x1;
		dev->enables.final_range = (sequence_config >> 7) & 0x1;
	}
	return st;
}

uint8_t getVcselPulsePeriod(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev, vcselPeriodType type)
{
	uint8_t st = 0;
	if (type == VcselPeriodPreRange) {
		st = I2C_ReadOneByte(_i2c, dev->addr, PRE_RANGE_CONFIG_VCSEL_PERIOD, &dev->tmp8);
		if (st) {
			dev->vcselPeriodValue = __decodeVcselPeriod(dev->tmp8);
		}
	}
	else if (type == VcselPeriodFinalRange) {
		st = I2C_ReadOneByte(_i2c, dev->addr, FINAL_RANGE_CONFIG_VCSEL_PERIOD, &dev->tmp8);
		if (st) {
			dev->vcselPeriodValue = __decodeVcselPeriod(dev->tmp8);
		}
	}
	else {
		dev->vcselPeriodValue = 255;
	}
	return st;
}

uint8_t getSequenceStepTimeouts(I2C_IRQ_Conn_t *_i2c,VL53L0x_t *dev)
{
	uint8_t st;
	switch (dev->stepL1) {
	case 0:
		st = getVcselPulsePeriod(_i2c, dev, VcselPeriodPreRange);
		if (st) {
			dev->timeouts.pre_range_vcsel_period_pclks = dev->vcselPeriodValue;
			dev->stepL1 = 1;
		}
		break;
	case 1:
		st = I2C_ReadOneByte(_i2c, dev->addr, MSRC_CONFIG_TIMEOUT_MACROP, &dev->tmp8);
		if (st) {
			dev->timeouts.msrc_dss_tcc_mclks = dev->tmp8 + 1;
			dev->timeouts.msrc_dss_tcc_us = timeoutMclksToMicroseconds(
					dev->timeouts.msrc_dss_tcc_mclks, dev->timeouts.pre_range_vcsel_period_pclks);
			dev->stepL1 = 2;
		}
		break;
	case 2:
		st = I2C_ReadBytes(_i2c, dev->addr, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, (uint8_t *)&dev->tmp16, 2);
		if (st) {
			dev->timeouts.pre_range_mclks = decodeTimeout(dev->tmp16);
			dev->timeouts.pre_range_us = timeoutMclksToMicroseconds(dev->timeouts.pre_range_mclks, dev->timeouts.pre_range_vcsel_period_pclks);
			dev->stepL1 = 3;
		}
		break;
	case 3:
		st = getVcselPulsePeriod(_i2c, dev, VcselPeriodFinalRange);
		if (st) {
			dev->timeouts.final_range_vcsel_period_pclks= dev->vcselPeriodValue;
			dev->stepL1 = 4;
		}
		break;
	case 4:
		st = I2C_ReadBytes(_i2c, dev->addr, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, (uint8_t *)&dev->tmp16, 2);
		if (st) {
			dev->timeouts.final_range_mclks = decodeTimeout(dev->tmp16);
			if (dev->enables.pre_range) {
				dev->timeouts.final_range_mclks -= dev->timeouts.pre_range_mclks;
			}
			dev->timeouts.final_range_us = timeoutMclksToMicroseconds(dev->timeouts.final_range_mclks, dev->timeouts.final_range_vcsel_period_pclks);
			dev->stepL1 = 0;
			return 1;
		}
		break;
	default:
		dev->stepL1 = 0;
		break;
	}
	return 0;
}

uint8_t getMeasurementTimingBudget(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev)
{
	uint8_t st;
	uint16_t const StartOverhead = 1910;  // note that this is different than the value in set_
	uint16_t const EndOverhead = 960;
	uint16_t const MsrcOverhead = 660;
	uint16_t const TccOverhead = 590;
	uint16_t const DssOverhead = 690;
	uint16_t const PreRangeOverhead = 660;
	uint16_t const FinalRangeOverhead = 550;
	switch (dev->stepL2) {
	case 0:
		// "Start and end overhead times always present"
		dev->measurement_timing_budget_us = StartOverhead + EndOverhead;
		dev->stepL2 = 1; // @suppress("No break at end of case")
		//break;
	case 1:
		st = getSequenceStepEnables(_i2c, dev);
		if (st) {
			dev->stepL2 = 2;
		}
		break;
	case 2:
		st = getSequenceStepTimeouts(_i2c, dev);
		if (st) {
			dev->stepL2 = 3;
		}
		break;
	case 3:
		if (dev->enables.tcc) {
			dev->measurement_timing_budget_us += (dev->timeouts.msrc_dss_tcc_us + TccOverhead);
		}
		if (dev->enables.dss) {
			dev->measurement_timing_budget_us += 2 * (dev->timeouts.msrc_dss_tcc_us + DssOverhead);
		}
		else if (dev->enables.msrc) {
			dev->measurement_timing_budget_us += (dev->timeouts.msrc_dss_tcc_us + MsrcOverhead);
		}
		if (dev->enables.pre_range) {
			dev->measurement_timing_budget_us += (dev->timeouts.pre_range_us + PreRangeOverhead);
		}
		if (dev->enables.final_range) {
			dev->measurement_timing_budget_us += (dev->timeouts.final_range_us + FinalRangeOverhead);
		}
		dev->stepL2 = 0;
		return 1;
	default:
		dev->stepL2 = 0;
		break;
	}
	return 0;
}

uint8_t setMeasurementTimingBudget(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev, uint32_t budget_us)//check
{
	uint8_t st;
	uint16_t const StartOverhead = 1320;// note that this is different than the value in get_
	uint16_t const EndOverhead = 960;
	uint16_t const MsrcOverhead = 660;
	uint16_t const TccOverhead = 590;
	uint16_t const DssOverhead = 690;
	uint16_t const PreRangeOverhead = 660;
	uint16_t const FinalRangeOverhead = 550;
	uint32_t const MinTimingBudget = 20000;
	
	switch (dev->stepL2) {
	case 0:
		dev->sp_budget_us = budget_us;
		if (dev->sp_budget_us < MinTimingBudget) { dev->sp_budget_us = MinTimingBudget; } // @suppress("No break at end of case")
	case 1:
		dev->measurement_timing_budget_us = StartOverhead + EndOverhead;
		dev->sp_budget_us += 1;
		dev->stepL2 = 2;
		break;
	case 2:
		st = getSequenceStepEnables(_i2c, dev);
		if (st) {
			dev->stepL2 = 3;
		}
		break;
	case 3:
		st = getSequenceStepTimeouts(_i2c, dev);
		if (st) {
			dev->stepL2 = 4;
		}
		break;
	case 4:
		if (dev->enables.tcc) {
			dev->measurement_timing_budget_us += (dev->timeouts.msrc_dss_tcc_us + TccOverhead);
		}
		if (dev->enables.dss) {
			dev->measurement_timing_budget_us += 2 * (dev->timeouts.msrc_dss_tcc_us + DssOverhead);
		}
		else if (dev->enables.msrc) {
			dev->measurement_timing_budget_us += (dev->timeouts.msrc_dss_tcc_us + MsrcOverhead);
		}
		if (dev->enables.pre_range) {
			dev->measurement_timing_budget_us += (dev->timeouts.pre_range_us + PreRangeOverhead);
		}
		if (dev->enables.final_range) {
			dev->measurement_timing_budget_us += FinalRangeOverhead;
			// "Note that the final range timeout is determined by the timing
			// budget and the sum of all other timeouts within the sequence.
			// If there is no room for the final range timeout, then an error
			// will be set. Otherwise the remaining time will be applied to
			// the final range."
			if (dev->measurement_timing_budget_us > dev->sp_budget_us) {//Requested timeout too small
				dev->stepL2 = 1;
				break;
			}
			uint32_t final_range_timeout_us = dev->sp_budget_us - dev->measurement_timing_budget_us;
			// set_sequence_step_timeout() begin
			// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)
			// "For the final range timeout, the pre-range timeout
			//  must be added. To do this both final and pre-range
			//  timeouts must be expressed in macro periods MClks
			//  because they have different vcsel periods."
			uint16_t final_range_timeout_mclks = timeoutMicrosecondsToMclks(final_range_timeout_us, dev->timeouts.final_range_vcsel_period_pclks);
			if (dev->enables.pre_range) {
				final_range_timeout_mclks += dev->timeouts.pre_range_mclks;
			}
			dev->tmp16 = encodeTimeout(final_range_timeout_mclks);
			dev->stepL2 = 5;
		}
		else {
			dev->status = DEVICE_FAULTH;
			dev->stepL2 = 0;
			return 1;
		}
		break;
	case 5:
		st = I2C_WriteBytes(_i2c, dev->addr, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, (uint8_t *)&dev->tmp16, 2);
		st = 1;
		if (st) {
			// set_sequence_step_timeout() end
			dev->measurement_timing_budget_us = budget_us;  // store for internal reuse
			dev->stepL2 = 0;
			return 1;
		}
		break;
	default:
		dev->stepL2 = 0;
		break;
	}
	return 0;
}

uint8_t performSingleRefCalibration(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev, uint8_t vhv_init_byte)
{
	uint8_t st;
	switch (dev->stepL1) {
	case 0:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSRANGE_START, 0x01 | vhv_init_byte);// VL53L0X_REG_SYSRANGE_MODE_START_STOP
		if (st) {
			startTimeout(dev);
			dev->stepL1 = 1;
		}
		break;
	case 1:
		st = I2C_ReadOneByte(_i2c, dev->addr, RESULT_INTERRUPT_STATUS, &dev->tmp8);
		if (st) {
			if ((dev->tmp8 & 0x07) != 0) {
				dev->stepL1 = 2;
			}
			else if (checkTimeoutExpired(dev)) {
				dev->status = DEVICE_FAULTH;
				dev->stepL1 = 2;
			}
		}
		break;
	case 2:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSTEM_INTERRUPT_CLEAR, 0x01);
		if (st) {
			dev->stepL1 = 3;
		}
		break;
	case 3:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSRANGE_START, 0x00);
		if (st) {
			dev->stepL1 = 0;
			return 1;
		}
		break;
	default:
		dev->stepL1 = 0;
		break;
	}
	return 0;
}

uint8_t setVcselPulsePeriod(I2C_IRQ_Conn_t *_i2c,VL53L0x_t *dev,vcselPeriodType type,uint8_t period_pclks)
{
	uint8_t st;
	switch (dev->stepL3) {
	case 0:
		st = getSequenceStepEnables(_i2c, dev);
		if (st) {
			dev->stepL3 = 1;
		}
		break;
	case 1:
		st = getSequenceStepTimeouts(_i2c, dev);
		if (st) {
			dev->vcsel_period_reg = __encodeVcselPeriod(period_pclks);
			dev->stepL3 = 2;
		}
		break;
	case 3:
		// "Apply specific settings for the requested clock period"
		// "Re-calculate and apply timeouts, in macro periods"
		// "When the VCSEL period for the pre or final range is changed,
		// the corresponding timeout must be read from the device using
		// the current VCSEL period, then the new VCSEL period can be
		// applied. The timeout then must be written back to the device
		// using the new VCSEL period.
		// For the MSRC timeout, the same applies - this timeout being
		// dependant on the pre-range vcsel period."
		if (type == VcselPeriodPreRange) {  // "Set phase check limits"
			switch (period_pclks) {
			case 12:
				dev->stepL3 = 5;
				break;
			case 14:
				dev->stepL3 = 6;
				break;
			case 16:
				dev->stepL3 = 7;
				break;
			case 18:
				dev->stepL3 = 8;
				break;
			default:// invalid period
				dev->status = DEVICE_FAULTH;
				return 1;
			}
		}
		else if (type == VcselPeriodFinalRange) {
			switch (period_pclks) {
			case 8:
				dev->stepL3 = 15;
				break;
			case 10:
				dev->stepL3 = 25;
				break;
			case 12:
				dev->stepL3 = 35;
				break;
			case 14:
				dev->stepL3 = 45;
				break;
			default:
				dev->status = DEVICE_FAULTH;// invalid period
				return 1;
			}
		}
		else {  // invalid type
			return 1;
		}
		break;
		//type == VcselPeriodPreRange
	case 5:  //period_pclks = 12
		st = I2C_WriteOneByte(_i2c, dev->addr, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
		if (st) {
			dev->stepL3 = 9;
		}
		break;
	case 6:  //period_pclks = 14
		st = I2C_WriteOneByte(_i2c, dev->addr, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
		if (st) {
			dev->stepL3 = 9;
		}
		break;
	case 7:  //period_pclks = 16
		st = I2C_WriteOneByte(_i2c, dev->addr, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
		if (st) {
			dev->stepL3 = 9;
		}
		break;
	case 8:  //period_pclks = 18
		st = I2C_WriteOneByte(_i2c, dev->addr, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
		if (st) {
			dev->stepL3 = 9;
		}
		break;
	case 9:
		st = I2C_WriteOneByte(_i2c, dev->addr, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
		if (st) {
			dev->stepL3 = 10;
		}
		break;
	case 10:// apply new VCSEL period
		st = I2C_WriteOneByte(_i2c, dev->addr, PRE_RANGE_CONFIG_VCSEL_PERIOD, dev->vcsel_period_reg);
		if (st) {
			// set_sequence_step_timeout() begin
			// (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)
			dev->new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(dev->timeouts.pre_range_us, period_pclks);
			dev->tmp16 = encodeTimeout(dev->new_pre_range_timeout_mclks);
			dev->stepL3 = 11;
		}
		break;
	case 11:// update timeouts
		st = I2C_WriteBytes(_i2c, dev->addr, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, (uint8_t *)&dev->tmp16, 2);
		// set_sequence_step_timeout() end
		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)
		if (st) {
			dev->stepL3 = 12;
		}
		break;
	case 12:
		dev->new_msrc_timeout_mclks = timeoutMicrosecondsToMclks(dev->timeouts.msrc_dss_tcc_us, period_pclks);
		dev->tmp8 = dev->new_msrc_timeout_mclks > 256 ? 255 : dev->new_msrc_timeout_mclks - 1;
		st = I2C_WriteOneByte(_i2c, dev->addr, MSRC_CONFIG_TIMEOUT_MACROP, dev->tmp8);
		if (st) {
			dev->stepL3 = 102;
		}
		break;// set_sequence_step_timeout() end

	//type == VcselPeriodFinalRange
	case 15:  //period_pclks = 8
		st = I2C_WriteOneByte(_i2c, dev->addr, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
		if (st) {
			dev->stepL3 = 16;
		}
		break;
	case 16:
		st = I2C_WriteOneByte(_i2c, dev->addr, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
		if (st) {
			dev->stepL3 = 17;
		}
		break;
	case 17:
		st = I2C_WriteOneByte(_i2c, dev->addr, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
		if (st) {
			dev->stepL3 = 18;
		}
		break;
	case 18:
		st = I2C_WriteOneByte(_i2c, dev->addr, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
		if (st) {
			dev->stepL3 = 19;
		}
		break;
	case 19:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x01);
		if (st) {
			dev->stepL3 = 20;
		}
		break;
	case 20:
		st = I2C_WriteOneByte(_i2c, dev->addr, ALGO_PHASECAL_LIM, 0x30);
		if (st) {
			dev->stepL3 = 51;
		}
		break;

	case 25:  //period_pclks = 10
		st = I2C_WriteOneByte(_i2c, dev->addr, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
		if (st) {
			dev->stepL3 = 26;
		}
		break;
	case 26:
		st = I2C_WriteOneByte(_i2c, dev->addr, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
		if (st) {
			dev->stepL3 = 27;
		}
		break;
	case 27:
		st = I2C_WriteOneByte(_i2c, dev->addr, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
		if (st) {
			dev->stepL3 = 28;
		}
		break;
	case 28:
		st = I2C_WriteOneByte(_i2c, dev->addr, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
		if (st) {
			dev->stepL3 = 49;
		}
		break;

	case 35:  //period_pclks = 12
		st = I2C_WriteOneByte(_i2c, dev->addr, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
		if (st) {
			dev->stepL3 = 36;
		}
		break;
	case 36:
		st = I2C_WriteOneByte(_i2c, dev->addr, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
		if (st) {
			dev->stepL3 = 37;
		}
		break;
	case 37:
		st = I2C_WriteOneByte(_i2c, dev->addr, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
		if (st) {
			dev->stepL3 = 38;
		}
		break;
	case 38:
		st = I2C_WriteOneByte(_i2c, dev->addr, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
		if (st) {
			dev->stepL3 = 49;
		}
		break;

	case 45:  //period_pclks = 14
		st = I2C_WriteOneByte(_i2c, dev->addr, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
		if (st) {
			dev->stepL3 = 46;
		}
		break;
	case 46:
		st = I2C_WriteOneByte(_i2c, dev->addr, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
		if (st) {
			dev->stepL3 = 47;
		}
		break;
	case 47:
		st = I2C_WriteOneByte(_i2c, dev->addr, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
		if (st) {
			dev->stepL3 = 48;
		}
		break;
	case 48:
		st = I2C_WriteOneByte(_i2c, dev->addr, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
		if (st) {
			dev->stepL3 = 49;
		}
		break;
	case 49:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x01);
		if (st) {
			dev->stepL3 = 50;
		}
		break;
	case 50:
		st = I2C_WriteOneByte(_i2c, dev->addr, ALGO_PHASECAL_LIM, 0x20);
		if (st) {
			dev->stepL3 = 51;
		}
		break;
	case 51:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x00);
		if (st) {
			dev->stepL3 = 55;
		}
		break;
	case 55:  // apply new VCSEL period
		st = I2C_WriteOneByte(_i2c, dev->addr, FINAL_RANGE_CONFIG_VCSEL_PERIOD, dev->vcsel_period_reg);
		if (st) {
		// "For the final range timeout, the pre-range timeout
		//  must be added. To do this both final and pre-range
		//  timeouts must be expressed in macro periods MClks
		//  because they have different vcsel periods."
			dev->new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(dev->timeouts.final_range_us, period_pclks);
			if (dev->enables.pre_range) {
				dev->new_final_range_timeout_mclks += dev->timeouts.pre_range_mclks;
			}
			dev->tmp16 = encodeTimeout(dev->new_final_range_timeout_mclks);
			dev->stepL3 = 56;
		}
		break;
	case 56:// update timeouts
		// set_sequence_step_timeout() begin
		st = I2C_WriteBytes(_i2c, dev->addr, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, (uint8_t *)&dev->tmp16, 2);
		if (st) {
			dev->stepL3 = 102;
		}
		break;
		// set_sequence_step_timeout end
	case 102:// "Finally, the timing budget must be re-applied"
		st = setMeasurementTimingBudget(_i2c, dev, dev->measurement_timing_budget_us);
		if (st) {
			dev->stepL3 = 103;
		}
		break;
	case 103:
		st = I2C_ReadOneByte(_i2c, dev->addr, SYSTEM_SEQUENCE_CONFIG, &dev->sequence_config);
		if (st) {
			dev->stepL3 = 104;
		}
		break;
	case 104:// "Perform the phase calibration. This is needed after changing on vcsel period."
		// VL53L0X_perform_phase_calibration() begin
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSTEM_SEQUENCE_CONFIG, 0x02);
		if (st) {
			dev->stepL3 = 105;
		}
		break;
	case 105:
		st = performSingleRefCalibration(_i2c, dev, 0x00);
		if (st) {
			dev->stepL3 = 106;
		}
		break;
	case 106:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSTEM_SEQUENCE_CONFIG, dev->sequence_config);
		if (st) {
			dev->stepL3 = 0;
			return 1;
		}
		break;
		// VL53L0X_perform_phase_calibration() end
	default:
		dev->stepL3 = 0;
		break;
	}
	return 0;
}

uint8_t startContinuous(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev, uint32_t period_ms)
{
	uint8_t st;
	switch (dev->stepL1) {
	case 0:
		st = I2C_WriteOneByte(_i2c, dev->addr, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
		if (st) {
			dev->stepL1 = 1;
		}
		break;
	case 1:
		st = I2C_WriteOneByte(_i2c, dev->addr, INTERNAL_TUNING_2, 0x01);
		if (st) {
			dev->stepL1 = 2;
		}
		break;
	case 2:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSRANGE_START, 0x00);
		if (st) {
			dev->stepL1 = 3;
		}
		break;
	case 3:
		st = I2C_WriteOneByte(_i2c, dev->addr, INTERNAL_TUNING_1, dev->stop_variable);
		if (st) {
			dev->stepL1 = 4;
		}
		break;
	case 4:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSRANGE_START, 0x01);
		if (st) {
			dev->stepL1 = 5;
		}
		break;
	case 5:
		st = I2C_WriteOneByte(_i2c, dev->addr, INTERNAL_TUNING_2, 0x00);
		if (st) {
			dev->stepL1 = 6;
		}
		break;
	case 6:
		st = I2C_WriteOneByte(_i2c, dev->addr, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
		if (st) {
			if (period_ms) {
				dev->stepL1 = 7;
			}
			else {
				dev->stepL1 = 11;
			}
		}
		break;
	case 7:// continuous timed mode
		st = I2C_ReadBytes(_i2c, dev->addr, OSC_CALIBRATE_VAL, (uint8_t *)&dev->tmp16, 2);// VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin
		if (st) {
			if (dev->tmp16) {
				dev->tmp16 *= period_ms;
			}
			dev->stepL1 = 8;
		}
		break;
	case 8:
		st = I2C_WriteBytes(_i2c, dev->addr, SYSTEM_INTERMEASUREMENT_PERIOD, (uint8_t *)&dev->tmp16, 2);
		if (st) {
			dev->stepL1 = 10;
		}
		break;
	case 10:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSRANGE_START, 0x04);  // VL53L0X_REG_SYSRANGE_MODE_TIMED
		if (st) {
			dev->stepL1 = 0;
			return 1;
		}
		break;
	case 11:
		// continuous back-to-back mode
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSRANGE_START, 0x02);  // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
		if (st) {
			dev->stepL1 = 0;
			return 1;
		}
		break;
	default:
		dev->stepL1 = 0;
		break;
	}
	return 0;
}
/*****************************************************************
  * @brief	OK
  * @param
  * @retval
  */
uint8_t stopContinuous(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev)
{
	uint8_t st;
	switch (dev->stepL1) {
	case 0:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSRANGE_START, 0x01);  // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT
		if (st) {
			dev->stepL1 = 1;
		}
		break;
	case 1:
		st = I2C_WriteOneByte(_i2c, dev->addr, INTERNAL_TUNING_2, 0x01);
		if (st) {
			dev->stepL1 = 2;
		}
		break;
	case 2:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSRANGE_START, 0x00);
		if (st) {
			dev->stepL1 = 3;
		}
		break;
	case 3:
		st = I2C_WriteOneByte(_i2c, dev->addr, INTERNAL_TUNING_1, 0x00);
		if (st) {
			dev->stepL1 = 4;
		}
		break;
	case 4:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSRANGE_START, 0x01);
		if (st) {
			dev->stepL1 = 5;
		}
		break;
	case 5:
		st = I2C_WriteOneByte(_i2c, dev->addr, INTERNAL_TUNING_2, 0x00);
		if (st) {
			dev->stepL1 = 0;
			return 1;
		}
		break;
	default:
		dev->stepL1 = 0;
	}
	return 0;
}
/*****************************************************************
  * @brief	OK
  * @param
  * @retval
  */
uint8_t readRangeContinuousMillimeters(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev)
{
	uint8_t st;
	switch (dev->stepL1) {
	case 0:
		st = I2C_ReadOneByte(_i2c, dev->addr, RESULT_INTERRUPT_STATUS, &dev->tmp8);
		if (st){
			if ((dev->tmp8 & 0x07)) {
				dev->stepL1 = 1;
			}
			else if (checkTimeoutExpired(dev)) {//timeout
				dev->timeoutFlag = 1;
				//dev->range = 65535;
				dev->status = DEVICE_ERROR;
				dev->stepL1 = 2;
			}
		}
		break;
	case 1:
		st = I2C_ReadBytes(_i2c, dev->addr, RESULT_RANGE_STATUS + 10, (uint8_t *)&dev->range, 2);
		if (st) {
			dev->status = DEVICE_DONE;
			dev->stepL1 = 2;
		}
		break;
	case 2:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSTEM_INTERRUPT_CLEAR, 0x01);
		if (st) {
			startTimeout(dev);
			dev->stepL1 = 0;
			return 1;
		}
		break;
	default:
		dev->stepL1 = 0;
	}
	return 0;
}
/*****************************************************************
  * @brief	OK
  * @param
  * @retval
  */
uint8_t readRangeSingleMillimeters(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev)
{
	uint8_t st;
	switch (dev->stepL2) {
	case 0:
		st = I2C_WriteOneByte(_i2c, dev->addr, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
		if (st) {
			dev->stepL2 = 1;
		}
		break;
	case 1:
		st = I2C_WriteOneByte(_i2c, dev->addr, INTERNAL_TUNING_2, 0x01);
		if (st) {
			dev->stepL2 = 2;
		}
		break;
	case 2:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSRANGE_START, 0x00);
		if (st) {
			dev->stepL2 = 3;
		}
		break;
	case 3:
		st = I2C_WriteOneByte(_i2c, dev->addr, INTERNAL_TUNING_1, dev->stop_variable);
		if (st) {
			dev->stepL2 = 4;
		}
		break;
	case 4:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSRANGE_START, 0x01);
		if (st) {
			dev->stepL2 = 5;
		}
		break;
	case 5:
		st = I2C_WriteOneByte(_i2c, dev->addr, INTERNAL_TUNING_2, 0x00);
		if (st) {
			dev->stepL2 = 6;
		}
		break;
	case 6:
		st = I2C_WriteOneByte(_i2c, dev->addr, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
		if (st) {
			dev->stepL2 = 7;
		}
		break;
	case 7:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSRANGE_START, 0x01);
		if (st) {
			startTimeout(dev);
			dev->stepL2 = 8;
		}
		break;
	case 8:// "Wait until start bit has been cleared"
		st = I2C_ReadOneByte(_i2c, dev->addr, SYSRANGE_START, &dev->tmp8);
		if (st) {
			if ((dev->tmp8 & 0x01) == 0) {
				dev->stepL2 = 9;
			}
			else if (checkTimeoutExpired(dev)) {
				dev->stepL2 = 0;
				dev->status = DEVICE_FAULTH;
				return 1;
			}
		}
		break;
	case 9:
		st = readRangeContinuousMillimeters(_i2c, dev);
		if (st) {
			dev->stepL2 = 0;
			return 1;
		}
		break;
	default:
		dev->stepL2 = 0;
		return 0;
	}
	return 0;
}
//--------------------------------------------------------------
uint8_t VL_Init(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev) {
	uint8_t st;
	// VL53L0X_DataInit() begin
	switch (dev->stepL3) {
	case 0://+
		st = I2C_ReadOneByte(_i2c, dev->addr, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, &dev->tmp8);
		if (st) {
			dev->stepL3 = 1;
		}
		break;
	case 1:// always 2.8v
		st = I2C_WriteOneByte(_i2c, dev->addr, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, dev->tmp8 | 0x01);  // set bit 0
		if (st) {
			dev->stepL3 = 2;
		}
		break;
	case 2:  //Set I2C standard mode
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x88, 0x00);
		if (st) {
			dev->stepL3 = 3;
		}
		break;
	case 3:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x80, 0x01);
		if (st) {
			dev->stepL3 = 4;
		}
		break;
	case 4:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x01);
		if (st) {
			dev->stepL3 = 5;
		}
		break;
	case 5:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x00, 0x00);
		if (st) {
			dev->stepL3 = 6;
		}
		break;
	case 6://stop variable 0x11
		st = I2C_ReadOneByte(_i2c, dev->addr, 0x91, &dev->stop_variable);
		if (st) {
			dev->stepL3 = 7;
		}
		break;
	case 7:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x00, 0x01);
		if (st) {
			dev->stepL3 = 8;
		}
		break;
	case 8:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x00);
		if (st) {
			dev->stepL3 = 9;
		}
		break;
	case 9:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x80, 0x00);
		if (st) {
			dev->stepL3 = 10;
		}
		break;
	case 10:// disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks//+
		st = I2C_ReadOneByte(_i2c, dev->addr, MSRC_CONFIG_CONTROL, &dev->tmp8);
		if (st) {
			dev->stepL3 = 11;
		}
		break;//tmp = 0x32
	case 11:
		st = I2C_WriteOneByte(_i2c, dev->addr, MSRC_CONFIG_CONTROL, dev->tmp8 | 0x12);
		if (st) {
			dev->stepL3 = 12;
		}
		break;
	case 12:
		st = setSignalRateLimit(_i2c, dev, 32);
		if (st) {
			dev->stepL3 = 13;
		}
		break;
	case 13:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSTEM_SEQUENCE_CONFIG, 0xFF);
		if (st) {
			dev->stepL3 = 14;
		}
		break;
		// VL53L0X_DataInit() end
		// VL53L0X_StaticInit() begin
		// The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
		// the API, but the same data seems to be more easily readable from
		// GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
	case 14:
		st = getSpadInfo(_i2c, dev);
		if (st) {
			dev->stepL3 = 15;
		}
		break;
	case 15:
		st = I2C_ReadBytes(_i2c, dev->addr, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, (uint8_t *)&dev->ref_spad_map, 6);
		if (st) {
			dev->stepL3 = 16;
		}
		break;
		// -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)
	case 16:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x01);
		if (st) {
			dev->stepL3 = 17;
		}
		break;
	case 17:
		st = I2C_WriteOneByte(_i2c, dev->addr, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
		if (st) {
			dev->stepL3 = 18;
		}
		break;
	case 18:
		st = I2C_WriteOneByte(_i2c, dev->addr, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
		if (st) {
			dev->stepL3 = 19;
		}
		break;
	case 19:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x00);
		if (st) {
			dev->stepL3 = 20;
		}
		break;
	case 20:
		st = I2C_WriteOneByte(_i2c, dev->addr, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);
		if (st) {
			uint8_t first_spad_to_enable = dev->spad_type_is_aperture ? 12 : 0;  // 12 is the first aperture spad
			uint8_t spads_enabled = 0;
			for (uint8_t i = 0; i < 48; i++) {
				if (i < first_spad_to_enable || spads_enabled == dev->spad_count) {
					// This bit is lower than the first one that should be enabled, or
					// (reference_spad_count) bits have already been enabled, so zero this bit
					dev->ref_spad_map[i / 8] &= ~(1 << (i % 7));
				}
				else if ((dev->ref_spad_map[i / 8] >> (i % 7)) & 0x1) {
					spads_enabled++;
				}
			}
			dev->stepL3 = 21;
		}
		break;
	case 21:
		st = I2C_WriteBytes(_i2c, dev->addr, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, dev->ref_spad_map, 6);
		if (st) {
			dev->stepL3 = 22;
		}
		break;
		// -- VL53L0X_set_reference_spads() end

		// -- VL53L0X_load_tuning_settings() begin
		// DefaultTuningSettings from vl53l0x_tuning.h
	case 22:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x01);
		if (st) {
			dev->stepL3 = 23;
		}
		break;
	case 23:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x00, 0x00);
		if (st) {
			dev->stepL3 = 24;
		}
		break;
	case 24:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x00);
		if (st) {
			dev->stepL3 = 25;
		}
		break;
	case 25:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x09, 0x00);
		if (st) {
			dev->stepL3 = 26;
		}
		break;
	case 26:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x10, 0x00);
		if (st) {
			dev->stepL3 = 27;
		}
		break;
	case 27:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x11, 0x00);
		if (st) {
			dev->stepL3 = 28;
		}
		break;//+
	case 28:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x24, 0x01);
		if (st) {
			dev->stepL3 = 29;
		}
		break;
	case 29:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x25, 0xFF);
		if (st) {
			dev->stepL3 = 30;
		}
		break;
	case 30:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x75, 0x00);
		if (st) {
			dev->stepL3 = 31;
		}
		break;
	case 31:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x01);
		if (st) {
			dev->stepL3 = 32;
		}
		break;
	case 32:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x4E, 0x2C);
		if (st) {
			dev->stepL3 = 33;
		}
		break;
	case 33:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x48, 0x00);
		if (st) {
			dev->stepL3 = 34;
		}
		break;
	case 34:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x30, 0x20);
		if (st) {
			dev->stepL3 = 35;
		}
		break;
	case 35:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x00);
		if (st) {
			dev->stepL3 = 36;
		}
		break;
	case 36:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x30, 0x09);
		if (st) {
			dev->stepL3 = 37;
		}
		break;
	case 37:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x54, 0x00);
		if (st) {
			dev->stepL3 = 38;
		}
		break;
	case 38:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x31, 0x04);
		if (st) {
			dev->stepL3 = 39;
		}
		break;
	case 39:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x32, 0x03);
		if (st) {
			dev->stepL3 = 40;
		}
		break;
	case 40:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x40, 0x83);
		if (st) {
			dev->stepL3 = 41;
		}
		break;
	case 41:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x46, 0x25);
		if (st) {
			dev->stepL3 = 42;
		}
		break;
	case 42:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x60, 0x00);
		if (st) {
			dev->stepL3 = 43;
		}
		break;
	case 43:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x27, 0x00);
		if (st) {
			dev->stepL3 = 44;
		}
		break;
	case 44:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x50, 0x06);
		if (st) {
			dev->stepL3 = 45;
		}
		break;
	case 45:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x51, 0x00);
		if (st) {
			dev->stepL3 = 46;
		}
		break;
	case 46:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x52, 0x96);
		if (st) {
			dev->stepL3 = 47;
		}
		break;
	case 47:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x56, 0x08);
		if (st) {
			dev->stepL3 = 48;
		}
		break;
	case 48:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x57, 0x30);
		if (st) {
			dev->stepL3 = 49;
		}
		break;
	case 49:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x61, 0x00);
		if (st) {
			dev->stepL3 = 50;
		}
		break;
	case 50:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x62, 0x00);
		if (st) {
			dev->stepL3 = 51;
		}
		break;
	case 51:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x64, 0x00);
		if (st) {
			dev->stepL3 = 52;
		}
		break;
	case 52:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x65, 0x00);
		if (st) {
			dev->stepL3 = 53;
		}
		break;
	case 53:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x66, 0xA0);
		if (st) {
			dev->stepL3 = 54;
		}
		break;
	case 54:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x01);
		if (st) {
			dev->stepL3 = 55;
		}
		break;
	case 55:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x22, 0x32);
		if (st) {
			dev->stepL3 = 56;
		}
		break;
	case 56:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x47, 0x14);
		if (st) {
			dev->stepL3 = 57;
		}
		break;
	case 57:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x49, 0xFF);
		if (st) {
			dev->stepL3 = 58;
		}
		break;
	case 58:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x4A, 0x00);
		if (st) {
			dev->stepL3 = 59;
		}
		break;
	case 59:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x00);
		if (st) {
			dev->stepL3 = 60;
		}
		break;
	case 60:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x7A, 0x0A);
		if (st) {
			dev->stepL3 = 61;
		}
		break;
	case 61:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x7B, 0x00);
		if (st) {
			dev->stepL3 = 62;
		}
		break;
	case 62:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x78, 0x21);
		if (st) {
			dev->stepL3 = 63;
		}
		break;
	case 63:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x01);
		if (st) {
			dev->stepL3 = 64;
		}
		break;
	case 64:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x23, 0x34);
		if (st) {
			dev->stepL3 = 65;
		}
		break;
	case 65:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x42, 0x00);
		if (st) {
			dev->stepL3 = 66;
		}
		break;
	case 66:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x44, 0xFF);
		if (st) {
			dev->stepL3 = 67;
		}
		break;
	case 67:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x45, 0x26);
		if (st) {
			dev->stepL3 = 68;
		}
		break;
	case 68:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x46, 0x05);
		if (st) {
			dev->stepL3 = 69;
		}
		break;
	case 69:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x40, 0x40);
		if (st) {
			dev->stepL3 = 70;
		}
		break;
	case 70:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x0E, 0x06);
		if (st) {
			dev->stepL3 = 71;
		}
		break;
	case 71:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x20, 0x1A);
		if (st) {
			dev->stepL3 = 72;
		}
		break;
	case 72:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x43, 0x40);
		if (st) {
			dev->stepL3 = 73;
		}
		break;
	case 73:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x00);
		if (st) {
			dev->stepL3 = 74;
		}
		break;
	case 74:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x34, 0x03);
		if (st) {
			dev->stepL3 = 75;
		}
		break;
	case 75:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x35, 0x44);
		if (st) {
			dev->stepL3 = 76;
		}
		break;
	case 76:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x01);
		if (st) {
			dev->stepL3 = 77;
		}
		break;
	case 77:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x31, 0x04);
		if (st) {
			dev->stepL3 = 78;
		}
		break;
	case 78:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x4B, 0x09);
		if (st) {
			dev->stepL3 = 79;
		}
		break;
	case 79:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x4C, 0x05);
		if (st) {
			dev->stepL3 = 80;
		}
		break;
	case 80:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x4D, 0x04);
		if (st) {
			dev->stepL3 = 81;
		}
		break;
	case 81:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x00);
		if (st) {
			dev->stepL3 = 82;
		}
		break;
	case 82:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x44, 0x00);
		if (st) {
			dev->stepL3 = 83;
		}
		break;
	case 83:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x45, 0x20);
		if (st) {
			dev->stepL3 = 84;
		}
		break;
	case 84:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x47, 0x08);
		if (st) {
			dev->stepL3 = 85;
		}
		break;
	case 85:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x48, 0x28);
		if (st) {
			dev->stepL3 = 86;
		}
		break;
	case 86:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x67, 0x00);
		if (st) {
			dev->stepL3 = 87;
		}
		break;
	case 87:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x70, 0x04);
		if (st) {
			dev->stepL3 = 88;
		}
		break;
	case 88:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x71, 0x01);
		if (st) {
			dev->stepL3 = 89;
		}
		break;
	case 89:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x72, 0xFE);
		if (st) {
			dev->stepL3 = 90;
		}
		break;
	case 90:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x76, 0x00);
		if (st) {
			dev->stepL3 = 91;
		}
		break;
	case 91:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x77, 0x00);
		if (st) {
			dev->stepL3 = 92;
		}
		break;
	case 92:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x01);
		if (st) {
			dev->stepL3 = 93;
		}
		break;
	case 93:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x0D, 0x01);
		if (st) {
			dev->stepL3 = 94;
		}
		break;
	case 94:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x00);
		if (st) {
			dev->stepL3 = 95;
		}
		break;
	case 95:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x80, 0x01);
		if (st) {
			dev->stepL3 = 96;
		}
		break;
	case 96:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x01, 0xF8);
		if (st) {
			dev->stepL3 = 97;
		}
		break;
	case 97:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x01);
		if (st) {
			dev->stepL3 = 98;
		}
		break;
	case 98:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x8E, 0x01);
		if (st) {
			dev->stepL3 = 99;
		}
		break;
	case 99:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x00, 0x01);
		if (st) {
			dev->stepL3 = 100;
		}
		break;
	case 100:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0xFF, 0x00);
		if (st) {
			dev->stepL3 = 101;
		}
		break;
	case 101:
		st = I2C_WriteOneByte(_i2c, dev->addr, 0x80, 0x00);
		if (st) {
			dev->stepL3 = 102;
		}
		break;
	case 102:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
		if (st) {
			dev->stepL3 = 103;
		}
		break;
	case 103:
		st = I2C_ReadOneByte(_i2c, dev->addr, GPIO_HV_MUX_ACTIVE_HIGH, &dev->tmp8);
		if (st) {
			dev->stepL3 = 104;
		}
		break;
	case 104:
		st = I2C_WriteOneByte(_i2c, dev->addr, GPIO_HV_MUX_ACTIVE_HIGH, dev->tmp8 & ~0x10);  // active low
		if (st) {
			dev->stepL3 = 105;
		}
		break;
	case 105:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSTEM_INTERRUPT_CLEAR, 0x01);
		if (st) {
			dev->stepL3 = 106;
		}
		break;
		// -- VL53L0X_SetGpioConfig() end

	case 106:
		st = getMeasurementTimingBudget(_i2c, dev);
		if (st) {
			dev->stepL3 = 107;
		}
		break;
		// "Disable MSRC and TCC by default"
		// MSRC = Minimum Signal Rate Check
		// TCC = Target CentreCheck
		// -- VL53L0X_SetSequenceStepEnable() begin
	case 107:
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSTEM_SEQUENCE_CONFIG, 0xE8);
		if (st) {
			dev->stepL3 = 108;
		}
		break;
		// -- VL53L0X_SetSequenceStepEnable() end

	case 108:  // "Recalculate timing budget"
		st = setMeasurementTimingBudget(_i2c, dev, dev->measurement_timing_budget_us);
		if (st) {
			dev->stepL3 = 109;
		}
		break;
		// VL53L0X_StaticInit() end
		// VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())
	case 109:  // -- VL53L0X_perform_vhv_calibration() begin
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSTEM_SEQUENCE_CONFIG, 0x01);
		if (st) {
			dev->stepL3 = 110;
		}
		break;
	case 110:  //  performSingleRefCalibration(_i2c,dev,0x40);
		st = performSingleRefCalibration(_i2c, dev, 0x40);
		if (st) {
			dev->stepL3 = 111;
		}
		break;
		// -- VL53L0X_perform_vhv_calibration() end
	case 111:  // -- VL53L0X_perform_phase_calibration() begin
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSTEM_SEQUENCE_CONFIG, 0x02);
		if (st) {
			dev->stepL3 = 112;
		}
		break;
	case 112:  //  performSingleRefCalibration(_i2c,dev,0x00);
		st = performSingleRefCalibration(_i2c, dev, 0x00);
		if (st) {
			dev->stepL3 = 113;
		}
		break;
		// -- VL53L0X_perform_phase_calibration() end
	case 113:  // "restore the previous Sequence Config"
		st = I2C_WriteOneByte(_i2c, dev->addr, SYSTEM_SEQUENCE_CONFIG, 0xE8);
		if (st) {
			dev->stepL3 = 0;
			setTimeout(dev, 50);
			startTimeout(dev);
			return 1;
		}
		break;
		// VL53L0X_PerformRefCalibration() end
	default:
		dev->stepL3 = 0;
		break;
	}
	return 0;
}

uint16_t getSmoothRange(VL53L0x_t *dev) {
	static_assert(VL53L0x_SHOOTH_DEEP % 2 == 0, "smooth deep most be power of 2");
	dev->smoothRange = alphabeta(dev->range, dev->smoothRange, VL53L0x_SHOOTH_DEEP);
	return dev->smoothRange;
}

