/*********************************************************************************
ginal author: Alexandr Pochtovy<alex.mail.prime@gmail.com>

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

//================================================================
// STATIC INLINE FUNCTIONS
//================================================================

/*****************************************************************
  * @brief
  * @param
  * @retval
  */

/*****************************************************************
  * @brief  Encode VCSEL pulse period register value from period in PCLKs
  * based on VL53L0X_encode_vcsel_period()
  * @param  uint8_t period_pclks in clks
  * @retval uint8_t period in microsec
  */
/* */
static inline uint8_t __encodeVcselPeriod(uint8_t period_pclks)//check
{
	return (((period_pclks) >> 1) - 1);
}
/*****************************************************************
  * @briefDecode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
 from register value
 based on VL53L0X_decode_vcsel_period()
  * @param
  * @retval
  */
static inline uint8_t __decodeVcselPeriod(uint8_t reg_val)//check
{
	return (((reg_val) + 1) << 1);
}
/*****************************************************************
  * @brief Calculate macro period in nanoseconds from VCSEL period in PCLKs
 based on VL53L0X_calc_macro_period_ps()
 PLL_period_ps = 1655; macro_period_vclks = 2304
  * @param
  * @retval
  */
static inline uint32_t __calcMacroPeriod(uint8_t vcsel_period_pclks)//check
{
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

/********************************************************************
*		SIMPLE GETTERS AND SETTERS									*
*********************************************************************/
/*****************************************************************
  * @brief:
  * @param
  * @retval
  */
void setupVL53L0X(VL53L0X *lidar, uint16_t timeout) {
	lidar->addr = VL53L0x_ADDR_DEFAULT;  // ADDRESS_DEFAULT;
	lidar->limit_timeout = timeout;
	lidar->count_timeout = 0;
	lidar->timeoutFlag = 0;
}
/*****************************************************************
  * @brief
  * @param
  * @retval
  */
uint8_t setAddress(I2C_IRQ_Conn_t *_i2c, VL53L0X *lidar, const uint8_t new_addr)
{
	uint8_t st;
	st = WriteOneRegByte(_i2c, lidar->addr, CHIP_I2C_ADDRESS, new_addr & 0x7F);
	if (st) {
		lidar->addr = new_addr;
	}
	return st;
}
/*****************************************************************
  * @brief
  * @param
  * @retval
  */
uint8_t getAddress(VL53L0X *lidar)
{
	return lidar->addr;
}
/*****************************************************************
  * @brief
  * @param
  * @retval
  */
uint8_t getModelId(I2C_IRQ_Conn_t *_i2c, VL53L0X *lidar)
{
	uint8_t st;
	st = ReadOneRegByte(_i2c, lidar->addr, IDENTIFICATION_MODEL_ID, &lidar->modelID);
	return st;
}
/*****************************************************************
  * @brief
  * @param
  * @retval
  */
uint8_t getRevisionId(I2C_IRQ_Conn_t *_i2c, VL53L0X *lidar)
{
	uint8_t st;
	st = ReadOneRegByte(_i2c, lidar->addr, IDENTIFICATION_REVISION_ID, &lidar->revisionID);
	return st;
}
/*****************************************************************
  * @brief
  * @param
  * @retval
  */
void setTimeout(VL53L0X *lidar, const uint16_t timeout)
{
	lidar->limit_timeout = timeout;
}

uint16_t getTimeout(VL53L0X *lidar)
{
	return lidar->count_timeout;
}

void startTimeout(VL53L0X *lidar)
{
	lidar->count_timeout = 0;
}
/*****************************************************************
  * @brief  Check if timeout is enabled (set to nonzero value) and has expired
  * @param
  * @retval
  */
uint8_t checkTimeoutExpired(VL53L0X *lidar)  //+
{
	lidar->timeoutFlag = ++lidar->count_timeout > lidar->limit_timeout;
	return lidar->timeoutFlag;
}

// ===============================================================
// INTERNAL FUNCTIONS
// ===============================================================
/*****************************************************************
  * @brief  OK
  * @param
  * @retval
  */
uint8_t setSignalRateLimit(I2C_IRQ_Conn_t *_i2c, VL53L0X *lidar, uint32_t limit_Mcps)
{
	uint8_t st;
	uint16_t data = __Min(__Max((uint16_t)limit_Mcps, 65535), 32);
	st = WriteRegBytes(_i2c, lidar->addr, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (uint8_t* )&data, 2);
	return st;
}
/*****************************************************************
  * @brief  OK
  * @param
  * @retval
  */
uint8_t getSpadInfo(I2C_IRQ_Conn_t *_i2c,VL53L0X *lidar)
{
	uint8_t st;
	switch (lidar->stepL1) {
	case 0:
		st = WriteOneRegByte(_i2c, lidar->addr, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
		if (st) {
			lidar->stepL1 = 1;
		}
		break;
	case 1:
		st = WriteOneRegByte(_i2c, lidar->addr, INTERNAL_TUNING_2, 0x01);
		if (st) {
			lidar->stepL1 = 2;
		}
		break;
	case 2:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSRANGE_START, 0x00);
		if (st) {
			lidar->stepL1 = 3;
		}
		break;
	case 3:
		st = WriteOneRegByte(_i2c, lidar->addr, INTERNAL_TUNING_2, 0x06);
		if (st) {
			lidar->stepL1 = 4;
		}
		break;
	case 4:
		st = ReadOneRegByte(_i2c, lidar->addr, SOMETHING_MAGIC_REG, &lidar->tmp8);
		if (st) {
			lidar->tmp8 |= 0x04;
			lidar->stepL1 = 5;
		}
		break;
	case 5:
		st = WriteOneRegByte(_i2c, lidar->addr, SOMETHING_MAGIC_REG, lidar->tmp8);
		if (st) {
			lidar->stepL1 = 6;
		}
		break;
	case 6:
		st = WriteOneRegByte(_i2c, lidar->addr, INTERNAL_TUNING_2, 0x07);
		if (st) {
			lidar->stepL1 = 7;
		}
		break;
	case 7:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSTEM_HISTOGRAM_BIN, 0x01);
		if (st) {
			lidar->stepL1 = 8;
		}
		break;
	case 8:
		st = WriteOneRegByte(_i2c, lidar->addr, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
		if (st) {
			lidar->stepL1 = 9;
		}
		;
		break;
	case 9:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x94, 0x6b);
		if (st) {
			lidar->stepL1 = 10;
		}
		break;
	case 10:
		st = WriteOneRegByte(_i2c, lidar->addr, SOMETHING_MAGIC_REG, 0x00);
		if (st) {
			setTimeout(lidar, 100);
			startTimeout(lidar);
			lidar->stepL1 = 11;
		}
		break;
	case 11:
		st = ReadOneRegByte(_i2c, lidar->addr, SOMETHING_MAGIC_REG, &lidar->tmp8);
		if (st) {
			if (lidar->tmp8 != 0x00) {
				lidar->stepL1 = 12;
			}
			else if (checkTimeoutExpired(lidar)) {
				lidar->stepL1 = 0;
				lidar->status = DEVICE_FAULTH;
				return 1;
			}
		}
		break;
	case 12:
		st = WriteOneRegByte(_i2c, lidar->addr, SOMETHING_MAGIC_REG, 0x01);
		if (st) {
			lidar->stepL1 = 13;
		}
		break;
	case 13:
		st = ReadOneRegByte(_i2c, lidar->addr, 0x92, &lidar->tmp8);
		if (st) {
			lidar->spad_count = lidar->tmp8 & 0x7f;
			lidar->spad_type_is_aperture = (lidar->tmp8 >> 7) & 0x01;
			lidar->stepL1 = 14;
		}
		break;
	case 14:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSTEM_HISTOGRAM_BIN, 0x00);
		if (st) {
			lidar->stepL1 = 15;
		}
		break;
	case 15:
		st = WriteOneRegByte(_i2c, lidar->addr, INTERNAL_TUNING_2, 0x06);
		if (st) {
			lidar->stepL1 = 16;
		}
		break;
	case 16:
		st = ReadOneRegByte(_i2c, lidar->addr, SOMETHING_MAGIC_REG, &lidar->tmp8);
		if (st) {
			lidar->tmp8 &= ~0x04;
			lidar->stepL1 = 17;
		}
		break;
	case 17:
		st = WriteOneRegByte(_i2c, lidar->addr, SOMETHING_MAGIC_REG, lidar->tmp8);
		if (st) {
			lidar->stepL1 = 18;
		}
		break;
	case 18:
		st = WriteOneRegByte(_i2c, lidar->addr, INTERNAL_TUNING_2, 0x01);
		if (st) {
			lidar->stepL1 = 19;
		}
		break;
	case 19:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSRANGE_START, 0x01);
		if (st) {
			lidar->stepL1 = 20;
		}
		break;
	case 20:
		st = WriteOneRegByte(_i2c, lidar->addr, INTERNAL_TUNING_2, 0x00);
		if (st) {
			lidar->stepL1 = 21;
		}
		break;
	case 21:
		st = WriteOneRegByte(_i2c, lidar->addr, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
		if (st) {
			lidar->stepL1 = 0;
			return 1;
		}
		break;
	default:
		lidar->stepL1 = 0;
		break;
	}
	return 0;
}
/*****************************************************************
  * @brief  OK
  * @param
  * @retval
  */
uint8_t getSequenceStepEnables(I2C_IRQ_Conn_t *_i2c,VL53L0X *lidar)
{
	uint8_t st;
	uint8_t sequence_config;
	st = ReadOneRegByte(_i2c, lidar->addr, SYSTEM_SEQUENCE_CONFIG, &sequence_config);
	if (st) {
		lidar->enables.tcc = (sequence_config >> 4) & 0x1;
		lidar->enables.dss = (sequence_config >> 3) & 0x1;
		lidar->enables.msrc = (sequence_config >> 2) & 0x1;
		lidar->enables.pre_range = (sequence_config >> 6) & 0x1;
		lidar->enables.final_range = (sequence_config >> 7) & 0x1;
	}
	return st;
}
/*****************************************************************
  * @brief	OK
  * @param
  * @retval
  */
uint8_t getVcselPulsePeriod(I2C_IRQ_Conn_t *_i2c, VL53L0X *lidar, vcselPeriodType type)
{
	uint8_t st = 0;
	if (type == VcselPeriodPreRange) {
		st = ReadOneRegByte(_i2c, lidar->addr, PRE_RANGE_CONFIG_VCSEL_PERIOD, &lidar->tmp8);
		if (st) {
			lidar->vcselPeriodValue = __decodeVcselPeriod(lidar->tmp8);
		}
	}
	else if (type == VcselPeriodFinalRange) {
		st = ReadOneRegByte(_i2c, lidar->addr, FINAL_RANGE_CONFIG_VCSEL_PERIOD, &lidar->tmp8);
		if (st) {
			lidar->vcselPeriodValue = __decodeVcselPeriod(lidar->tmp8);
		}
	}
	else {
		lidar->vcselPeriodValue = 255;
	}
	return st;
}
/*****************************************************************
  * @brief	OK
  * @param
  * @retval
  */
uint8_t getSequenceStepTimeouts(I2C_IRQ_Conn_t *_i2c,VL53L0X *lidar)
{
	uint8_t st;
	switch (lidar->stepL1) {
	case 0:
		st = getVcselPulsePeriod(_i2c, lidar, VcselPeriodPreRange);
		if (st) {
			lidar->timeouts.pre_range_vcsel_period_pclks = lidar->vcselPeriodValue;
			lidar->stepL1 = 1;
		}
		break;
	case 1:
		st = ReadOneRegByte(_i2c, lidar->addr, MSRC_CONFIG_TIMEOUT_MACROP, &lidar->tmp8);
		if (st) {
			lidar->timeouts.msrc_dss_tcc_mclks = lidar->tmp8 + 1;
			lidar->timeouts.msrc_dss_tcc_us = timeoutMclksToMicroseconds(
					lidar->timeouts.msrc_dss_tcc_mclks, lidar->timeouts.pre_range_vcsel_period_pclks);
			lidar->stepL1 = 2;
		}
		break;
	case 2:
		st = ReadRegBytes(_i2c, lidar->addr, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, (uint8_t *)&lidar->tmp16, 2);
		if (st) {
			lidar->timeouts.pre_range_mclks = decodeTimeout(lidar->tmp16);
			lidar->timeouts.pre_range_us = timeoutMclksToMicroseconds(lidar->timeouts.pre_range_mclks, lidar->timeouts.pre_range_vcsel_period_pclks);
			lidar->stepL1 = 3;
		}
		break;
	case 3:
		st = getVcselPulsePeriod(_i2c, lidar, VcselPeriodFinalRange);
		if (st) {
			lidar->timeouts.final_range_vcsel_period_pclks= lidar->vcselPeriodValue;
			lidar->stepL1 = 4;
		}
		break;
	case 4:
		st = ReadRegBytes(_i2c, lidar->addr, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, (uint8_t *)&lidar->tmp16, 2);
		if (st) {
			lidar->timeouts.final_range_mclks = decodeTimeout(lidar->tmp16);
			if (lidar->enables.pre_range) {
				lidar->timeouts.final_range_mclks -= lidar->timeouts.pre_range_mclks;
			}
			lidar->timeouts.final_range_us = timeoutMclksToMicroseconds(lidar->timeouts.final_range_mclks, lidar->timeouts.final_range_vcsel_period_pclks);
			lidar->stepL1 = 0;
			return 1;
		}
		break;
	default:
		lidar->stepL1 = 0;
		break;
	}
	return 0;
}
/*****************************************************************
  * @brief
  * @param
  * @retval
  */
uint8_t getMeasurementTimingBudget(I2C_IRQ_Conn_t *_i2c, VL53L0X *lidar)
{
	uint8_t st;
	uint16_t const StartOverhead = 1910;  // note that this is different than the value in set_
	uint16_t const EndOverhead = 960;
	uint16_t const MsrcOverhead = 660;
	uint16_t const TccOverhead = 590;
	uint16_t const DssOverhead = 690;
	uint16_t const PreRangeOverhead = 660;
	uint16_t const FinalRangeOverhead = 550;
	switch (lidar->stepL2) {
	case 0:
		// "Start and end overhead times always present"
		lidar->measurement_timing_budget_us = StartOverhead + EndOverhead;
		lidar->stepL2 = 1; // @suppress("No break at end of case")
		//break;
	case 1:
		st = getSequenceStepEnables(_i2c, lidar);
		if (st) {
			lidar->stepL2 = 2;
		}
		break;
	case 2:
		st = getSequenceStepTimeouts(_i2c, lidar);
		if (st) {
			lidar->stepL2 = 3;
		}
		break;
	case 3:
		if (lidar->enables.tcc) {
			lidar->measurement_timing_budget_us += (lidar->timeouts.msrc_dss_tcc_us + TccOverhead);
		}
		if (lidar->enables.dss) {
			lidar->measurement_timing_budget_us += 2 * (lidar->timeouts.msrc_dss_tcc_us + DssOverhead);
		}
		else if (lidar->enables.msrc) {
			lidar->measurement_timing_budget_us += (lidar->timeouts.msrc_dss_tcc_us + MsrcOverhead);
		}
		if (lidar->enables.pre_range) {
			lidar->measurement_timing_budget_us += (lidar->timeouts.pre_range_us + PreRangeOverhead);
		}
		if (lidar->enables.final_range) {
			lidar->measurement_timing_budget_us += (lidar->timeouts.final_range_us + FinalRangeOverhead);
		}
		lidar->stepL2 = 0;
		return 1;
	default:
		lidar->stepL2 = 0;
		break;
	}
	return 0;
}
/*****************************************************************
  * @brief OK/2!!!!!!!
  * @param
  * @retval
  */
uint8_t setMeasurementTimingBudget(I2C_IRQ_Conn_t *_i2c, VL53L0X *lidar, uint32_t budget_us)//check
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
	
	switch (lidar->stepL2) {
	case 0:
		lidar->sp_budget_us = budget_us;
		if (lidar->sp_budget_us < MinTimingBudget) { lidar->sp_budget_us = MinTimingBudget; }
	case 1:
		lidar->measurement_timing_budget_us = StartOverhead + EndOverhead;
		lidar->sp_budget_us += 1;
		lidar->stepL2 = 2;
		break;
	case 2:
		st = getSequenceStepEnables(_i2c, lidar);
		if (st) {
			lidar->stepL2 = 3;
		}
		break;
	case 3:
		st = getSequenceStepTimeouts(_i2c, lidar);
		if (st) {
			lidar->stepL2 = 4;
		}
		break;
	case 4:
		if (lidar->enables.tcc) {
			lidar->measurement_timing_budget_us += (lidar->timeouts.msrc_dss_tcc_us + TccOverhead);
		}
		if (lidar->enables.dss) {
			lidar->measurement_timing_budget_us += 2 * (lidar->timeouts.msrc_dss_tcc_us + DssOverhead);
		}
		else if (lidar->enables.msrc) {
			lidar->measurement_timing_budget_us += (lidar->timeouts.msrc_dss_tcc_us + MsrcOverhead);
		}
		if (lidar->enables.pre_range) {
			lidar->measurement_timing_budget_us += (lidar->timeouts.pre_range_us + PreRangeOverhead);
		}
		if (lidar->enables.final_range) {
			lidar->measurement_timing_budget_us += FinalRangeOverhead;
			// "Note that the final range timeout is determined by the timing
			// budget and the sum of all other timeouts within the sequence.
			// If there is no room for the final range timeout, then an error
			// will be set. Otherwise the remaining time will be applied to
			// the final range."
			if (lidar->measurement_timing_budget_us > lidar->sp_budget_us) {//Requested timeout too small
				lidar->stepL2 = 1;
				break;
			}
			uint32_t final_range_timeout_us = lidar->sp_budget_us - lidar->measurement_timing_budget_us;
			// set_sequence_step_timeout() begin
			// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)
			// "For the final range timeout, the pre-range timeout
			//  must be added. To do this both final and pre-range
			//  timeouts must be expressed in macro periods MClks
			//  because they have different vcsel periods."
			uint16_t final_range_timeout_mclks = timeoutMicrosecondsToMclks(final_range_timeout_us, lidar->timeouts.final_range_vcsel_period_pclks);
			if (lidar->enables.pre_range) {
				final_range_timeout_mclks += lidar->timeouts.pre_range_mclks;
			}
			lidar->tmp16 = encodeTimeout(final_range_timeout_mclks);
			lidar->stepL2 = 5;
		}
		else {
			lidar->status = DEVICE_FAULTH;
			lidar->stepL2 = 0;
			return 1;
		}
		break;
	case 5:
		st = WriteRegBytes(_i2c, lidar->addr, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, (uint8_t *)&lidar->tmp16, 2);
		st = 1;
		if (st) {
			// set_sequence_step_timeout() end
			lidar->measurement_timing_budget_us = budget_us;  // store for internal reuse
			lidar->stepL2 = 0;
			return 1;
		}
		break;
	default:
		lidar->stepL2 = 0;
		break;
	}
	return 0;
}
/*****************************************************************
  * @brief	OK
  * @param
  * @retval
  */
uint8_t performSingleRefCalibration(I2C_IRQ_Conn_t *_i2c, VL53L0X *lidar, uint8_t vhv_init_byte)
{
	uint8_t st;
	switch (lidar->stepL1) {
	case 0:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSRANGE_START, 0x01 | vhv_init_byte);// VL53L0X_REG_SYSRANGE_MODE_START_STOP
		if (st) {
			startTimeout(lidar);
			lidar->stepL1 = 1;
		}
		break;
	case 1:
		st = ReadOneRegByte(_i2c, lidar->addr, RESULT_INTERRUPT_STATUS, &lidar->tmp8);
		if (st) {
			if ((lidar->tmp8 & 0x07) != 0) {
				lidar->stepL1 = 2;
			}
			else if (checkTimeoutExpired(lidar)) {
				lidar->status = DEVICE_FAULTH;
				lidar->stepL1 = 2;
			}
		}
		break;
	case 2:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSTEM_INTERRUPT_CLEAR, 0x01);
		if (st) {
			lidar->stepL1 = 3;
		}
		break;
	case 3:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSRANGE_START, 0x00);
		if (st) {
			lidar->stepL1 = 0;
			return 1;
		}
		break;
	default:
		lidar->stepL1 = 0;
		break;
	}
	return 0;
}
/*****************************************************************
  * @brief	
  * @param
  * @retval
  */
uint8_t setVcselPulsePeriod(I2C_IRQ_Conn_t *_i2c,VL53L0X *lidar,vcselPeriodType type,uint8_t period_pclks)
{
	uint8_t st;
	switch (lidar->stepL3) {
	case 0:
		st = getSequenceStepEnables(_i2c, lidar);
		if (st) {
			lidar->stepL3 = 1;
		}
		break;
	case 1:
		st = getSequenceStepTimeouts(_i2c, lidar);
		if (st) {
			lidar->vcsel_period_reg = __encodeVcselPeriod(period_pclks);
			lidar->stepL3 = 2;
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
				lidar->stepL3 = 5;
				break;
			case 14:
				lidar->stepL3 = 6;
				break;
			case 16:
				lidar->stepL3 = 7;
				break;
			case 18:
				lidar->stepL3 = 8;
				break;
			default:// invalid period
				lidar->status = DEVICE_FAULTH;
				return 1;
			}
		}
		else if (type == VcselPeriodFinalRange) {
			switch (period_pclks) {
			case 8:
				lidar->stepL3 = 15;
				break;
			case 10:
				lidar->stepL3 = 25;
				break;
			case 12:
				lidar->stepL3 = 35;
				break;
			case 14:
				lidar->stepL3 = 45;
				break;
			default:
				lidar->status = DEVICE_FAULTH;// invalid period
				return 1;
			}
		}
		else {  // invalid type
			return 1;
		}
		break;
		//type == VcselPeriodPreRange
	case 5:  //period_pclks = 12
		st = WriteOneRegByte(_i2c, lidar->addr, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
		if (st) {
			lidar->stepL3 = 9;
		}
		break;
	case 6:  //period_pclks = 14
		st = WriteOneRegByte(_i2c, lidar->addr, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
		if (st) {
			lidar->stepL3 = 9;
		}
		break;
	case 7:  //period_pclks = 16
		st = WriteOneRegByte(_i2c, lidar->addr, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
		if (st) {
			lidar->stepL3 = 9;
		}
		break;
	case 8:  //period_pclks = 18
		st = WriteOneRegByte(_i2c, lidar->addr, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
		if (st) {
			lidar->stepL3 = 9;
		}
		break;
	case 9:
		st = WriteOneRegByte(_i2c, lidar->addr, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
		if (st) {
			lidar->stepL3 = 10;
		}
		break;
	case 10:// apply new VCSEL period
		st = WriteOneRegByte(_i2c, lidar->addr, PRE_RANGE_CONFIG_VCSEL_PERIOD, lidar->vcsel_period_reg);
		if (st) {
			// set_sequence_step_timeout() begin
			// (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)
			lidar->new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(lidar->timeouts.pre_range_us, period_pclks);
			lidar->tmp16 = encodeTimeout(lidar->new_pre_range_timeout_mclks);
			lidar->stepL3 = 11;
		}
		break;
	case 11:// update timeouts
		st = WriteRegBytes(_i2c, lidar->addr, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, (uint8_t *)&lidar->tmp16, 2);
		// set_sequence_step_timeout() end
		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)
		if (st) {
			lidar->stepL3 = 12;
		}
		break;
	case 12:
		lidar->new_msrc_timeout_mclks = timeoutMicrosecondsToMclks(lidar->timeouts.msrc_dss_tcc_us, period_pclks);
		lidar->tmp8 = lidar->new_msrc_timeout_mclks > 256 ? 255 : lidar->new_msrc_timeout_mclks - 1;
		st = WriteOneRegByte(_i2c, lidar->addr, MSRC_CONFIG_TIMEOUT_MACROP, lidar->tmp8);
		if (st) {
			lidar->stepL3 = 102;
		}
		break;// set_sequence_step_timeout() end

	//type == VcselPeriodFinalRange
	case 15:  //period_pclks = 8
		st = WriteOneRegByte(_i2c, lidar->addr, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
		if (st) {
			lidar->stepL3 = 16;
		}
		break;
	case 16:
		st = WriteOneRegByte(_i2c, lidar->addr, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
		if (st) {
			lidar->stepL3 = 17;
		}
		break;
	case 17:
		st = WriteOneRegByte(_i2c, lidar->addr, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
		if (st) {
			lidar->stepL3 = 18;
		}
		break;
	case 18:
		st = WriteOneRegByte(_i2c, lidar->addr, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
		if (st) {
			lidar->stepL3 = 19;
		}
		break;
	case 19:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x01);
		if (st) {
			lidar->stepL3 = 20;
		}
		break;
	case 20:
		st = WriteOneRegByte(_i2c, lidar->addr, ALGO_PHASECAL_LIM, 0x30);
		if (st) {
			lidar->stepL3 = 51;
		}
		break;

	case 25:  //period_pclks = 10
		st = WriteOneRegByte(_i2c, lidar->addr, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
		if (st) {
			lidar->stepL3 = 26;
		}
		break;
	case 26:
		st = WriteOneRegByte(_i2c, lidar->addr, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
		if (st) {
			lidar->stepL3 = 27;
		}
		break;
	case 27:
		st = WriteOneRegByte(_i2c, lidar->addr, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
		if (st) {
			lidar->stepL3 = 28;
		}
		break;
	case 28:
		st = WriteOneRegByte(_i2c, lidar->addr, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
		if (st) {
			lidar->stepL3 = 49;
		}
		break;

	case 35:  //period_pclks = 12
		st = WriteOneRegByte(_i2c, lidar->addr, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
		if (st) {
			lidar->stepL3 = 36;
		}
		break;
	case 36:
		st = WriteOneRegByte(_i2c, lidar->addr, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
		if (st) {
			lidar->stepL3 = 37;
		}
		break;
	case 37:
		st = WriteOneRegByte(_i2c, lidar->addr, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
		if (st) {
			lidar->stepL3 = 38;
		}
		break;
	case 38:
		st = WriteOneRegByte(_i2c, lidar->addr, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
		if (st) {
			lidar->stepL3 = 49;
		}
		break;

	case 45:  //period_pclks = 14
		st = WriteOneRegByte(_i2c, lidar->addr, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
		if (st) {
			lidar->stepL3 = 46;
		}
		break;
	case 46:
		st = WriteOneRegByte(_i2c, lidar->addr, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
		if (st) {
			lidar->stepL3 = 47;
		}
		break;
	case 47:
		st = WriteOneRegByte(_i2c, lidar->addr, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
		if (st) {
			lidar->stepL3 = 48;
		}
		break;
	case 48:
		st = WriteOneRegByte(_i2c, lidar->addr, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
		if (st) {
			lidar->stepL3 = 49;
		}
		break;
	case 49:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x01);
		if (st) {
			lidar->stepL3 = 50;
		}
		break;
	case 50:
		st = WriteOneRegByte(_i2c, lidar->addr, ALGO_PHASECAL_LIM, 0x20);
		if (st) {
			lidar->stepL3 = 51;
		}
		break;
	case 51:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x00);
		if (st) {
			lidar->stepL3 = 55;
		}
		break;
	case 55:  // apply new VCSEL period
		st = WriteOneRegByte(_i2c, lidar->addr, FINAL_RANGE_CONFIG_VCSEL_PERIOD, lidar->vcsel_period_reg);
		if (st) {
		// "For the final range timeout, the pre-range timeout
		//  must be added. To do this both final and pre-range
		//  timeouts must be expressed in macro periods MClks
		//  because they have different vcsel periods."
			lidar->new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(lidar->timeouts.final_range_us, period_pclks);
			if (lidar->enables.pre_range) {
				lidar->new_final_range_timeout_mclks += lidar->timeouts.pre_range_mclks;
			}
			lidar->tmp16 = encodeTimeout(lidar->new_final_range_timeout_mclks);
			lidar->stepL3 = 56;
		}
		break;
	case 56:// update timeouts
		// set_sequence_step_timeout() begin
		st = WriteRegBytes(_i2c, lidar->addr, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, (uint8_t *)&lidar->tmp16, 2);
		if (st) {
			lidar->stepL3 = 102;
		}
		break;
		// set_sequence_step_timeout end
	case 102:// "Finally, the timing budget must be re-applied"
		st = setMeasurementTimingBudget(_i2c, lidar, lidar->measurement_timing_budget_us);
		if (st) {
			lidar->stepL3 = 103;
		}
		break;
	case 103:
		st = ReadOneRegByte(_i2c, lidar->addr, SYSTEM_SEQUENCE_CONFIG, &lidar->sequence_config);
		if (st) {
			lidar->stepL3 = 104;
		}
		break;
	case 104:// "Perform the phase calibration. This is needed after changing on vcsel period."
		// VL53L0X_perform_phase_calibration() begin
		st = WriteOneRegByte(_i2c, lidar->addr, SYSTEM_SEQUENCE_CONFIG, 0x02);
		if (st) {
			lidar->stepL3 = 105;
		}
		break;
	case 105:
		st = performSingleRefCalibration(_i2c, lidar, 0x00);
		if (st) {
			lidar->stepL3 = 106;
		}
		break;
	case 106:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSTEM_SEQUENCE_CONFIG, lidar->sequence_config);
		if (st) {
			lidar->stepL3 = 0;
			return 1;
		}
		break;
		// VL53L0X_perform_phase_calibration() end
	default:
		lidar->stepL3 = 0;
		break;
	}
	return 0;
}

/*****************************************************************
  * @brief	OK
  * @param
  * @retval
  */
uint8_t startContinuous(I2C_IRQ_Conn_t *_i2c, VL53L0X *lidar, uint32_t period_ms)
{
	uint8_t st;
	switch (lidar->stepL1) {
	case 0:
		st = WriteOneRegByte(_i2c, lidar->addr, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
		if (st) {
			lidar->stepL1 = 1;
		}
		break;
	case 1:
		st = WriteOneRegByte(_i2c, lidar->addr, INTERNAL_TUNING_2, 0x01);
		if (st) {
			lidar->stepL1 = 2;
		}
		break;
	case 2:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSRANGE_START, 0x00);
		if (st) {
			lidar->stepL1 = 3;
		}
		break;
	case 3:
		st = WriteOneRegByte(_i2c, lidar->addr, INTERNAL_TUNING_1, lidar->stop_variable);
		if (st) {
			lidar->stepL1 = 4;
		}
		break;
	case 4:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSRANGE_START, 0x01);
		if (st) {
			lidar->stepL1 = 5;
		}
		break;
	case 5:
		st = WriteOneRegByte(_i2c, lidar->addr, INTERNAL_TUNING_2, 0x00);
		if (st) {
			lidar->stepL1 = 6;
		}
		break;
	case 6:
		st = WriteOneRegByte(_i2c, lidar->addr, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
		if (st) {
			if (period_ms) {
				lidar->stepL1 = 7;
			}
			else {
				lidar->stepL1 = 11;
			}
		}
		break;
	case 7:// continuous timed mode
		st = ReadRegBytes(_i2c, lidar->addr, OSC_CALIBRATE_VAL, &lidar->tmp16, 2);// VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin
		if (st) {
			if (lidar->tmp16) {
				lidar->tmp16 *= period_ms;
			}
			lidar->stepL1 = 8;
		}
		break;
	case 8:
		st = WriteRegBytes(_i2c, lidar->addr, SYSTEM_INTERMEASUREMENT_PERIOD, (uint8_t *)&lidar->tmp16, 2);
		if (st) {
			lidar->stepL1 = 10;
		}
		break;
	case 10:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSRANGE_START, 0x04);  // VL53L0X_REG_SYSRANGE_MODE_TIMED
		if (st) {
			lidar->stepL1 = 0;
			return 1;
		}
		break;
	case 11:
		// continuous back-to-back mode
		st = WriteOneRegByte(_i2c, lidar->addr, SYSRANGE_START, 0x02);  // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
		if (st) {
			lidar->stepL1 = 0;
			return 1;
		}
		break;
	default:
		lidar->stepL1 = 0;
		break;
	}
	return 0;
}
/*****************************************************************
  * @brief	OK
  * @param
  * @retval
  */
uint8_t stopContinuous(I2C_IRQ_Conn_t *_i2c, VL53L0X *lidar)
{
	uint8_t st;
	switch (lidar->stepL1) {
	case 0:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSRANGE_START, 0x01);  // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT
		if (st) {
			lidar->stepL1 = 1;
		}
		break;
	case 1:
		st = WriteOneRegByte(_i2c, lidar->addr, INTERNAL_TUNING_2, 0x01);
		if (st) {
			lidar->stepL1 = 2;
		}
		break;
	case 2:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSRANGE_START, 0x00);
		if (st) {
			lidar->stepL1 = 3;
		}
		break;
	case 3:
		st = WriteOneRegByte(_i2c, lidar->addr, INTERNAL_TUNING_1, 0x00);
		if (st) {
			lidar->stepL1 = 4;
		}
		break;
	case 4:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSRANGE_START, 0x01);
		if (st) {
			lidar->stepL1 = 5;
		}
		break;
	case 5:
		st = WriteOneRegByte(_i2c, lidar->addr, INTERNAL_TUNING_2, 0x00);
		if (st) {
			lidar->stepL1 = 0;
			return 1;
		}
		break;
	default:
		lidar->stepL1 = 0;
	}
	return 0;
}
/*****************************************************************
  * @brief	OK
  * @param
  * @retval
  */
uint8_t readRangeContinuousMillimeters(I2C_IRQ_Conn_t *_i2c, VL53L0X *lidar)
{
	uint8_t st;
	switch (lidar->stepL1) {
	case 0:
		st = ReadOneRegByte(_i2c, lidar->addr, RESULT_INTERRUPT_STATUS, &lidar->tmp8);
		if (st){
			if ((lidar->tmp8 & 0x07)) {
				lidar->stepL1 = 1;
			}
			else if (checkTimeoutExpired(lidar)) {//timeout
				lidar->timeoutFlag = 1;
				//lidar->range = 65535;
				lidar->status = DEVICE_ERROR;
				lidar->stepL1 = 2;
			}
		}
		break;
	case 1:
		st = ReadRegBytes(_i2c, lidar->addr, RESULT_RANGE_STATUS + 10, &lidar->range, 2);
		if (st) {
			lidar->status = DEVICE_DONE;
			lidar->stepL1 = 2;
		}
		break;
	case 2:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSTEM_INTERRUPT_CLEAR, 0x01);
		if (st) {
			startTimeout(lidar);
			lidar->stepL1 = 0;
			return 1;
		}
		break;
	default:
		lidar->stepL1 = 0;
	}
	return 0;
}
/*****************************************************************
  * @brief	OK
  * @param
  * @retval
  */
uint8_t readRangeSingleMillimeters(I2C_IRQ_Conn_t *_i2c, VL53L0X *lidar)
{
	uint8_t st;
	switch (lidar->stepL2) {
	case 0:
		st = WriteOneRegByte(_i2c, lidar->addr, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
		if (st) {
			lidar->stepL2 = 1;
		}
		break;
	case 1:
		st = WriteOneRegByte(_i2c, lidar->addr, INTERNAL_TUNING_2, 0x01);
		if (st) {
			lidar->stepL2 = 2;
		}
		break;
	case 2:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSRANGE_START, 0x00);
		if (st) {
			lidar->stepL2 = 3;
		}
		break;
	case 3:
		st = WriteOneRegByte(_i2c, lidar->addr, INTERNAL_TUNING_1, lidar->stop_variable);
		if (st) {
			lidar->stepL2 = 4;
		}
		break;
	case 4:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSRANGE_START, 0x01);
		if (st) {
			lidar->stepL2 = 5;
		}
		break;
	case 5:
		st = WriteOneRegByte(_i2c, lidar->addr, INTERNAL_TUNING_2, 0x00);
		if (st) {
			lidar->stepL2 = 6;
		}
		break;
	case 6:
		st = WriteOneRegByte(_i2c, lidar->addr, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
		if (st) {
			lidar->stepL2 = 7;
		}
		break;
	case 7:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSRANGE_START, 0x01);
		if (st) {
			startTimeout(lidar);
			lidar->stepL2 = 8;
		}
		break;
	case 8:// "Wait until start bit has been cleared"
		st = ReadOneRegByte(_i2c, lidar->addr, SYSRANGE_START, &lidar->tmp8);
		if (st) {
			if ((lidar->tmp8 & 0x01) == 0) {
				lidar->stepL2 = 9;
			}
			else if (checkTimeoutExpired(lidar)) {
				lidar->stepL2 = 0;
				lidar->status = DEVICE_FAULTH;
				return 1;
			}
		}
		break;
	case 9:
		st = readRangeContinuousMillimeters(_i2c, lidar);
		if (st) {
			lidar->stepL2 = 0;
			return 1;
		}
		break;
	default:
		lidar->stepL2 = 0;
		return 0;
	}
	return 0;
}
//--------------------------------------------------------------
uint8_t VL_Init(I2C_IRQ_Conn_t *_i2c, VL53L0X *lidar) {
	uint8_t st;
	// VL53L0X_DataInit() begin
	switch (lidar->stepL3) {
	case 0://+
		st = ReadOneRegByte(_i2c, lidar->addr, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, &lidar->tmp8);
		if (st) {
			lidar->stepL3 = 1;
		}
		break;
	case 1:// always 2.8v
		st = WriteOneRegByte(_i2c, lidar->addr, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, lidar->tmp8 | 0x01);  // set bit 0
		if (st) {
			lidar->stepL3 = 2;
		}
		break;
	case 2:  //Set I2C standard mode
		st = WriteOneRegByte(_i2c, lidar->addr, 0x88, 0x00);
		if (st) {
			lidar->stepL3 = 3;
		}
		break;
	case 3:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x80, 0x01);
		if (st) {
			lidar->stepL3 = 4;
		}
		break;
	case 4:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x01);
		if (st) {
			lidar->stepL3 = 5;
		}
		break;
	case 5:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x00, 0x00);
		if (st) {
			lidar->stepL3 = 6;
		}
		break;
	case 6://stop variable 0x11
		st = readOneByte(_i2c, lidar->addr, 0x91, &lidar->stop_variable);
		if (st) {
			lidar->stepL3 = 7;
		}
		break;
	case 7:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x00, 0x01);
		if (st) {
			lidar->stepL3 = 8;
		}
		break;
	case 8:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x00);
		if (st) {
			lidar->stepL3 = 9;
		}
		break;
	case 9:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x80, 0x00);
		if (st) {
			lidar->stepL3 = 10;
		}
		break;
	case 10:// disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks//+
		st = readOneByte(_i2c, lidar->addr, MSRC_CONFIG_CONTROL, &lidar->tmp8);
		if (st) {
			lidar->stepL3 = 11;
		}
		break;//tmp = 0x32
	case 11:
		st = WriteOneRegByte(_i2c, lidar->addr, MSRC_CONFIG_CONTROL, lidar->tmp8 | 0x12);
		if (st) {
			lidar->stepL3 = 12;
		}
		break;
	case 12:
		st = setSignalRateLimit(_i2c, lidar->addr, 32);
		if (st) {
			lidar->stepL3 = 13;
		}
		break;
	case 13:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSTEM_SEQUENCE_CONFIG, 0xFF);
		if (st) {
			lidar->stepL3 = 14;
		}
		break;
		// VL53L0X_DataInit() end
		// VL53L0X_StaticInit() begin
		// The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
		// the API, but the same data seems to be more easily readable from
		// GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
	case 14:
		st = getSpadInfo(_i2c, lidar);
		if (st) {
			lidar->stepL3 = 15;
		}
		break;
	case 15:
		st = readBytes(_i2c, lidar->addr, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, &lidar->ref_spad_map, 6);
		if (st) {
			lidar->stepL3 = 16;
		}
		break;
		// -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)
	case 16:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x01);
		if (st) {
			lidar->stepL3 = 17;
		}
		break;
	case 17:
		st = WriteOneRegByte(_i2c, lidar->addr, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
		if (st) {
			lidar->stepL3 = 18;
		}
		break;
	case 18:
		st = WriteOneRegByte(_i2c, lidar->addr, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
		if (st) {
			lidar->stepL3 = 19;
		}
		break;
	case 19:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x00);
		if (st) {
			lidar->stepL3 = 20;
		}
		break;
	case 20:
		st = WriteOneRegByte(_i2c, lidar->addr, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);
		if (st) {
			uint8_t first_spad_to_enable = lidar->spad_type_is_aperture ? 12 : 0;  // 12 is the first aperture spad
			uint8_t spads_enabled = 0;
			for (uint8_t i = 0; i < 48; i++) {
				if (i < first_spad_to_enable || spads_enabled == lidar->spad_count) {
					// This bit is lower than the first one that should be enabled, or
					// (reference_spad_count) bits have already been enabled, so zero this bit
					lidar->ref_spad_map[i / 8] &= ~(1 << (i % 7));
				}
				else if ((lidar->ref_spad_map[i / 8] >> (i % 7)) & 0x1) {
					spads_enabled++;
				}
			}
			lidar->stepL3 = 21;
		}
		break;
	case 21:
		st = writeBytes(_i2c, lidar->addr, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, lidar->ref_spad_map, 6);
		if (st) {
			lidar->stepL3 = 22;
		}
		break;
		// -- VL53L0X_set_reference_spads() end

		// -- VL53L0X_load_tuning_settings() begin
		// DefaultTuningSettings from vl53l0x_tuning.h
	case 22:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x01);
		if (st) {
			lidar->stepL3 = 23;
		}
		break;
	case 23:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x00, 0x00);
		if (st) {
			lidar->stepL3 = 24;
		}
		break;
	case 24:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x00);
		if (st) {
			lidar->stepL3 = 25;
		}
		break;
	case 25:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x09, 0x00);
		if (st) {
			lidar->stepL3 = 26;
		}
		break;
	case 26:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x10, 0x00);
		if (st) {
			lidar->stepL3 = 27;
		}
		break;
	case 27:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x11, 0x00);
		if (st) {
			lidar->stepL3 = 28;
		}
		break;//+
	case 28:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x24, 0x01);
		if (st) {
			lidar->stepL3 = 29;
		}
		break;
	case 29:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x25, 0xFF);
		if (st) {
			lidar->stepL3 = 30;
		}
		break;
	case 30:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x75, 0x00);
		if (st) {
			lidar->stepL3 = 31;
		}
		break;
	case 31:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x01);
		if (st) {
			lidar->stepL3 = 32;
		}
		break;
	case 32:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x4E, 0x2C);
		if (st) {
			lidar->stepL3 = 33;
		}
		break;
	case 33:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x48, 0x00);
		if (st) {
			lidar->stepL3 = 34;
		}
		break;
	case 34:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x30, 0x20);
		if (st) {
			lidar->stepL3 = 35;
		}
		break;
	case 35:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x00);
		if (st) {
			lidar->stepL3 = 36;
		}
		break;
	case 36:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x30, 0x09);
		if (st) {
			lidar->stepL3 = 37;
		}
		break;
	case 37:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x54, 0x00);
		if (st) {
			lidar->stepL3 = 38;
		}
		break;
	case 38:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x31, 0x04);
		if (st) {
			lidar->stepL3 = 39;
		}
		break;
	case 39:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x32, 0x03);
		if (st) {
			lidar->stepL3 = 40;
		}
		break;
	case 40:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x40, 0x83);
		if (st) {
			lidar->stepL3 = 41;
		}
		break;
	case 41:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x46, 0x25);
		if (st) {
			lidar->stepL3 = 42;
		}
		break;
	case 42:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x60, 0x00);
		if (st) {
			lidar->stepL3 = 43;
		}
		break;
	case 43:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x27, 0x00);
		if (st) {
			lidar->stepL3 = 44;
		}
		break;
	case 44:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x50, 0x06);
		if (st) {
			lidar->stepL3 = 45;
		}
		break;
	case 45:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x51, 0x00);
		if (st) {
			lidar->stepL3 = 46;
		}
		break;
	case 46:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x52, 0x96);
		if (st) {
			lidar->stepL3 = 47;
		}
		break;
	case 47:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x56, 0x08);
		if (st) {
			lidar->stepL3 = 48;
		}
		break;
	case 48:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x57, 0x30);
		if (st) {
			lidar->stepL3 = 49;
		}
		break;
	case 49:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x61, 0x00);
		if (st) {
			lidar->stepL3 = 50;
		}
		break;
	case 50:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x62, 0x00);
		if (st) {
			lidar->stepL3 = 51;
		}
		break;
	case 51:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x64, 0x00);
		if (st) {
			lidar->stepL3 = 52;
		}
		break;
	case 52:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x65, 0x00);
		if (st) {
			lidar->stepL3 = 53;
		}
		break;
	case 53:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x66, 0xA0);
		if (st) {
			lidar->stepL3 = 54;
		}
		break;
	case 54:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x01);
		if (st) {
			lidar->stepL3 = 55;
		}
		break;
	case 55:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x22, 0x32);
		if (st) {
			lidar->stepL3 = 56;
		}
		break;
	case 56:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x47, 0x14);
		if (st) {
			lidar->stepL3 = 57;
		}
		break;
	case 57:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x49, 0xFF);
		if (st) {
			lidar->stepL3 = 58;
		}
		break;
	case 58:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x4A, 0x00);
		if (st) {
			lidar->stepL3 = 59;
		}
		break;
	case 59:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x00);
		if (st) {
			lidar->stepL3 = 60;
		}
		break;
	case 60:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x7A, 0x0A);
		if (st) {
			lidar->stepL3 = 61;
		}
		break;
	case 61:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x7B, 0x00);
		if (st) {
			lidar->stepL3 = 62;
		}
		break;
	case 62:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x78, 0x21);
		if (st) {
			lidar->stepL3 = 63;
		}
		break;
	case 63:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x01);
		if (st) {
			lidar->stepL3 = 64;
		}
		break;
	case 64:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x23, 0x34);
		if (st) {
			lidar->stepL3 = 65;
		}
		break;
	case 65:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x42, 0x00);
		if (st) {
			lidar->stepL3 = 66;
		}
		break;
	case 66:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x44, 0xFF);
		if (st) {
			lidar->stepL3 = 67;
		}
		break;
	case 67:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x45, 0x26);
		if (st) {
			lidar->stepL3 = 68;
		}
		break;
	case 68:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x46, 0x05);
		if (st) {
			lidar->stepL3 = 69;
		}
		break;
	case 69:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x40, 0x40);
		if (st) {
			lidar->stepL3 = 70;
		}
		break;
	case 70:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x0E, 0x06);
		if (st) {
			lidar->stepL3 = 71;
		}
		break;
	case 71:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x20, 0x1A);
		if (st) {
			lidar->stepL3 = 72;
		}
		break;
	case 72:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x43, 0x40);
		if (st) {
			lidar->stepL3 = 73;
		}
		break;
	case 73:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x00);
		if (st) {
			lidar->stepL3 = 74;
		}
		break;
	case 74:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x34, 0x03);
		if (st) {
			lidar->stepL3 = 75;
		}
		break;
	case 75:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x35, 0x44);
		if (st) {
			lidar->stepL3 = 76;
		}
		break;
	case 76:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x01);
		if (st) {
			lidar->stepL3 = 77;
		}
		break;
	case 77:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x31, 0x04);
		if (st) {
			lidar->stepL3 = 78;
		}
		break;
	case 78:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x4B, 0x09);
		if (st) {
			lidar->stepL3 = 79;
		}
		break;
	case 79:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x4C, 0x05);
		if (st) {
			lidar->stepL3 = 80;
		}
		break;
	case 80:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x4D, 0x04);
		if (st) {
			lidar->stepL3 = 81;
		}
		break;
	case 81:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x00);
		if (st) {
			lidar->stepL3 = 82;
		}
		break;
	case 82:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x44, 0x00);
		if (st) {
			lidar->stepL3 = 83;
		}
		break;
	case 83:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x45, 0x20);
		if (st) {
			lidar->stepL3 = 84;
		}
		break;
	case 84:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x47, 0x08);
		if (st) {
			lidar->stepL3 = 85;
		}
		break;
	case 85:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x48, 0x28);
		if (st) {
			lidar->stepL3 = 86;
		}
		break;
	case 86:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x67, 0x00);
		if (st) {
			lidar->stepL3 = 87;
		}
		break;
	case 87:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x70, 0x04);
		if (st) {
			lidar->stepL3 = 88;
		}
		break;
	case 88:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x71, 0x01);
		if (st) {
			lidar->stepL3 = 89;
		}
		break;
	case 89:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x72, 0xFE);
		if (st) {
			lidar->stepL3 = 90;
		}
		break;
	case 90:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x76, 0x00);
		if (st) {
			lidar->stepL3 = 91;
		}
		break;
	case 91:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x77, 0x00);
		if (st) {
			lidar->stepL3 = 92;
		}
		break;
	case 92:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x01);
		if (st) {
			lidar->stepL3 = 93;
		}
		break;
	case 93:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x0D, 0x01);
		if (st) {
			lidar->stepL3 = 94;
		}
		break;
	case 94:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x00);
		if (st) {
			lidar->stepL3 = 95;
		}
		break;
	case 95:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x80, 0x01);
		if (st) {
			lidar->stepL3 = 96;
		}
		break;
	case 96:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x01, 0xF8);
		if (st) {
			lidar->stepL3 = 97;
		}
		break;
	case 97:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x01);
		if (st) {
			lidar->stepL3 = 98;
		}
		break;
	case 98:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x8E, 0x01);
		if (st) {
			lidar->stepL3 = 99;
		}
		break;
	case 99:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x00, 0x01);
		if (st) {
			lidar->stepL3 = 100;
		}
		break;
	case 100:
		st = WriteOneRegByte(_i2c, lidar->addr, 0xFF, 0x00);
		if (st) {
			lidar->stepL3 = 101;
		}
		break;
	case 101:
		st = WriteOneRegByte(_i2c, lidar->addr, 0x80, 0x00);
		if (st) {
			lidar->stepL3 = 102;
		}
		break;
	case 102:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
		if (st) {
			lidar->stepL3 = 103;
		}
		break;
	case 103:
		st = readOneByte(_i2c, lidar->addr, GPIO_HV_MUX_ACTIVE_HIGH, &lidar->tmp8);
		if (st) {
			lidar->stepL3 = 104;
		}
		break;
	case 104:
		st = WriteOneRegByte(_i2c, lidar->addr, GPIO_HV_MUX_ACTIVE_HIGH, lidar->tmp8 & ~0x10);  // active low
		if (st) {
			lidar->stepL3 = 105;
		}
		break;
	case 105:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSTEM_INTERRUPT_CLEAR, 0x01);
		if (st) {
			lidar->stepL3 = 106;
		}
		break;
		// -- VL53L0X_SetGpioConfig() end

	case 106:
		st = getMeasurementTimingBudget(_i2c, lidar);
		if (st) {
			lidar->stepL3 = 107;
		}
		break;
		// "Disable MSRC and TCC by default"
		// MSRC = Minimum Signal Rate Check
		// TCC = Target CentreCheck
		// -- VL53L0X_SetSequenceStepEnable() begin
	case 107:
		st = WriteOneRegByte(_i2c, lidar->addr, SYSTEM_SEQUENCE_CONFIG, 0xE8);
		if (st) {
			lidar->stepL3 = 108;
		}
		break;
		// -- VL53L0X_SetSequenceStepEnable() end

	case 108:  // "Recalculate timing budget"
		st = setMeasurementTimingBudget(_i2c, lidar->addr, lidar->measurement_timing_budget_us);
		if (st) {
			lidar->stepL3 = 109;
		}
		break;
		// VL53L0X_StaticInit() end
		// VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())
	case 109:  // -- VL53L0X_perform_vhv_calibration() begin
		st = WriteOneRegByte(_i2c, lidar->addr, SYSTEM_SEQUENCE_CONFIG, 0x01);
		if (st) {
			lidar->stepL3 = 110;
		}
		break;
	case 110:  //  performSingleRefCalibration(_i2c,lidar,0x40);
		st = performSingleRefCalibration(_i2c, lidar->addr, 0x40);
		if (st) {
			lidar->stepL3 = 111;
		}
		break;
		// -- VL53L0X_perform_vhv_calibration() end
	case 111:  // -- VL53L0X_perform_phase_calibration() begin
		st = WriteOneRegByte(_i2c, lidar->addr, SYSTEM_SEQUENCE_CONFIG, 0x02);
		if (st) {
			lidar->stepL3 = 112;
		}
		break;
	case 112:  //  performSingleRefCalibration(_i2c,lidar,0x00);
		st = performSingleRefCalibration(_i2c, lidar->addr, 0x00);
		if (st) {
			lidar->stepL3 = 113;
		}
		break;
		// -- VL53L0X_perform_phase_calibration() end
	case 113:  // "restore the previous Sequence Config"
		st = WriteOneRegByte(_i2c, lidar->addr, SYSTEM_SEQUENCE_CONFIG, 0xE8);
		if (st) {
			lidar->stepL3 = 0;
			setTimeout(lidar, 50);
			startTimeout(lidar);
			return 1;
		}
		break;
		// VL53L0X_PerformRefCalibration() end
	default:
		lidar->stepL3 = 0;
		break;
	}
	return 0;
}

uint16_t getSmoothRange(VL53L0X *lidar) {
	//static_assert(VL53L0x_SHOOTH_DEEP % 2 == 0, "smooth deep most be power of 2"); //проверка степени двойки
	lidar->smoothRange = alphabeta(lidar->range, lidar->smoothRange, 8);
	return lidar->smoothRange;
}

