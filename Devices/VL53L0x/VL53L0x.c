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
  * @brief  concat two bytes into word
  * @param  msb - hi byte, lsb - lo byte
  * @retval uint16_t
  */
static inline uint16_t __CONCAT_BYTES(uint8_t msb, uint8_t lsb) {
	return (((uint16_t)msb << 8) | (uint16_t)lsb);
}
/*****************************************************************
  * @brief
  * @param
  * @retval
  */
static inline size_t __Min(size_t val, size_t min) {
	return val > min ? val : min;
}
/*****************************************************************
  * @brief
  * @param
  * @retval
  */
static inline size_t __Max(size_t val, size_t max) {
	return val < max ? val : max;
}
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

//================================================================
// VALUE CONVERTERS
// ===============================================================
/*****************************************************************
  * @brief
  * @param
  * @retval
  */
uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
	uint32_t macro_period_ns = __calcMacroPeriod(vcsel_period_pclks);
	return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}
/*****************************************************************
  * @brief
  * @param
  * @retval
  */
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
	uint32_t macro_period_ns = __calcMacroPeriod(vcsel_period_pclks);
	return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}
/*****************************************************************
  * @brief
  * @param
  * @retval
  */
uint16_t decodeTimeout(uint16_t reg_val)
{  // format: "(LSByte * 2^MSByte) + 1"
	return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}
/*****************************************************************
  * @brief
  * @param
  * @retval
  */
uint16_t encodeTimeout(uint16_t timeout_mclks)
{  // format: "(LSByte * 2^MSByte) + 1"
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



// =================================================================
// BUS WRITE
// =================================================================
/*****************************************************************
  * @brief
  * @param
  * @retval
  */



/*****************************************************************
  * @brief
  * @param
  * @retval
  */
uint16_t readReg16Bit(I2C_IRQ_Connection_t *_i2c,VL53L0X *lidar,uint8_t reg)
{
  int16_t value;
	uint8_t tmp[2];
	value = __CONCAT_BYTES(tmp[0], tmp[1]);
	return value;
}
/*****************************************************************
  * @brief
  * @param
  * @retval
  */
uint32_t readReg32Bit(I2C_IRQ_Connection_t *_i2c,VL53L0X *lidar,uint8_t reg)
{
	uint8_t tmp[4];
	uint32_t value;
	uint16_t msw = __CONCAT_BYTES(tmp[0], tmp[1]);
	uint16_t lsw = __CONCAT_BYTES(tmp[2], tmp[3]);
	value = (((uint32_t)msw)<<16) | (uint32_t)lsw;
	return value;
}
/*****************************************************************
  * @brief
  * @param
  * @retval
  */
void readMulti(I2C_IRQ_Connection_t *_i2c,VL53L0X *lidar,uint8_t reg,uint8_t *data,uint8_t count)
{

}

// =================================================================
// SIMPLE GETTERS AND SETTERS
// =================================================================
/*****************************************************************
  * @brief
  * @param
  * @retval
  */
void setupVL53L0X(VL53L0X *lidar, uint16_t timeout)//check
{
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
uint8_t setAddress(I2C_IRQ_Connection_t *_i2c, VL53L0X *lidar, const uint8_t new_addr)
{
	uint8_t st;
	st = writeOneByte(_i2c, lidar, CHIP_I2C_ADDRESS, new_addr & 0x7F);
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
uint8_t getModelId(I2C_IRQ_Connection_t *_i2c, VL53L0X *lidar)
{
	uint8_t st;
	st = readOneByte(_i2c, lidar, IDENTIFICATION_MODEL_ID, &lidar->modelID);
	return st;
}
/*****************************************************************
  * @brief
  * @param
  * @retval
  */
uint8_t getRevisionId(I2C_IRQ_Connection_t *_i2c, VL53L0X *lidar)
{
	uint8_t st;
	st = readOneByte(_i2c, lidar, IDENTIFICATION_REVISION_ID, &lidar->revisionID);
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
uint8_t setSignalRateLimit(I2C_IRQ_Connection_t *_i2c, VL53L0X *lidar, uint32_t limit_Mcps)
{
	uint8_t st;
	//uint16_t data = Min(Max((uint16_t)_limit_Mcps, 65535), 32);
	uint16_t val = (uint16_t)limit_Mcps;
	uint8_t data[2];
	data[0] = (uint8_t)val>>8 & 0xFF; //MSB
	data[1] = (uint8_t)val & 0xFF; //LSB
	st = writeBytes(_i2c, lidar, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, &data, 2);
	return st;
}
/*****************************************************************
  * @brief  OK
  * @param
  * @retval
  */
uint8_t getSpadInfo(I2C_IRQ_Connection_t *_i2c,VL53L0X *lidar)
{
	uint8_t st;
	switch (lidar->step_l1) {
	case 0:
		st = writeOneByte(_i2c, lidar, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
		if (st) {
			lidar->step_l1 = 1;
		}
		break;
	case 1:
		st = writeOneByte(_i2c, lidar, INTERNAL_TUNING_2, 0x01);
		if (st) {
			lidar->step_l1 = 2;
		}
		break;
	case 2:
		st = writeOneByte(_i2c, lidar, SYSRANGE_START, 0x00);
		if (st) {
			lidar->step_l1 = 3;
		}
		break;
	case 3:
		st = writeOneByte(_i2c, lidar, INTERNAL_TUNING_2, 0x06);
		if (st) {
			lidar->step_l1 = 4;
		}
		break;
	case 4:
		st = readOneByte(_i2c, lidar, SOMETHING_MAGIC_REG, &lidar->tmp);
		if (st) {
			lidar->tmp |= 0x04;
			lidar->step_l1 = 5;
		}
		break;
	case 5:
		st = writeOneByte(_i2c, lidar, SOMETHING_MAGIC_REG, lidar->tmp);
		if (st) {
			lidar->step_l1 = 6;
		}
		break;
	case 6:
		st = writeOneByte(_i2c, lidar, INTERNAL_TUNING_2, 0x07);
		if (st) {
			lidar->step_l1 = 7;
		}
		break;
	case 7:
		st = writeOneByte(_i2c, lidar, SYSTEM_HISTOGRAM_BIN, 0x01);
		if (st) {
			lidar->step_l1 = 8;
		}
		break;
	case 8:
		st = writeOneByte(_i2c, lidar, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
		if (st) {
			lidar->step_l1 = 9;
		}
		;
		break;
	case 9:
		st = writeOneByte(_i2c, lidar, 0x94, 0x6b);
		if (st) {
			lidar->step_l1 = 10;
		}
		break;
	case 10:
		st = writeOneByte(_i2c, lidar, SOMETHING_MAGIC_REG, 0x00);
		if (st) {
			setTimeout(lidar, 100);
			startTimeout(lidar);
			lidar->step_l1 = 11;
		}
		break;
	case 11:
		st = readOneByte(_i2c, lidar, SOMETHING_MAGIC_REG, &lidar->tmp);
		if (st) {
			if (lidar->tmp != 0x00) {
				lidar->step_l1 = 12;
			}
			else if (checkTimeoutExpired(lidar)) {
				lidar->step_l1 = 0;
				lidar->status = DEVICE_FAULTH;
				return 1;
			}
		}
		break;
	case 12:
		st = writeOneByte(_i2c, lidar, SOMETHING_MAGIC_REG, 0x01);
		if (st) {
			lidar->step_l1 = 13;
		}
		break;
	case 13:
		st = readOneByte(_i2c, lidar, 0x92, &lidar->tmp);
		if (st) {
			lidar->spad_count = lidar->tmp & 0x7f;
			lidar->spad_type_is_aperture = (lidar->tmp >> 7) & 0x01;
			lidar->step_l1 = 14;
		}
		break;
	case 14:
		st = writeOneByte(_i2c, lidar, SYSTEM_HISTOGRAM_BIN, 0x00);
		if (st) {
			lidar->step_l1 = 15;
		}
		break;
	case 15:
		st = writeOneByte(_i2c, lidar, INTERNAL_TUNING_2, 0x06);
		if (st) {
			lidar->step_l1 = 16;
		}
		break;
	case 16:
		st = readOneByte(_i2c, lidar, SOMETHING_MAGIC_REG, &lidar->tmp);
		if (st) {
			lidar->tmp &= ~0x04;
			lidar->step_l1 = 17;
		}
		break;
	case 17:
		st = writeOneByte(_i2c, lidar, SOMETHING_MAGIC_REG, lidar->tmp);
		if (st) {
			lidar->step_l1 = 18;
		}
		break;
	case 18:
		st = writeOneByte(_i2c, lidar, INTERNAL_TUNING_2, 0x01);
		if (st) {
			lidar->step_l1 = 19;
		}
		break;
	case 19:
		st = writeOneByte(_i2c, lidar, SYSRANGE_START, 0x01);
		if (st) {
			lidar->step_l1 = 20;
		}
		break;
	case 20:
		st = writeOneByte(_i2c, lidar, INTERNAL_TUNING_2, 0x00);
		if (st) {
			lidar->step_l1 = 21;
		}
		break;
	case 21:
		st = writeOneByte(_i2c, lidar, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
		if (st) {
			lidar->step_l1 = 0;
			return 1;
		}
		break;
	default:
		lidar->step_l1 = 0;
		break;
	}
	return 0;
}
/*****************************************************************
  * @brief  OK
  * @param
  * @retval
  */
uint8_t getSequenceStepEnables(I2C_IRQ_Connection_t *_i2c,VL53L0X *lidar)
{
	uint8_t st;
	uint8_t sequence_config;
	st = readOneByte(_i2c, lidar, SYSTEM_SEQUENCE_CONFIG, &sequence_config);
	if (st) {
		lidar->enables.tcc = (sequence_config >> 4) & 0x1;
		lidar->enables.dss = (sequence_config >> 3) & 0x1;
		lidar->enables.msrc = (sequence_config >> 2) & 0x1;
		lidar->enables.pre_range = (sequence_config >> 6) & 0x1;
		lidar->enables.final_range = (sequence_config >> 7) & 0x1;
		return 1;
	}
	return 0;
}
/*****************************************************************
  * @brief	OK
  * @param
  * @retval
  */
uint8_t getVcselPulsePeriod(I2C_IRQ_Connection_t *_i2c,VL53L0X *lidar, vcselPeriodType type)
{
	uint8_t st = 0;
	if (type == VcselPeriodPreRange) {
		st = readOneByte(_i2c, lidar, PRE_RANGE_CONFIG_VCSEL_PERIOD, &lidar->tmp);
		if (st) {
			lidar->vcselPeriodValue = __decodeVcselPeriod(lidar->tmp);
		}
	}
	else if (type == VcselPeriodFinalRange) {
		st = readOneByte(_i2c, lidar, FINAL_RANGE_CONFIG_VCSEL_PERIOD, &lidar->tmp);
		if (st) {
			lidar->vcselPeriodValue = __decodeVcselPeriod(lidar->tmp);
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
uint8_t getSequenceStepTimeouts(I2C_IRQ_Connection_t *_i2c,VL53L0X *lidar)
{
	uint8_t st;
	static uint8_t dt[2];
	switch (lidar->step_l1) {
	case 0:
		st = getVcselPulsePeriod(_i2c, lidar, VcselPeriodPreRange);
		if (st) {
			lidar->timeouts.pre_range_vcsel_period_pclks = lidar->vcselPeriodValue;
			lidar->step_l1 = 1;
		}
		break;
	case 1:
		st = readOneByte(_i2c, lidar, MSRC_CONFIG_TIMEOUT_MACROP, &lidar->tmp);
		if (st) {
			lidar->timeouts.msrc_dss_tcc_mclks = lidar->tmp + 1;
			lidar->timeouts.msrc_dss_tcc_us = timeoutMclksToMicroseconds(
					lidar->timeouts.msrc_dss_tcc_mclks, lidar->timeouts.pre_range_vcsel_period_pclks);
			lidar->step_l1 = 2;
		}
		break;
	case 2:
		st = readBytes(_i2c, lidar, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, dt, 2);
		if (st) {
			lidar->timeouts.pre_range_mclks = decodeTimeout(__CONCAT_BYTES(dt[0], dt[1]));
			lidar->timeouts.pre_range_us = timeoutMclksToMicroseconds
																			(lidar->timeouts.pre_range_mclks,
																			 lidar->timeouts.pre_range_vcsel_period_pclks);
			lidar->step_l1 = 3;
		}
		break;
	case 3:
		st = getVcselPulsePeriod(_i2c, lidar, VcselPeriodFinalRange);
		if (st) {
			lidar->timeouts.final_range_vcsel_period_pclks= lidar->vcselPeriodValue;
			lidar->step_l1 = 4;
		}
		break;
	case 4:
		st = readBytes(_i2c, lidar, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &dt, 2);
		if (st) {
			lidar->timeouts.final_range_mclks = decodeTimeout(__CONCAT_BYTES(dt[0], dt[1]));
			if (lidar->enables.pre_range) {
				lidar->timeouts.final_range_mclks -= lidar->timeouts.pre_range_mclks;
			}
			lidar->timeouts.final_range_us = timeoutMclksToMicroseconds(
																				lidar->timeouts.final_range_mclks,
																				lidar->timeouts.final_range_vcsel_period_pclks);
			lidar->step_l1 = 0;
			return 1;
		}
		break;
	default:
		lidar->step_l1 = 0;
		break;
	}
	return 0;
}
/*****************************************************************
  * @brief
  * @param
  * @retval
  */
uint8_t getMeasurementTimingBudget(I2C_IRQ_Connection_t *_i2c, VL53L0X *lidar)
{
	uint8_t st;
	uint16_t const StartOverhead = 1910;  // note that this is different than the value in set_
	uint16_t const EndOverhead = 960;
	uint16_t const MsrcOverhead = 660;
	uint16_t const TccOverhead = 590;
	uint16_t const DssOverhead = 690;
	uint16_t const PreRangeOverhead = 660;
	uint16_t const FinalRangeOverhead = 550;
	switch (lidar->step_l2) {
	case 0:
		// "Start and end overhead times always present"
		lidar->measurement_timing_budget_us = StartOverhead + EndOverhead;
		lidar->step_l2 = 1; // @suppress("No break at end of case")
		//break;
	case 1:
		st = getSequenceStepEnables(_i2c, lidar);
		if (st) {
			lidar->step_l2 = 2;
		}
		break;
	case 2:
		st = getSequenceStepTimeouts(_i2c, lidar);
		if (st) {
			lidar->step_l2 = 3;
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
		lidar->step_l2 = 0;
		return 1;
		break;
	default:
		lidar->step_l2 = 0;
		break;
	}
	return 0;
}
/*****************************************************************
  * @brief OK/2!!!!!!!
  * @param
  * @retval
  */
//TODO добавить автоподстройку таймаута чтобы не было ошибок, вынести переменные таймаутов в структуру датчика для наглядности
uint8_t setMeasurementTimingBudget(I2C_IRQ_Connection_t *_i2c, VL53L0X *lidar, uint32_t budget_us)//check
{
	uint8_t st;
	static uint16_t tmp;
	static uint8_t dt[2];

	uint16_t const StartOverhead = 1320;// note that this is different than the value in get_
	uint16_t const EndOverhead = 960;
	uint16_t const MsrcOverhead = 660;
	uint16_t const TccOverhead = 590;
	uint16_t const DssOverhead = 690;
	uint16_t const PreRangeOverhead = 660;
	uint16_t const FinalRangeOverhead = 550;
	uint32_t const MinTimingBudget = 20000;

	switch (lidar->step_l2) {
	case 0:
		if (budget_us < MinTimingBudget)// EXIT!!!
		{
			lidar->status = DEVICE_FAULTH;
			return 1;
		}
		lidar->measurement_timing_budget_us = StartOverhead + EndOverhead;
		lidar->step_l2 = 1;
		break;
	case 1:
		st = getSequenceStepEnables(_i2c, lidar);
		if (st) {
			lidar->step_l2 = 2;
		}
		break;
	case 2:
		st = getSequenceStepTimeouts(_i2c, lidar);
		if (st) {
			lidar->step_l2 = 3;
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
			lidar->measurement_timing_budget_us += FinalRangeOverhead;
			// "Note that the final range timeout is determined by the timing
			// budget and the sum of all other timeouts within the sequence.
			// If there is no room for the final range timeout, then an error
			// will be set. Otherwise the remaining time will be applied to
			// the final range."
			if (lidar->measurement_timing_budget_us > budget_us) {//"Requested timeout too big."
				lidar->step_l2 = 0;
				return 1;
			}
			uint32_t final_range_timeout_us = budget_us - lidar->measurement_timing_budget_us;
			// set_sequence_step_timeout() begin
			// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)
			// "For the final range timeout, the pre-range timeout
			//  must be added. To do this both final and pre-range
			//  timeouts must be expressed in macro periods MClks
			//  because they have different vcsel periods."
			uint16_t final_range_timeout_mclks = timeoutMicrosecondsToMclks(
					final_range_timeout_us, lidar->timeouts.final_range_vcsel_period_pclks);
			if (lidar->enables.pre_range) {
				final_range_timeout_mclks += lidar->timeouts.pre_range_mclks;
			}
			tmp = encodeTimeout(final_range_timeout_mclks);
			dt[0] = (uint8_t)(tmp >> 8);//старший байт передается первым
			dt[1] = (uint8_t)tmp;
			lidar->step_l2 = 4;
		}
		else {
			lidar->status = DEVICE_FAULTH;
			lidar->step_l2 = 0;
			return 1;
		}
		break;
	case 4:
		st = writeBytes(_i2c, lidar, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, dt, 2);
		writeReg16Bit(_i2c, lidar, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, tmp);
		st = 1;
		if (st) {
			// set_sequence_step_timeout() end
			lidar->measurement_timing_budget_us = budget_us;  // store for internal reuse
			lidar->step_l2 = 0;
			return 1;
		}
		break;
	default:
		lidar->step_l2 = 0;
		break;
	}
	return 0;
}
/*****************************************************************
  * @brief	OK
  * @param
  * @retval
  */
uint8_t performSingleRefCalibration(I2C_IRQ_Connection_t *_i2c, VL53L0X *lidar, uint8_t vhv_init_byte)
{
	uint8_t st;
	switch (lidar->step_l1) {
	case 0:
		st = writeOneByte(_i2c, lidar, SYSRANGE_START, 0x01 | vhv_init_byte);// VL53L0X_REG_SYSRANGE_MODE_START_STOP
		if (st) {
			startTimeout(lidar);
			lidar->step_l1 = 1;
		}
		break;
	case 1:
		st = readOneByte(_i2c, lidar, RESULT_INTERRUPT_STATUS, &lidar->tmp);
		if (st) {
			if ((lidar->tmp & 0x07) != 0) {
				lidar->step_l1 = 2;
			}
			else if (checkTimeoutExpired(lidar)) {
				lidar->status = DEVICE_FAULTH;
				lidar->step_l1 = 2;
			}
		}
		break;
	case 2:
		st = writeOneByte(_i2c, lidar, SYSTEM_INTERRUPT_CLEAR, 0x01);
		if (st) {
			lidar->step_l1 = 3;
		}
		break;
	case 3:
		st = writeOneByte(_i2c, lidar, SYSRANGE_START, 0x00);
		if (st) {
			lidar->step_l1 = 0;
			return 1;
		}
		break;
	default:
		lidar->step_l1 = 0;
		break;
	}
	return 0;
}
/*****************************************************************
  * @brief	NOT RECHECK
  * @param
  * @retval
  */
uint8_t setVcselPulsePeriod(I2C_IRQ_Connection_t *_i2c,VL53L0X *lidar,vcselPeriodType type,uint8_t period_pclks)
{
	static uint8_t step = 0;
	uint8_t st;
	uint8_t tmp;
	static uint8_t sequence_config;
	static uint8_t vcsel_period_reg;
	uint16_t new_pre_range_timeout_mclks;
	uint16_t val;
	static uint8_t dt[2];
	uint16_t new_msrc_timeout_mclks;
	uint16_t new_final_range_timeout_mclks;

	switch (step) {
	case 0:
		st = getSequenceStepEnables(_i2c, lidar);
		if (st) {
			step = 1;
		}
		break;
	case 1:
		st = getSequenceStepTimeouts(_i2c, lidar);
		if (st) {
			vcsel_period_reg = __encodeVcselPeriod(period_pclks);
			step = 2;
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
				step = 5;
				break;
			case 14:
				step = 6;
				break;
			case 16:
				step = 7;
				break;
			case 18:
				step = 8;
				break;
			default:// invalid period
				lidar->status = DEVICE_FAULTH;
				return 1;
			}
		}
		else if (type == VcselPeriodFinalRange) {
			switch (period_pclks) {
			case 8:
				step = 15;
				break;
			case 10:
				step = 25;
				break;
			case 12:
				step = 35;
				break;
			case 14:
				step = 45;
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
		st = writeOneByte(_i2c, lidar, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
		if (st) {
			step = 9;
		}
		break;
	case 6:  //period_pclks = 14
		st = writeOneByte(_i2c, lidar, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
		if (st) {
			step = 9;
		}
		break;
	case 7:  //period_pclks = 16
		st = writeOneByte(_i2c, lidar, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
		if (st) {
			step = 9;
		}
		break;
	case 8:  //period_pclks = 18
		st = writeOneByte(_i2c, lidar, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
		if (st) {
			step = 9;
		}
		break;
	case 9:
		st = writeOneByte(_i2c, lidar, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
		if (st) {
			step = 10;
		}
		break;
	case 10:// apply new VCSEL period
		st = writeOneByte(_i2c, lidar, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
		if (st) {
			// set_sequence_step_timeout() begin
			// (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)
			new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(lidar->timeouts.pre_range_us, period_pclks);
			val = encodeTimeout(new_pre_range_timeout_mclks);
			dt[0]=((uint8_t)val>>8) & 0xFF;	//MSB
			dt[1]=(uint8_t)val & 0xFF; 			//LSB
			step = 11;
		}
		break;
	case 11:// update timeouts
		st = writeBytes(_i2c, lidar, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, dt, 2);
		// set_sequence_step_timeout() end
		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)
		if (st) {
			step = 12;
		}
		break;
	case 12:
		new_msrc_timeout_mclks = timeoutMicrosecondsToMclks(lidar->timeouts.msrc_dss_tcc_us, period_pclks);
		tmp = new_msrc_timeout_mclks > 256 ? 255 : new_msrc_timeout_mclks - 1;
		st = writeOneByte(_i2c, lidar, MSRC_CONFIG_TIMEOUT_MACROP, tmp);
		if (st) {
			step = 102;
		}
		break;// set_sequence_step_timeout() end

	//type == VcselPeriodFinalRange
	case 15:  //period_pclks = 8
		st = writeOneByte(_i2c, lidar, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
		if (st) {
			step = 16;
		}
		break;
	case 16:
		st = writeOneByte(_i2c, lidar, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
		if (st) {
			step = 17;
		}
		break;
	case 17:
		st = writeOneByte(_i2c, lidar, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
		if (st) {
			step = 18;
		}
		break;
	case 18:
		st = writeOneByte(_i2c, lidar, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
		if (st) {
			step = 19;
		}
		break;
	case 19:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x01);
		if (st) {
			step = 20;
		}
		break;
	case 20:
		st = writeOneByte(_i2c, lidar, ALGO_PHASECAL_LIM, 0x30);
		if (st) {
			step = 51;
		}
		break;

	case 25:  //period_pclks = 10
		st = writeOneByte(_i2c, lidar, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
		if (st) {
			step = 26;
		}
		break;
	case 26:
		st = writeOneByte(_i2c, lidar, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
		if (st) {
			step = 27;
		}
		break;
	case 27:
		st = writeOneByte(_i2c, lidar, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
		if (st) {
			step = 28;
		}
		break;
	case 28:
		st = writeOneByte(_i2c, lidar, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
		if (st) {
			step = 49;
		}
		break;

	case 35:  //period_pclks = 12
		st = writeOneByte(_i2c, lidar, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
		if (st) {
			step = 36;
		}
		break;
	case 36:
		st = writeOneByte(_i2c, lidar, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
		if (st) {
			step = 37;
		}
		break;
	case 37:
		st = writeOneByte(_i2c, lidar, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
		if (st) {
			step = 38;
		}
		break;
	case 38:
		st = writeOneByte(_i2c, lidar, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
		if (st) {
			step = 49;
		}
		break;

	case 45:  //period_pclks = 14
		st = writeOneByte(_i2c, lidar, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
		if (st) {
			step = 46;
		}
		break;
	case 46:
		st = writeOneByte(_i2c, lidar, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
		if (st) {
			step = 47;
		}
		break;
	case 47:
		st = writeOneByte(_i2c, lidar, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
		if (st) {
			step = 48;
		}
		break;
	case 48:
		st = writeOneByte(_i2c, lidar, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
		if (st) {
			step = 49;
		}
		break;
	case 49:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x01);
		if (st) {
			step = 50;
		}
		break;
	case 50:
		st = writeOneByte(_i2c, lidar, ALGO_PHASECAL_LIM, 0x20);
		if (st) {
			step = 51;
		}
		break;
	case 51:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x00);
		if (st) {
			step = 55;
		}
		break;
	case 55:  // apply new VCSEL period
		st = writeOneByte(_i2c, lidar, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
		if (st) {
		// "For the final range timeout, the pre-range timeout
		//  must be added. To do this both final and pre-range
		//  timeouts must be expressed in macro periods MClks
		//  because they have different vcsel periods."
			new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(lidar->timeouts.final_range_us, period_pclks);
			if (lidar->enables.pre_range) {
				new_final_range_timeout_mclks += lidar->timeouts.pre_range_mclks;
			}
			uint16_t val = encodeTimeout(new_final_range_timeout_mclks);
			dt[0]=((uint8_t)val>>8) & 0xFF;	//MSB
			dt[1]=(uint8_t)val & 0xFF; 			//LSB
			step = 56;
		}
		break;
	case 56:// update timeouts
		// set_sequence_step_timeout() begin
		st = writeBytes(_i2c, lidar, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, dt, 2);
		if (st) {
			step = 102;
		}
		break;
		// set_sequence_step_timeout end
	case 102:// "Finally, the timing budget must be re-applied"
		st = setMeasurementTimingBudget(_i2c, lidar, lidar->measurement_timing_budget_us);
		if (st) {
			step = 103;
		}
		break;
	case 103:
		st = readOneByte(_i2c, lidar, SYSTEM_SEQUENCE_CONFIG, &sequence_config);
		if (st) {
			step = 104;
		}
		break;
	case 104:// "Perform the phase calibration. This is needed after changing on vcsel period."
		// VL53L0X_perform_phase_calibration() begin
		st = writeOneByte(_i2c, lidar, SYSTEM_SEQUENCE_CONFIG, 0x02);
		if (st) {
			step = 105;
		}
		break;
	case 105:
		st = performSingleRefCalibration(_i2c, lidar, 0x00);
		if (st) {
			step = 106;
		}
		break;
	case 106:
		st = writeOneByte(_i2c, lidar, SYSTEM_SEQUENCE_CONFIG, sequence_config);
		if (st) {
			step = 0;
			return 1;
		}
		break;
		// VL53L0X_perform_phase_calibration() end
	default:
		step = 0;
		break;
	}
	return 0;
}

/*****************************************************************
  * @brief	OK
  * @param
  * @retval
  */
uint8_t startContinuous(I2C_IRQ_Connection_t *_i2c, VL53L0X *lidar, uint32_t period_ms)
{
	uint8_t st;
	static uint8_t tmp[2];
	static uint16_t osc_calibrate_val;
	static uint32_t period_ms_int;
	switch (lidar->step_l1) {
	case 0:
		st = writeOneByte(_i2c, lidar, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
		if (st) {
			lidar->step_l1 = 1;
		}
		break;
	case 1:
		st = writeOneByte(_i2c, lidar, INTERNAL_TUNING_2, 0x01);
		if (st) {
			lidar->step_l1 = 2;
		}
		break;
	case 2:
		st = writeOneByte(_i2c, lidar, SYSRANGE_START, 0x00);
		if (st) {
			lidar->step_l1 = 3;
		}
		break;
	case 3:
		st = writeOneByte(_i2c, lidar, INTERNAL_TUNING_1, lidar->stop_variable);
		if (st) {
			lidar->step_l1 = 4;
		}
		break;
	case 4:
		st = writeOneByte(_i2c, lidar, SYSRANGE_START, 0x01);
		if (st) {
			lidar->step_l1 = 5;
		}
		break;
	case 5:
		st = writeOneByte(_i2c, lidar, INTERNAL_TUNING_2, 0x00);
		if (st) {
			lidar->step_l1 = 6;
		}
		break;
	case 6:
		st = writeOneByte(_i2c, lidar, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
		if (st) {
			if (period_ms) {
				period_ms_int = period_ms;
				lidar->step_l1 = 7;
			}
			else {
				lidar->step_l1 = 11;
			}
		}
		break;
	case 7:// continuous timed mode
		st = readBytes(_i2c, lidar, OSC_CALIBRATE_VAL, tmp, 2);// VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin
		if (st) {
			osc_calibrate_val = __CONCAT_BYTES(tmp[0], tmp[1]);
			if (osc_calibrate_val) {
				period_ms_int = period_ms * osc_calibrate_val;
				tmp[0]=(uint8_t)period_ms_int>>8 & 0xFF; //MSB
				tmp[1]=(uint8_t)period_ms_int & 0xFF; //LSB
			}
			lidar->step_l1 = 8;
		}
		break;
	case 8:
		st = writeBytes(_i2c, lidar, SYSTEM_INTERMEASUREMENT_PERIOD, tmp, 2);
		if (st) {
			lidar->step_l1 = 10;
		}
		break;
	case 10:
		st = writeOneByte(_i2c, lidar, SYSRANGE_START, 0x04);  // VL53L0X_REG_SYSRANGE_MODE_TIMED
		if (st) {
			lidar->step_l1 = 0;
			return 1;
		}
		break;
	case 11:
		// continuous back-to-back mode
		st = writeOneByte(_i2c, lidar, SYSRANGE_START, 0x02);  // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
		if (st) {
			lidar->step_l1 = 0;
			return 1;
		}
		break;
	default:
		lidar->step_l1 = 0;
		break;
	}
	return 0;
}
/*****************************************************************
  * @brief	OK
  * @param
  * @retval
  */
uint8_t stopContinuous(I2C_IRQ_Connection_t *_i2c, VL53L0X *lidar)
{
	uint8_t st;
	switch (lidar->step_l1) {
	case 0:
		st = writeOneByte(_i2c, lidar, SYSRANGE_START, 0x01);  // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT
		if (st) {
			lidar->step_l1 = 1;
		}
		break;
	case 1:
		st = writeOneByte(_i2c, lidar, INTERNAL_TUNING_2, 0x01);
		if (st) {
			lidar->step_l1 = 2;
		}
		break;
	case 2:
		st = writeOneByte(_i2c, lidar, SYSRANGE_START, 0x00);
		if (st) {
			lidar->step_l1 = 3;
		}
		break;
	case 3:
		st = writeOneByte(_i2c, lidar, INTERNAL_TUNING_1, 0x00);
		if (st) {
			lidar->step_l1 = 4;
		}
		break;
	case 4:
		st = writeOneByte(_i2c, lidar, SYSRANGE_START, 0x01);
		if (st) {
			lidar->step_l1 = 5;
		}
		break;
	case 5:
		st = writeOneByte(_i2c, lidar, INTERNAL_TUNING_2, 0x00);
		if (st) {
			lidar->step_l1 = 0;
			return 1;
		}
		break;
	default:
		lidar->step_l1 = 0;
	}
	return 0;
}
/*****************************************************************
  * @brief	OK
  * @param
  * @retval
  */
uint8_t readRangeContinuousMillimeters(I2C_IRQ_Connection_t *_i2c, VL53L0X *lidar)
{
	uint8_t st;
	static uint8_t dt[2];
	switch (lidar->step_l1) {
	case 0:
		st = readOneByte(_i2c, lidar, RESULT_INTERRUPT_STATUS, &lidar->tmp);
		if (st){
			if ((lidar->tmp & 0x07)) {
				lidar->step_l1 = 1;
			}
			else if (checkTimeoutExpired(lidar)) {//timeout
				lidar->timeoutFlag = 1;
				//lidar->range = 65535;
				lidar->status = DEVICE_ERROR;
				lidar->step_l1 = 2;
			}
		}
		break;
	case 1:
		st = readBytes(_i2c, lidar, RESULT_RANGE_STATUS + 10, &dt, 2);
		if (st) {
			lidar->range = __CONCAT_BYTES(dt[0], dt[1]);
			lidar->status = DEVICE_DONE;
			lidar->step_l1 = 2;
		}
		break;
	case 2:
		st = writeOneByte(_i2c, lidar, SYSTEM_INTERRUPT_CLEAR, 0x01);
		if (st) {
			startTimeout(lidar);
			lidar->step_l1 = 0;
			return 1;
		}
		break;
	default:
		lidar->step_l1 = 0;
	}
	return 0;
}
/*****************************************************************
  * @brief	OK
  * @param
  * @retval
  */
uint8_t readRangeSingleMillimeters(I2C_IRQ_Connection_t *_i2c, VL53L0X *lidar)
{
	uint8_t st;
	switch (lidar->step_l2) {
	case 0:
		st = writeOneByte(_i2c, lidar, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
		if (st) {
			lidar->step_l2 = 1;
		}
		break;
	case 1:
		st = writeOneByte(_i2c, lidar, INTERNAL_TUNING_2, 0x01);
		if (st) {
			lidar->step_l2 = 2;
		}
		break;
	case 2:
		st = writeOneByte(_i2c, lidar, SYSRANGE_START, 0x00);
		if (st) {
			lidar->step_l2 = 3;
		}
		break;
	case 3:
		st = writeOneByte(_i2c, lidar, INTERNAL_TUNING_1, lidar->stop_variable);
		if (st) {
			lidar->step_l2 = 4;
		}
		break;
	case 4:
		st = writeOneByte(_i2c, lidar, SYSRANGE_START, 0x01);
		if (st) {
			lidar->step_l2 = 5;
		}
		break;
	case 5:
		st = writeOneByte(_i2c, lidar, INTERNAL_TUNING_2, 0x00);
		if (st) {
			lidar->step_l2 = 6;
		}
		break;
	case 6:
		st = writeOneByte(_i2c, lidar, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
		if (st) {
			lidar->step_l2 = 7;
		}
		break;
	case 7:
		st = writeOneByte(_i2c, lidar, SYSRANGE_START, 0x01);
		if (st) {
			startTimeout(lidar);
			lidar->step_l2 = 8;
		}
		break;
	case 8:// "Wait until start bit has been cleared"
		st = readOneByte(_i2c, lidar, SYSRANGE_START, &lidar->tmp);
		if (st) {
			if ((lidar->tmp & 0x01) == 0) {
				lidar->step_l2 = 9;
			}
			else if (checkTimeoutExpired(lidar)) {
				lidar->step_l2 = 0;
				lidar->status = DEVICE_FAULTH;
				return 1;
			}
		}
		break;
	case 9:
		st = readRangeContinuousMillimeters(_i2c, lidar);
		if (st) {
			lidar->step_l2 = 0;
			return 1;
		}
		break;
	default:
		lidar->step_l2 = 0;
		return 0;
	}
	return 0;
}
//--------------------------------------------------------------
uint8_t VL_Init(I2C_IRQ_Connection_t *_i2c, VL53L0X *lidar) {
	uint8_t st;
	// VL53L0X_DataInit() begin
	switch (lidar->step_l3) {
	case 0://+
		st = readOneByte(_i2c, lidar, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, &lidar->tmp);
		if (st) {
			lidar->step_l3 = 1;
		}
		break;
	case 1:// always 2.8v
		st = writeOneByte(_i2c, lidar, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, lidar->tmp | 0x01);  // set bit 0
		if (st) {
			lidar->step_l3 = 2;
		}
		break;
	case 2:  //Set I2C standard mode
		st = writeOneByte(_i2c, lidar, 0x88, 0x00);
		if (st) {
			lidar->step_l3 = 3;
		}
		break;
	case 3:
		st = writeOneByte(_i2c, lidar, 0x80, 0x01);
		if (st) {
			lidar->step_l3 = 4;
		}
		break;
	case 4:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x01);
		if (st) {
			lidar->step_l3 = 5;
		}
		break;
	case 5:
		st = writeOneByte(_i2c, lidar, 0x00, 0x00);
		if (st) {
			lidar->step_l3 = 6;
		}
		break;
	case 6://stop variable 0x11
		st = readOneByte(_i2c, lidar, 0x91, &lidar->stop_variable);
		if (st) {
			lidar->step_l3 = 7;
		}
		break;
	case 7:
		st = writeOneByte(_i2c, lidar, 0x00, 0x01);
		if (st) {
			lidar->step_l3 = 8;
		}
		break;
	case 8:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x00);
		if (st) {
			lidar->step_l3 = 9;
		}
		break;
	case 9:
		st = writeOneByte(_i2c, lidar, 0x80, 0x00);
		if (st) {
			lidar->step_l3 = 10;
		}
		break;
	case 10:// disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks//+
		st = readOneByte(_i2c, lidar, MSRC_CONFIG_CONTROL, &lidar->tmp);
		if (st) {
			lidar->step_l3 = 11;
		}
		break;//tmp = 0x32
	case 11:
		st = writeOneByte(_i2c, lidar, MSRC_CONFIG_CONTROL, lidar->tmp | 0x12);
		if (st) {
			lidar->step_l3 = 12;
		}
		break;
	case 12:
		st = setSignalRateLimit(_i2c, lidar, 32);
		if (st) {
			lidar->step_l3 = 13;
		}
		break;
	case 13:
		st = writeOneByte(_i2c, lidar, SYSTEM_SEQUENCE_CONFIG, 0xFF);
		if (st) {
			lidar->step_l3 = 14;
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
			lidar->step_l3 = 15;
		}
		break;
	case 15:
		st = readBytes(_i2c, lidar, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, &lidar->ref_spad_map, 6);
		if (st) {
			lidar->step_l3 = 16;
		}
		break;
		// -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)
	case 16:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x01);
		if (st) {
			lidar->step_l3 = 17;
		}
		break;
	case 17:
		st = writeOneByte(_i2c, lidar, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
		if (st) {
			lidar->step_l3 = 18;
		}
		break;
	case 18:
		st = writeOneByte(_i2c, lidar, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
		if (st) {
			lidar->step_l3 = 19;
		}
		break;
	case 19:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x00);
		if (st) {
			lidar->step_l3 = 20;
		}
		break;
	case 20:
		st = writeOneByte(_i2c, lidar, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);
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
			lidar->step_l3 = 21;
		}
		break;
	case 21:
		st = writeBytes(_i2c, lidar, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, lidar->ref_spad_map, 6);
		if (st) {
			lidar->step_l3 = 22;
		}
		break;
		// -- VL53L0X_set_reference_spads() end

		// -- VL53L0X_load_tuning_settings() begin
		// DefaultTuningSettings from vl53l0x_tuning.h
	case 22:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x01);
		if (st) {
			lidar->step_l3 = 23;
		}
		break;
	case 23:
		st = writeOneByte(_i2c, lidar, 0x00, 0x00);
		if (st) {
			lidar->step_l3 = 24;
		}
		break;
	case 24:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x00);
		if (st) {
			lidar->step_l3 = 25;
		}
		break;
	case 25:
		st = writeOneByte(_i2c, lidar, 0x09, 0x00);
		if (st) {
			lidar->step_l3 = 26;
		}
		break;
	case 26:
		st = writeOneByte(_i2c, lidar, 0x10, 0x00);
		if (st) {
			lidar->step_l3 = 27;
		}
		break;
	case 27:
		st = writeOneByte(_i2c, lidar, 0x11, 0x00);
		if (st) {
			lidar->step_l3 = 28;
		}
		break;//+
	case 28:
		st = writeOneByte(_i2c, lidar, 0x24, 0x01);
		if (st) {
			lidar->step_l3 = 29;
		}
		break;
	case 29:
		st = writeOneByte(_i2c, lidar, 0x25, 0xFF);
		if (st) {
			lidar->step_l3 = 30;
		}
		break;
	case 30:
		st = writeOneByte(_i2c, lidar, 0x75, 0x00);
		if (st) {
			lidar->step_l3 = 31;
		}
		break;
	case 31:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x01);
		if (st) {
			lidar->step_l3 = 32;
		}
		break;
	case 32:
		st = writeOneByte(_i2c, lidar, 0x4E, 0x2C);
		if (st) {
			lidar->step_l3 = 33;
		}
		break;
	case 33:
		st = writeOneByte(_i2c, lidar, 0x48, 0x00);
		if (st) {
			lidar->step_l3 = 34;
		}
		break;
	case 34:
		st = writeOneByte(_i2c, lidar, 0x30, 0x20);
		if (st) {
			lidar->step_l3 = 35;
		}
		break;
	case 35:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x00);
		if (st) {
			lidar->step_l3 = 36;
		}
		break;
	case 36:
		st = writeOneByte(_i2c, lidar, 0x30, 0x09);
		if (st) {
			lidar->step_l3 = 37;
		}
		break;
	case 37:
		st = writeOneByte(_i2c, lidar, 0x54, 0x00);
		if (st) {
			lidar->step_l3 = 38;
		}
		break;
	case 38:
		st = writeOneByte(_i2c, lidar, 0x31, 0x04);
		if (st) {
			lidar->step_l3 = 39;
		}
		break;
	case 39:
		st = writeOneByte(_i2c, lidar, 0x32, 0x03);
		if (st) {
			lidar->step_l3 = 40;
		}
		break;
	case 40:
		st = writeOneByte(_i2c, lidar, 0x40, 0x83);
		if (st) {
			lidar->step_l3 = 41;
		}
		break;
	case 41:
		st = writeOneByte(_i2c, lidar, 0x46, 0x25);
		if (st) {
			lidar->step_l3 = 42;
		}
		break;
	case 42:
		st = writeOneByte(_i2c, lidar, 0x60, 0x00);
		if (st) {
			lidar->step_l3 = 43;
		}
		break;
	case 43:
		st = writeOneByte(_i2c, lidar, 0x27, 0x00);
		if (st) {
			lidar->step_l3 = 44;
		}
		break;
	case 44:
		st = writeOneByte(_i2c, lidar, 0x50, 0x06);
		if (st) {
			lidar->step_l3 = 45;
		}
		break;
	case 45:
		st = writeOneByte(_i2c, lidar, 0x51, 0x00);
		if (st) {
			lidar->step_l3 = 46;
		}
		break;
	case 46:
		st = writeOneByte(_i2c, lidar, 0x52, 0x96);
		if (st) {
			lidar->step_l3 = 47;
		}
		break;
	case 47:
		st = writeOneByte(_i2c, lidar, 0x56, 0x08);
		if (st) {
			lidar->step_l3 = 48;
		}
		break;
	case 48:
		st = writeOneByte(_i2c, lidar, 0x57, 0x30);
		if (st) {
			lidar->step_l3 = 49;
		}
		break;
	case 49:
		st = writeOneByte(_i2c, lidar, 0x61, 0x00);
		if (st) {
			lidar->step_l3 = 50;
		}
		break;
	case 50:
		st = writeOneByte(_i2c, lidar, 0x62, 0x00);
		if (st) {
			lidar->step_l3 = 51;
		}
		break;
	case 51:
		st = writeOneByte(_i2c, lidar, 0x64, 0x00);
		if (st) {
			lidar->step_l3 = 52;
		}
		break;
	case 52:
		st = writeOneByte(_i2c, lidar, 0x65, 0x00);
		if (st) {
			lidar->step_l3 = 53;
		}
		break;
	case 53:
		st = writeOneByte(_i2c, lidar, 0x66, 0xA0);
		if (st) {
			lidar->step_l3 = 54;
		}
		break;
	case 54:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x01);
		if (st) {
			lidar->step_l3 = 55;
		}
		break;
	case 55:
		st = writeOneByte(_i2c, lidar, 0x22, 0x32);
		if (st) {
			lidar->step_l3 = 56;
		}
		break;
	case 56:
		st = writeOneByte(_i2c, lidar, 0x47, 0x14);
		if (st) {
			lidar->step_l3 = 57;
		}
		break;
	case 57:
		st = writeOneByte(_i2c, lidar, 0x49, 0xFF);
		if (st) {
			lidar->step_l3 = 58;
		}
		break;
	case 58:
		st = writeOneByte(_i2c, lidar, 0x4A, 0x00);
		if (st) {
			lidar->step_l3 = 59;
		}
		break;
	case 59:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x00);
		if (st) {
			lidar->step_l3 = 60;
		}
		break;
	case 60:
		st = writeOneByte(_i2c, lidar, 0x7A, 0x0A);
		if (st) {
			lidar->step_l3 = 61;
		}
		break;
	case 61:
		st = writeOneByte(_i2c, lidar, 0x7B, 0x00);
		if (st) {
			lidar->step_l3 = 62;
		}
		break;
	case 62:
		st = writeOneByte(_i2c, lidar, 0x78, 0x21);
		if (st) {
			lidar->step_l3 = 63;
		}
		break;
	case 63:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x01);
		if (st) {
			lidar->step_l3 = 64;
		}
		break;
	case 64:
		st = writeOneByte(_i2c, lidar, 0x23, 0x34);
		if (st) {
			lidar->step_l3 = 65;
		}
		break;
	case 65:
		st = writeOneByte(_i2c, lidar, 0x42, 0x00);
		if (st) {
			lidar->step_l3 = 66;
		}
		break;
	case 66:
		st = writeOneByte(_i2c, lidar, 0x44, 0xFF);
		if (st) {
			lidar->step_l3 = 67;
		}
		break;
	case 67:
		st = writeOneByte(_i2c, lidar, 0x45, 0x26);
		if (st) {
			lidar->step_l3 = 68;
		}
		break;
	case 68:
		st = writeOneByte(_i2c, lidar, 0x46, 0x05);
		if (st) {
			lidar->step_l3 = 69;
		}
		break;
	case 69:
		st = writeOneByte(_i2c, lidar, 0x40, 0x40);
		if (st) {
			lidar->step_l3 = 70;
		}
		break;
	case 70:
		st = writeOneByte(_i2c, lidar, 0x0E, 0x06);
		if (st) {
			lidar->step_l3 = 71;
		}
		break;
	case 71:
		st = writeOneByte(_i2c, lidar, 0x20, 0x1A);
		if (st) {
			lidar->step_l3 = 72;
		}
		break;
	case 72:
		st = writeOneByte(_i2c, lidar, 0x43, 0x40);
		if (st) {
			lidar->step_l3 = 73;
		}
		break;
	case 73:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x00);
		if (st) {
			lidar->step_l3 = 74;
		}
		break;
	case 74:
		st = writeOneByte(_i2c, lidar, 0x34, 0x03);
		if (st) {
			lidar->step_l3 = 75;
		}
		break;
	case 75:
		st = writeOneByte(_i2c, lidar, 0x35, 0x44);
		if (st) {
			lidar->step_l3 = 76;
		}
		break;
	case 76:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x01);
		if (st) {
			lidar->step_l3 = 77;
		}
		break;
	case 77:
		st = writeOneByte(_i2c, lidar, 0x31, 0x04);
		if (st) {
			lidar->step_l3 = 78;
		}
		break;
	case 78:
		st = writeOneByte(_i2c, lidar, 0x4B, 0x09);
		if (st) {
			lidar->step_l3 = 79;
		}
		break;
	case 79:
		st = writeOneByte(_i2c, lidar, 0x4C, 0x05);
		if (st) {
			lidar->step_l3 = 80;
		}
		break;
	case 80:
		st = writeOneByte(_i2c, lidar, 0x4D, 0x04);
		if (st) {
			lidar->step_l3 = 81;
		}
		break;
	case 81:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x00);
		if (st) {
			lidar->step_l3 = 82;
		}
		break;
	case 82:
		st = writeOneByte(_i2c, lidar, 0x44, 0x00);
		if (st) {
			lidar->step_l3 = 83;
		}
		break;
	case 83:
		st = writeOneByte(_i2c, lidar, 0x45, 0x20);
		if (st) {
			lidar->step_l3 = 84;
		}
		break;
	case 84:
		st = writeOneByte(_i2c, lidar, 0x47, 0x08);
		if (st) {
			lidar->step_l3 = 85;
		}
		break;
	case 85:
		st = writeOneByte(_i2c, lidar, 0x48, 0x28);
		if (st) {
			lidar->step_l3 = 86;
		}
		break;
	case 86:
		st = writeOneByte(_i2c, lidar, 0x67, 0x00);
		if (st) {
			lidar->step_l3 = 87;
		}
		break;
	case 87:
		st = writeOneByte(_i2c, lidar, 0x70, 0x04);
		if (st) {
			lidar->step_l3 = 88;
		}
		break;
	case 88:
		st = writeOneByte(_i2c, lidar, 0x71, 0x01);
		if (st) {
			lidar->step_l3 = 89;
		}
		break;
	case 89:
		st = writeOneByte(_i2c, lidar, 0x72, 0xFE);
		if (st) {
			lidar->step_l3 = 90;
		}
		break;
	case 90:
		st = writeOneByte(_i2c, lidar, 0x76, 0x00);
		if (st) {
			lidar->step_l3 = 91;
		}
		break;
	case 91:
		st = writeOneByte(_i2c, lidar, 0x77, 0x00);
		if (st) {
			lidar->step_l3 = 92;
		}
		break;
	case 92:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x01);
		if (st) {
			lidar->step_l3 = 93;
		}
		break;
	case 93:
		st = writeOneByte(_i2c, lidar, 0x0D, 0x01);
		if (st) {
			lidar->step_l3 = 94;
		}
		break;
	case 94:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x00);
		if (st) {
			lidar->step_l3 = 95;
		}
		break;
	case 95:
		st = writeOneByte(_i2c, lidar, 0x80, 0x01);
		if (st) {
			lidar->step_l3 = 96;
		}
		break;
	case 96:
		st = writeOneByte(_i2c, lidar, 0x01, 0xF8);
		if (st) {
			lidar->step_l3 = 97;
		}
		break;
	case 97:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x01);
		if (st) {
			lidar->step_l3 = 98;
		}
		break;
	case 98:
		st = writeOneByte(_i2c, lidar, 0x8E, 0x01);
		if (st) {
			lidar->step_l3 = 99;
		}
		break;
	case 99:
		st = writeOneByte(_i2c, lidar, 0x00, 0x01);
		if (st) {
			lidar->step_l3 = 100;
		}
		break;
	case 100:
		st = writeOneByte(_i2c, lidar, 0xFF, 0x00);
		if (st) {
			lidar->step_l3 = 101;
		}
		break;
	case 101:
		st = writeOneByte(_i2c, lidar, 0x80, 0x00);
		if (st) {
			lidar->step_l3 = 102;
		}
		break;
	case 102:
		st = writeOneByte(_i2c, lidar, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
		if (st) {
			lidar->step_l3 = 103;
		}
		break;
	case 103:
		st = readOneByte(_i2c, lidar, GPIO_HV_MUX_ACTIVE_HIGH, &lidar->tmp);
		if (st) {
			lidar->step_l3 = 104;
		}
		break;
	case 104:
		st = writeOneByte(_i2c, lidar, GPIO_HV_MUX_ACTIVE_HIGH, lidar->tmp & ~0x10);  // active low
		if (st) {
			lidar->step_l3 = 105;
		}
		break;
	case 105:
		st = writeOneByte(_i2c, lidar, SYSTEM_INTERRUPT_CLEAR, 0x01);
		if (st) {
			lidar->step_l3 = 106;
		}
		break;
		// -- VL53L0X_SetGpioConfig() end

	case 106:
		st = getMeasurementTimingBudget(_i2c, lidar);
		if (st) {
			lidar->step_l3 = 107;
		}
		break;
		// "Disable MSRC and TCC by default"
		// MSRC = Minimum Signal Rate Check
		// TCC = Target CentreCheck
		// -- VL53L0X_SetSequenceStepEnable() begin
	case 107:
		st = writeOneByte(_i2c, lidar, SYSTEM_SEQUENCE_CONFIG, 0xE8);
		if (st) {
			lidar->step_l3 = 108;
		}
		break;
		// -- VL53L0X_SetSequenceStepEnable() end

	case 108:  // "Recalculate timing budget"
		st = setMeasurementTimingBudget(_i2c, lidar, lidar->measurement_timing_budget_us);
		if (st) {
			lidar->step_l3 = 109;
		}
		break;
		// VL53L0X_StaticInit() end
		// VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())
	case 109:  // -- VL53L0X_perform_vhv_calibration() begin
		st = writeOneByte(_i2c, lidar, SYSTEM_SEQUENCE_CONFIG, 0x01);
		if (st) {
			lidar->step_l3 = 110;
		}
		break;
	case 110:  //  performSingleRefCalibration(_i2c,lidar,0x40);
		st = performSingleRefCalibration(_i2c, lidar, 0x40);
		if (st) {
			lidar->step_l3 = 111;
		}
		break;
		// -- VL53L0X_perform_vhv_calibration() end
	case 111:  // -- VL53L0X_perform_phase_calibration() begin
		st = writeOneByte(_i2c, lidar, SYSTEM_SEQUENCE_CONFIG, 0x02);
		if (st) {
			lidar->step_l3 = 112;
		}
		break;
	case 112:  //  performSingleRefCalibration(_i2c,lidar,0x00);
		st = performSingleRefCalibration(_i2c, lidar, 0x00);
		if (st) {
			lidar->step_l3 = 113;
		}
		break;
		// -- VL53L0X_perform_phase_calibration() end
	case 113:  // "restore the previous Sequence Config"
		st = writeOneByte(_i2c, lidar, SYSTEM_SEQUENCE_CONFIG, 0xE8);
		if (st) {
			lidar->step_l3 = 0;
			setTimeout(lidar, 50);
			startTimeout(lidar);
			return 1;
		}
		break;
		// VL53L0X_PerformRefCalibration() end
	default:
		lidar->step_l3 = 0;
		break;
	}
	return 0;
}

uint16_t getSmoothRange(VL53L0X *lidar) {
	//static_assert(VL53L0x_SHOOTH_DEEP % 2 == 0, "smooth deep most be power of 2"); //проверка степени двойки
	lidar->smoothRange = alphabeta(lidar->range, lidar->smoothRange, 8);
	return lidar->smoothRange;
}

