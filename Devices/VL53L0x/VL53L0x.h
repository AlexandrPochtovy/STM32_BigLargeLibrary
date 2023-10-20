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

 * 	VL53L0x.h
 *	Created on: Jul 25, 2022
 ********************************************************************************/

#ifndef _VL53L0X_H_
#define _VL53L0X_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "VL53L0x_Registers.h"
#include "Function/Function.h"
#include "I2C_API.h"

  /**********************************************************************
  *                       TYPEDEF & ENUM                                *
  ***********************************************************************/
  enum VL53L0x_ADDRESS {
    VL53L0x_ADDR_DEFAULT = 0x52,//0b01010010
    VL53L0x_EXPECTED_DEVICE_ID = 0xEE
    };

  typedef enum __vcselPeriodType {
    VcselPeriodPreRange,
    VcselPeriodFinalRange
    } vcselPeriodType;

  typedef struct __SequenceStepEnables {
    uint8_t tcc;
    uint8_t msrc;
    uint8_t dss;
    uint8_t pre_range;
    uint8_t final_range;
    } SequenceStepEnables;

  typedef struct __SequenceStepTimeouts {
    uint16_t pre_range_vcsel_period_pclks;
    uint16_t final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks;
    uint16_t pre_range_mclks;
    uint16_t final_range_mclks;
    uint32_t msrc_dss_tcc_us;
    uint32_t pre_range_us;
    uint32_t final_range_us;
    } SequenceStepTimeouts;

  typedef struct __VL53L0X {
    uint8_t addr;
    volatile DeviceStatus_t status;
    GPIO_TypeDef *XShut_Port;	//xshut port
    uint32_t XShut_Pin;				//xshut pin
    const uint8_t errLimit;
    uint8_t errCount;
    uint8_t stepL1;
    uint8_t stepL2;
    uint8_t stepL3;
    uint8_t modelID;
    uint8_t revisionID;
    uint16_t count_timeout;
    uint16_t limit_timeout;
    uint8_t timeoutFlag;
    uint8_t stop_variable; // read by unused when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
    uint32_t sp_budget_us;
    uint32_t measurement_timing_budget_us;
    uint8_t spad_count;
    uint8_t spad_type_is_aperture;
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;
    uint8_t sequence_config;
    uint8_t vcsel_period_reg;
    uint16_t new_pre_range_timeout_mclks;
    uint16_t new_msrc_timeout_mclks;
    uint16_t new_final_range_timeout_mclks;
    uint8_t vcselPeriodValue;
    uint8_t ref_spad_map[6];
    uint8_t tmp8;
    uint16_t tmp16;
    uint16_t range;
    uint16_t smoothRange;
    } VL53L0x_t;

  /*****************************************************************
    * @brief init sensor's structure: set timeout, clear flags, etc
    * @param _i2c - pointer to I2C bus connection structure
    * @param dev - pointer to sensor's main structure
    * @retval none
    */
  void setupVL53L0X(VL53L0x_t *dev, uint16_t lim);

  /*****************************************************************
    * @brief write to sensor new I2C address and store it in main sensor's structure
    * @note there is a problem, the new address is recorded but the sensor does not respond to the new address
    * do not use!
    * @param _i2c - pointer to I2C bus connection structure
    * @param dev - pointer to sensor's main structure
    * @param new_addr - new I2C address
    * @retval 1 when complite
    */
  uint8_t setAddress(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev, const uint8_t new_addr);

  /*****************************************************************
    * @brief read sensor's I2C address from structure
    * @param dev - pointer to sensor's main structure
    * @retval actual i2c address
    */
  uint8_t getAddress(VL53L0x_t *dev);

  /*****************************************************************
    * @brief read sensor's model ID and store it in main sensor's structure
    * @note use it for connection check
    * @param _i2c - pointer to I2C bus connection structure
    * @param dev - pointer to sensor's main structure
    * @retval 1 when complite
    */
  uint8_t VL53_getModelId(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev);

  /*****************************************************************
    * @brief read sensor's revision ID and store it in main sensor's structure
    * @note use it for connection check
    * @param _i2c - pointer to I2C bus connection structure
    * @param dev - pointer to sensor's main structure
    * @retval 1 when complite
    */
  uint8_t getRevisionId(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev);

  /*****************************************************************
    * @brief set new timeout value to main sensor's structure
    * @param dev - pointer to sensor's main structure
    * @param timeout - new timeout value
    * @retval none
    */
  void setTimeout(VL53L0x_t *dev, const uint16_t timeout);

  /*****************************************************************
    * @brief get actual timeout value from main sensor's structure
    * @note useless feature, made to be
    * @param dev - pointer to sensor's main structure
    * @retval timeout value
    */
  uint16_t getTimeout(VL53L0x_t *dev);

  /*****************************************************************
    * @brief clear timeout's count and flags in main sensor's structure
    * @param dev - pointer to sensor's main structure
    * @retval timeout value
    */
  void startTimeout(VL53L0x_t *dev);

  /*****************************************************************
    * @brief Check if timeout is enabled (set to nonzero value) and has expired
    * @param dev - pointer to sensor's main structure
    * @retval timeout flag
    */
  uint8_t checkTimeoutExpired(VL53L0x_t *dev);

  /**********************************************************************
  *                       INERNAL FUNCTION                              *
  ***********************************************************************/
  /*****************************************************************
    * @brief Set the return signal rate limit check value in units of MCPS (mega counts per second).
    * "This represents the amplitude of the signal reflected from the
    * target and detected by the device"; setting this limit presumably determines
    * the minimum measurement necessary for the sensor to report a valid reading.
    * Setting a lower limit increases the potential range of the sensor but also
    * seems to increase the likelihood of getting an inaccurate reading because of
    * unwanted reflections from objects other than the intended target.
    * Defaults to 0.25 MCPS as initialized by the ST API and this library.
    * Write an arbitrary number of bytes from the given array to the sensor,
    * starting at the given register
    * OLD FUNCTION
    * bool setSignalRateLimit(VL53L0X * lidar,float limit_Mcps) {
    * uint16_t val;
    * if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }
    * val = (uint16_t)(limit_Mcps * (1 << 7));
    * Q9.7 fixed point format (9 integer bits, 7 fractional bits)
    * @param _i2c  - pointer to I2C bus connection structure
    * @param dev - pointer to sensor's main structure
    * @param limit_Mcps - time limit in Million capture per sesonds
    * @retval 1 when done
    */
  uint8_t setSignalRateLimit(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev, uint32_t limit_Mcps);

  /*****************************************************************
   * @brief Get the return signal rate limit check value in MCPS
   * Get reference SPAD (single photon avalanche diode) count and type
   * based on VL53L0X_get_info_from_device(), but only gets reference SPAD count and
   * type float getSignalRateLimit(VL53L0X * lidar) {
   *    return (float)readReg16Bit(_i2c,lidar,FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
   * }
   * @param _i2c  - pointer to I2C bus connection structure
   * @param dev - pointer to sensor's main structure
   * @retval 1 when done
   */
  uint8_t getSpadInfo(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev);

  /*****************************************************************
   * @brief Get sequence step enables based on VL53L0X_GetSequenceStepEnables()
   * @param _i2c  - pointer to I2C bus connection structure
   * @param dev - pointer to sensor's main structure
   * @retval 1 when done
   */
  uint8_t getSequenceStepEnables(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev);

  /*****************************************************************
   * @brief Get the VCSEL pulse period in PCLKs for the given period type based on VL53L0X_get_vcsel_pulse_period()
   * @param _i2c  - pointer to I2C bus connection structure
   * @param dev - pointer to sensor's main structure
   * @retval 1 when done
   */
  uint8_t getVcselPulsePeriod(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev, vcselPeriodType type);

  /*****************************************************************
   * @brief Get sequence step timeouts based on get_sequence_step_timeout(),
   * but gets all timeouts instead of just the requested one, and also stores intermediate values
   * @param _i2c  - pointer to I2C bus connection structure
   * @param dev - pointer to sensor's main structure
   * @retval 1 when done
   */
  uint8_t getSequenceStepTimeouts(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev);

  /*****************************************************************
   * @brief Get the measurement timing budget in microseconds
   * based on VL53L0X_get_measurement_timing_budget_micro_seconds()
   * @param _i2c  - pointer to I2C bus connection structure
   * @param dev - pointer to sensor's main structure
   * @retval 1 when done
   */
  uint8_t getMeasurementTimingBudget(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev);

  /*****************************************************************
   * @brief Set the measurement timing budget in microseconds, which is the time allowed for one measurement;
   * the ST API and this library take care of splitting the timing budget among the sub-steps in the ranging sequence.
   * A longer timing budget allows for more accurate measurements. Increasing the budget by a factor of N decreases
   * the range measurement standard deviation by a factor of sqrt(N).
   * Defaults to about 33 milliseconds; the minimum is 20 ms. based on VL53L0X_set_measurement_timing_budget_micro_seconds()
   * @param _i2c  - pointer to I2C bus connection structure
   * @param dev - pointer to sensor's main structure
   * @retval 1 when done
   */
  uint8_t setMeasurementTimingBudget(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev, uint32_t budget_us);

  /*****************************************************************
   * @brief based on VL53L0X_perform_single_ref_calibration() interrupt enable need
   * @param _i2c  - pointer to I2C bus connection structure
   * @param dev - pointer to sensor's main structure
   * @retval 1 when done
   */
  uint8_t performSingleRefCalibration(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev, uint8_t vhv_init_byte);

  /*****************************************************************
   * @brief Set the VCSEL (vertical cavity surface emitting laser)
   * pulse period for the given period type (pre-range or final range)
   * to the given value in PCLKs. Longer periods seem to increase the potential range of the sensor.
   * based on VL53L0X_set_vcsel_pulse_period()
   * @param _i2c  - pointer to I2C bus connection structure
   * @param dev - pointer to sensor's main structure
   * @param type - type of vcsel, valid values are (even numbers only):
   * pre:  12 to 18 (initialized default: 14)
   * final: 8 to 14 (initialized default: 10)
   * @param period_pclks - period vcsel type
   * @retval 1 when done
   */
  uint8_t setVcselPulsePeriod(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev, vcselPeriodType type, uint8_t period_pclks);

  /**********************************************************************
  *                       MEASURE FUNCTION                              *
  ***********************************************************************/
  /*****************************************************************
   * @brief Start continuous ranging measurements. If period_ms (optional) is 0 or not given,
   * continuous back-to-back mode is used (the sensor takes measurements as often as possible);
   * otherwise, continuous timed mode is used, with the given inter-measurement period in milliseconds
   * determining how often the sensor takes a measurement.
   * based on VL53L0X_StartMeasurement()
   * @param _i2c  - pointer to I2C bus connection structure
   * @param dev - pointer to sensor's main structure
   * @param period_ms - measuring period in msec
   * @retval 1 when done
   */
  uint8_t VL53_StartContinuous(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev, uint32_t period_ms);

  /*****************************************************************
   * @brief Stop continuous measurements based on VL53L0X_StopMeasurement()
   * @param _i2c  - pointer to I2C bus connection structure
   * @param dev - pointer to sensor's main structure
   * @retval 1 when done
   */
  uint8_t VL53_StopContinuous(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev);

  /*****************************************************************
   * @brief Returns a range reading in millimeters when continuous mode is active
   * readRangeSingleMillimeters() also calls this function after starting a single-shot range measurement)
   * @param _i2c  - pointer to I2C bus connection structure
   * @param dev - pointer to sensor's main structure
   * @retval 1 when done
   */
  uint8_t VL53_readRangeContinuousMillimeters(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev);

  /*****************************************************************
   * @brief Performs a single-shot range measurement and returns the reading in
   * millimeters based on VL53L0X_PerformSingleRangingMeasurement()
   * @param _i2c  - pointer to I2C bus connection structure
   * @param dev - pointer to sensor's main structure
   * @retval 1 when done
   */
  uint8_t readRangeSingleMillimeters(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev);

  /*****************************************************************
   * @brief Initialize sensor using sequence based on VL53L0X_DataInit(),
   * VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
   * This function does not perform reference SPAD calibration
   * (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
   * is performed by ST on the bare modules; it seems like that should work well
   * enough unless a cover glass is added.
   * If io_2v8 (optional) is true or not given, the sensor is configured for 2V8 mode.
   * Initialize sensor using sequence based on VL53L0X_DataInit(),
   * VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
   * This function does not perform reference SPAD calibration
   * (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
   * is performed by ST on the bare modules; it seems like that should work well
   * enough unless a cover glass is added.
   * If io_2v8 (optional) is true or not given, the sensor is configured for 2V8 mode.
   * @param _i2c  - pointer to I2C bus connection structure
   * @param dev - pointer to sensor's main structure
   * @retval 1 when done
   */
  uint8_t VL53L0x_Init(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev);


  uint8_t VL53L0X_ResetDevice(I2C_IRQ_Conn_t *_i2c, VL53L0x_t *dev);

  /*****************************************************************
   * @brief calculate smooth value of range
   * @param dev - pointer to sensor's main structure
   * @retval range in mm
   */
  uint16_t getSmoothRange(VL53L0x_t *dev);

#ifdef __cplusplus
  }
#endif

#endif

