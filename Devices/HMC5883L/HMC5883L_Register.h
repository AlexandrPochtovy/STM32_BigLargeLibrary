/*********************************************************************************
   Original author: Alexandr Pochtovy<alex.mail.prime@gmail.com>

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
   
 	HMC5883L_Register.h
	Created on: 31.01.2022
 ********************************************************************************/

#ifndef HMC5883L_REGISTER_H_
#define HMC5883L_REGISTER_H_

enum HMC5883L_Registers {
   HMC5883L_REG_CONFIG_A	= 0x00,
   HMC5883L_REG_CONFIG_B	= 0x01,
   HMC5883L_REG_MODE			=0x02,
   HMC5883L_REG_OUT_X_M		=0x03,
   HMC5883L_REG_OUT_X_L		=0x04,
   HMC5883L_REG_OUT_Z_M		=0x05,
   HMC5883L_REG_OUT_Z_L		=0x06,
   HMC5883L_REG_OUT_Y_M		=0x07,
   HMC5883L_REG_OUT_Y_L		=0x08,
   HMC5883L_REG_STATUS        =0x09,
   HMC5883L_REG_IDENT_A       =0x0A,
   HMC5883L_REG_IDENT_B       =0x0B,
   HMC5883L_REG_IDENT_C       =0x0C
};

enum HMC5883l_Samples {
    HMC5883L_SAMPLES_8     = 0x60,
    HMC5883L_SAMPLES_4     = 0x40,
    HMC5883L_SAMPLES_2     = 0x20,
    HMC5883L_SAMPLES_1     = 0x00
};

enum HMC5883l_DataRate {
    HMC5883L_DATARATE_75HZ		= 0x18,
    HMC5883L_DATARATE_30HZ		= 0x14,
    HMC5883L_DATARATE_15HZ		= 0x10,
    HMC5883L_DATARATE_7_5HZ		= 0x0C,
    HMC5883L_DATARATE_3HZ			= 0x08,
    HMC5883L_DATARATE_1_5HZ		= 0x04,
    HMC5883L_DATARATE_0_75_HZ	= 0x00
};

enum HMC5883l_MeasMode {
    HMC5883L_NORMAL			= 0x00,
    HMC5883L_POS_BIAS			= 0x01,
    HMC5883L_NEG_BIAS		= 0x02,
};

enum HMC5883l_GAIN {
    HMC5883L_GAIN_8_1GA	= 0xE0,
    HMC5883L_GAIN_5_6GA	= 0xC0,
    HMC5883L_GAIN_4_7GA	= 0xA0,
    HMC5883L_GAIN_4GA		= 0x80,
    HMC5883L_GAIN_2_5GA	= 0x60,
    HMC5883L_GAIN_1_9GA	= 0x40,
    HMC5883L_GAIN_1_3GA	= 0x20,
    HMC5883L_GAIN_0_88GA	= 0x00
};

enum HMC5883l_OperMode {
	HMC5883L_IDLE					= 0x02,
    HMC5883L_SINGLE				= 0x01,
    HMC5883L_CONTINOUS	= 0x00
};

#endif /* HMC5883L_REGISTER_H_ */
