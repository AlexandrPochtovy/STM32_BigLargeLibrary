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

 * SpeedControl.h
 * Created on: Sep 16, 2023
*********************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "Function/Function.h"
#include "Buffers/FILObuffer/FILObuffer.h"
#include "PID/PID_Wiki/PID_Moto.h"

#define WHEEL_RAD_mm	    	24u						// wheel radius in mm
#define WHEEL_PULSE_COUNT   1800u					// number of encoder pulses per wheel revolution
#define BASE_mm 		        122u					// distance between wheels of one axle in mm
#define WALL_LIMIT_mm	      300u					// permissible distance to object in mm
#define K_I				          10u						// integral coefficient
#define ERROR_mm 		        (BASE_mm / 2)	// position error in mm
#define kDI_step						0.1f

typedef enum direction {
	STOP,
	FORWARD,
	BACKWARD
	} direction_t;

typedef struct Wheel {
	direction_t direction;
	float speedSP;
	float speedAct;
	} Wheel_t;

typedef enum moveMode {
	WAITING,
	NORMAL_DRIVE,
	EMERGENCY,
	DOUB_STONE,
	SEARCH
	} moveMode_t;

typedef struct point {
	float x;	//Х-coordinate
	float y;	//Y-coordinate
	} point_t;

typedef struct setPoint {
	point_t target;	//robot target coordinates
	float pwmLeft;	//PWM value left
	float pwmRight;	//PWM value right
	} setPoint_t;

typedef struct Drive {
	moveMode_t mode;	//main move mode
	uint8_t step;			//machine-state's step for processing move mode
	point_t position;	//actual coordinates of robot
	Wheel_t WL;				//left wheel actual data
	Wheel_t WR;				//right wheel actual data
	float speed;	//main speed
	float distance;		//distance to target
	float fullPath;		//full patch, integral
	float intLength;	//integral length for move to target
	float bearing;		// theta
	float course;			//psi кур�? робота, угол в �?и�?теме координат
	float angle;			//course angle to target point
	float iAngle;			//
	float gyroSpeed;	//
	setPoint_t SP;		//заданнные значения для движения
	} Drive_t;

//TODO продумать корректную систему проверки перехода через 0 и максимум энкодера при смене направления вращения колеса
float WheelSpeedMeasure(uint32_t deltaPulse, uint32_t deltaTime);

size_t WheelSpeedZeroLimiter(size_t act, size_t sp, size_t low, size_t hi);

