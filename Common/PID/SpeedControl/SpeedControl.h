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

#include "stdint.h"
#include "stddef.h"
#include "math.h"
#include "Function/Function.h"
#include "PID/PID_Wiki/PID_Moto.h"

#define WHEEL_RAD_mm	    20	        // радиуc колеcа, мм
#define WHEEL_PULSE_COUNT   900          // число импульсов энкодера на один оборот колеса
#define BASE_mm 		        122         // раccтояние между осями колес, мм
#define WALL_LIMIT 		      300  	      // предельное раccто�?ние до преп�?т�?тви�?, мм
#define K_I				          10  	      // ко�?ффициент интегральной �?о�?тавл�?ющей линейной �?коро�?ти
#define ERROR_m 		        BASE_m / 2	// точно�?ть до�?тижени�? целевой точки мм

typedef enum direction {
	STOP,
	FORWARD,
	BACKWARD
} direction_t;

typedef enum moveMode {
	WAITING,
	NORMAL_DRIVE,
	EMERGENCY,
	DOUB_STONE,
	SEARCH
} moveMode_t;

typedef struct point {
	float x;	//координата Х
	float y;	//координата Y
} point_t;

typedef struct setPoint {
	point_t target;		//координаты робота
	float speedLeft;  	//PWM value left
	float speedRight;  //PWM value right
} setPoint_t;

typedef struct Drive {
	moveMode_t mode;		//режим движения
	uint8_t step;			//шаг автомата состояний
	point_t coord;			//координаты робота
	float speedL;			//left wheel speed
	float speedR;			//right wheel speed
	float speedRobot;		//main speed
	float distance;			//distance to target
	float fullPath;			//
	float intLength;		//интеграл пути
	float bearing;  		//пеленг (theta) и�?комый
	float course;  			//psi кур�? робота, угол в �?и�?теме координат
	float angle;  			//курcовой угол
	float iAngle;			//интеграл поправки курсового угла
	float gyroSpeed;		//угловая скорость
	//wheel_t wheelLeft;	//левое колесо
	//wheel_t wheelRight;	//правое колесо
	setPoint_t SP;			//заданнные значения для движения
} Drive_t;


//TODO продумать корректную систему проверки перехода через 0 и максимум энкодера при смене направления вращения колеса
float WheelSpeedMeasure(uint32_t deltaPulse, uint32_t deltaTime);
//TODO продумать гистерезис чтобы не было колебаний скорости в граничных точках
int32_t WheelSpeedZeroLimiter(int32_t sp, int32_t low, int32_t hi);

