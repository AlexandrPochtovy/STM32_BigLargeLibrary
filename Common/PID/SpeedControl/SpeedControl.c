
#include "SpeedControl.h"


	static int32_t x[32] = { 0 };
	static int32_t y[32] = { 0 };
	static filo_t pointX = { .buffer = (void *)&x, .buffer_size = 32, .ind = 0, .bytes_avail = 0, .state = BUFFER_FREE };
	static filo_t pointY = { .buffer = (void *)&y, .buffer_size = 32, .ind = 0, .bytes_avail = 0, .state = BUFFER_FREE };


static inline int32_t float_to_int_mm(float value) {
	return ( int32_t )(value * 1000);
	}

static inline float int_mm_to_float(int value) {
	return (( float )value) / 1000.0f;
	}

/*****************************************************************
	* @brief calculate distance's correction
	* @param range - distance to wall
	* @retval - distance for correction
	*/
uint16_t distCalc(uint16_t range) {
	if (range > 0) {
		if (range > WALL_LIMIT_mm) {
			return 0;
			}
		else {
			return WALL_LIMIT_mm - range;
			}
		}
	else {
		return 0;
		}
	}

/*****************************************************************
	* @brief положение ближайшей стены: 1 слева, -1 справа;
	* @param
	* @retval
	*/
int16_t distMaxCalc(uint16_t rangeL, uint16_t rangeR) {
	uint16_t dR = distCalc(rangeR);
	uint16_t dL = distCalc(rangeL);
	uint16_t tmp = dR >= dL ? dR : dL;
	return tmp * signum_t(dL - dR);
	}

/*****************************************************************
	* @brief обнуляет координаты робота и траекторию
	* @param
	* @retval
	*/
void ResetTrace(Drive_t* bot) {
	bot->position.x = 0;
	bot->position.y = 0;
	bot->distance = 0;
	bot->intLength = 0;
	}

/*****************************************************************
	* @brief добавляет промежуточную точку если объехать препятствие не удалось
	* @param
	* @retval
	*/
void AddIntermediatePoint(Drive_t* bot, filo_t* pointX, filo_t* pointY, float limitLen) {
	FILO_PutOne(pointX, float_to_int_mm(bot->SP.target.x));  //save SP x
	FILO_PutOne(pointY, float_to_int_mm(bot->SP.target.y));  //save SP y
	//check max distance
	float len = bot->distance > limitLen ? limitLen : bot->distance;
	bot->SP.target.x = bot->position.x + len * cosf(bot->course + bot->angle);
	bot->SP.target.y = bot->position.y + len * sinf(bot->course + bot->angle);
	}

/*****************************************************************
	* @brief считывает промежуточную точку и едет к ней
	* @param
	* @retval
	*/
void GetStoreTargetPoint(Drive_t* bot, filo_t* pointX, filo_t* pointY) {
	int32_t tmp;
	FILO_GetOne(pointX, &tmp);
	bot->SP.target.x = int_mm_to_float(tmp);
	FILO_GetOne(pointY, &tmp);
	bot->SP.target.y = int_mm_to_float(tmp);
	}
/*****************************************************************
	* @brief измеряет скорость колеса
	* @param
	* @retval
	*/
float WheelSpeedMeasure(uint32_t deltaPulse, uint32_t deltaTime) {
	return M_PI * ( float )(2 * WHEEL_RAD_mm * deltaPulse * 1000) / (WHEEL_PULSE_COUNT * deltaTime);
	}


/*****************************************************************
	* @brief гистерезис колебания скорости около нуля для избежания дрейфа и дребезга колес
	* @param
	* @retval
	*/
size_t WheelSpeedZeroLimiter(size_t act, size_t sp, size_t low, size_t hi) {
	if (sp > 0) {
		if (sp < low) {
			return 0;
			}
		else if (sp > hi) {
			return sp;
			}
		else {
			if (act > 0) {
				return sp;
				}
			else {
				return 0;
				}
			}
		}
	else if (sp < 0) {
		if (sp > -low) {
			return 0;
			}
		else if (sp < -hi) {
			return sp;
			}
		else {
			if (act < 0) {
				return sp;
				}
			else {
				return 0;
				}
			}
		}
	else {
		return 0;
		}
	}


uint8_t MoveDrive(Drive_t* bot, uint32_t dt, point_t point, int16_t distL, int16_t distR,
		uint16_t wall, uint8_t bamper, float gyroAngle) {
	uint8_t retValue = 0;

	if (bamper) { bot->mode = EMERGENCY; }
	float delthaX = bot->SP.target.x - bot->position.x;  					//расстояние до точки по оси X
	float delthaY = bot->SP.target.y - bot->position.y;  					//расстояние до точки по оси Y
	bot->distance = sqrtf(delthaY * delthaY + delthaX * delthaX);	//расстояние до точки назначения
	bot->bearing = atan2f(delthaY, delthaX);  										//пеленг - угол вектора из текущего положения к заданному
	//SpeedCalc(&bot->wheelLeft);																		//рассчет скорости левого колеса
	//SpeedCalc(&bot->wheelRight);																	//рассчет скорости правого колеса
	bot->speed = (bot->WL.speedAct + bot->WR.speedAct) / 2;				//линейная скорость робота мм/сек
	//угловая скорость робота, можно рассчитать из гироскопа
	bot->gyroSpeed = (bot->WR.speedAct - bot->WL.speedAct) / BASE_mm;	//угловая скорость робота рад/cек

	/*bot->course += bot->gyroSpeed * dt / 1000;  											//ра�?чет кур�?а - текущий угол поворота робота, рад
	if (bot->course > M_PI) {
		bot->course = bot->course - M_TWOPI;
	}
	else if (bot->course < -M_PI) {
		bot->course = bot->course + M_TWOPI;
	}*/
	bot->course = gyroAngle;//угол поворота взять из МЕМС датчика
	float delthaPath_m = bot->speed * dt / 1000;		//пройденный путь за цикл, может быть < 0
	float deltaPath_abs = fabsf(delthaPath_m);			//абсолютный пройденный путь за цикл
	bot->fullPath += deltaPath_abs;									//одометр
	bot->intLength = bot->intLength >= 10 ? 10 : bot->intLength + deltaPath_abs;
	bot->position.x += delthaPath_m * cosf(bot->course);
	bot->position.y += delthaPath_m * sinf(bot->course);

	int16_t kDist = distMaxCalc(distL, distR);
	switch (bot->mode) {
		/* здесь определяем можно ли двигаться: ждем настройки датчиков, ждем точку назначения
		 * проверяем что в буфере точек назначения есть координаты
		 */
		case WAITING:
			bot->SP.pwmLeft = bot->SP.pwmRight = 0;
			if ((bot->distance > ERROR_mm) && //есть новая точка назначения
					(distR > WALL_LIMIT_mm) &&  //датчик 1 исправен
					(distL > WALL_LIMIT_mm))  	//датчик 2 исправен
				{
				bot->intLength = 0;
				bot->mode = NORMAL_DRIVE;
				} /*else {
					bot->SP.target.x = point.x;
					bot->SP.target.y = point.y;
				}*/
			retValue = 1;
			break;
		case NORMAL_DRIVE:  //обычный режим движения к точке
			if (bot->distance < ERROR_mm) {  //доехали до точки
				//проверка на наличие новой точки
				if ((pointX.bytes_avail > 0) && (pointY.bytes_avail > 0)) {
					GetStoreTargetPoint(bot, &pointX, &pointY);//предыдущая точка есть
					}
				else {//новой точки нет
					bot->SP.pwmLeft = bot->SP.pwmRight = 0;
					bot->mode = WAITING;
					}
				}
			else {//едем к точке
				float kd = 0.0;
				if (kDist != 0) {//видим препятствие
					kd = kDist * M_PI / WALL_LIMIT_mm;
					if (bot->iAngle > M_PI_2) {
						bot->iAngle = M_PI_2;
						}
					else if (bot->iAngle < -M_PI_2) {
						bot->iAngle = -M_PI_2;
						}
					else {
						bot->iAngle += kDI_step * kd;
						}
					}
				else {
					kd = 0.0;
					if (bot->iAngle > kDI_step) {
						bot->iAngle -= kDI_step;
						}
					else if (bot->iAngle < -kDI_step) {
						bot->iAngle += kDI_step;
						}
					else {
						bot->iAngle = 0.0;
						}
					}
				bot->angle = bot->bearing - bot->course - (kd + bot->iAngle);
				if (bot->angle > M_PI) {
					bot->angle -= M_TWOPI;//препятствие ровно справа формируем новую промежуточную точку
					}
				else if (bot->angle < -M_PI) {
					bot->angle += M_TWOPI;//препятствие ровно слева формируем новую промежуточную точку
					}
				//TODO добавить интегральную составляющую по углу поворота
				float vector = tanhf(bot->distance + K_I * bot->intLength);
				int32_t baseSpeed = ( int32_t )(400 * cosf(bot->angle / 2) * vector);
				int32_t control = ( int32_t )(300 * sinf(bot->angle / 2) * vector);
				//int32_t control = (int32_t)((300 * sinf(bot->angle / 2)) * vector + K_P * bot->angle);
				bot->SP.pwmLeft = baseSpeed - control - wall;
				bot->SP.pwmRight = baseSpeed + control - wall;
				}
			break;

			/* движение назад т.к. наехали бампером на препятствие, по шагам:
			 * 0 аварийный останов на 30% мощности
			 * 1 останов
			 * 2 едем назад некоторое расстояние, накапливаем путь в lenInt
			 * 3 переходим на режим поиска выхода
			 * */
		case EMERGENCY:
			switch (bot->step) {
				case 0://останов
					bot->intLength = 0;
					bot->SP.pwmLeft = bot->SP.pwmRight = 0;
					bot->step = 1;
					break;
				case 1://едем назад некоторое расстояние, накапливаем путь в lenInt
					bot->SP.pwmLeft = bot->SP.pwmRight = -150;
					if (bot->intLength > 2 * BASE_mm) {
						bot->SP.pwmLeft = bot->SP.pwmRight = 0;
						bot->step = 2;
						}
					break;
				case 2://определяем можно ли ехать длаьше
					bot->step = 0;
					if ((distL > WALL_LIMIT_mm) & (distR > WALL_LIMIT_mm)) {//все ок можно ехать дальше
						/*TODO здесь уже нужны датчики сзади а то мало ли*/
						bot->mode = WAITING;
						}
					else {																//препятствие рядом, ищем выход
						bot->mode = SEARCH;
						}
					break;
				default:
					bot->step = 0;
					break;
				}
			break;

			/* выезжаем из тупика:
			 * определяем какая стена ближе и крутимся в другую сторону*/
		case SEARCH:
			switch (bot->step) {
				case 0:							//крутимся на месте пока не найдем проход
					if (kDist == 1) {	//стена слева ближе крутимся вправо
						bot->SP.pwmLeft = 150; bot->SP.pwmRight = -150;
						}
					else {						//стена справа ближе крутимся влево
						bot->SP.pwmLeft = -150; bot->SP.pwmRight = 150;
						}
					bot->step = 1;
					break;
				case 1:
					if ((distL > WALL_LIMIT_mm * 2) & (distR > WALL_LIMIT_mm * 2)) {
						bot->SP.pwmLeft = bot->SP.pwmRight = 0;
						bot->intLength = 0;
						bot->step = 2;
						/* здесь надо бы рассчитать новую промежуточную точку назначения, но мне лень
						 * поэтому просто поедем вперед пока
						 */
						}
					break;
				case 2:
					bot->SP.pwmLeft = bot->SP.pwmRight = 200;
					if (bot->intLength > 0.4) {
						bot->SP.pwmLeft = bot->SP.pwmRight = 0;
						bot->step = 3;
						}
					break;
				case 3:
					bot->SP.pwmLeft = bot->SP.pwmRight = 0;
					bot->step = 0;
					bot->mode = WAITING;
					break;
				default:
					bot->step = 0;
					break;
				}
			break;
		default:
			bot->mode = WAITING;
			bot->SP.pwmLeft = bot->SP.pwmRight = 0;
			bot->step = 0;
			break;
		}

	/*управление скоростью и запись в ШИМ выполняется здесь*/

	return retValue;
	}
