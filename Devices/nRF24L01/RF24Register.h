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

 * 	nRF24Register.h
 *  Created on: 23 feb 2022
 */

//=========================================  COMMANDS  ============================================
//команда начинается с отправки командного байта, и потом с отправки байта(ов) данных
//любая команда всегда возвращает статус радиомодуля, можно использовать для проверки подключенного радиомодуля
enum Command {
	R_REGISTER		= 0x00,	//use (R_REGISTER | REG) Read command and status registers. REG - see Register Map Address
	W_REGISTER		= 0x20,	//use (W_REGISTER | REG) Write command and status registers. REG - see Register Map Address
	R_RX_PAYLOAD	= 0x61,	//read RX-payload: 1 – 32 bytes. A read operation always starts at byte 0.
	W_TX_PAYLOAD	= 0xA0, //write TX-payload: 1 – 32 bytes. A write operation always starts at byte 0.
	FLUSH_TX			= 0xE1,	//Сбросить буфер передатчика, работает только в режиме передатчика
	FLUSH_RX			= 0xE2,	//Сбросить буфер приёмника, работает только в режиме приемника
	REUSE_TX_PL		= 0xE3,	//повторно отправить последний пакет
	R_RX_PL_WID		= 0x60,	//Считать размер буфера принятого пакета приёмника. Если больше 32 - ошибка, нужно очистить командой FLUSH_RX
	W_ACK_PAYLOAD	= 0xA8,	// + p Записать автоответ для канала p. При этом бит EN_ACK_PAY в регистре FEATURE должен быть установлен
													//команда для канала - W_ACK_PAYLOAD & number (10101PPP), ppp - number 000..101 разрешенные
	W_TX_PAYLOAD_NOACK = 0xB0,	//Записать в передатчик данные, которые отправляются без ожидания автоответа
	RF24_NOP		= 0xFF,	//Нет операции. Может быть использовано для чтения регистра статуса или для поверки подключенного модуля
	ACTIVATE		= 0x50	//команда может понадобиться для китайского клона радиомодуля
};
//=========================================  REGISTERS  ===========================================
enum Registers {
	CONFIG			= 0x00,	//RW настройка прерываний, контрольной суммы, питания и статуса TX/RX
	EN_AA				= 0x01,	//RW включает автоподтверждение приема на выбранном канале RX 1 = ON; 0 = OFF
	EN_RXADDR		= 0x02,	//RW Выбор каналов приёмника, включение и отключение канала Rx 1 = ON; 0 = OFF
	SETUP_AW		= 0x03,	//RW Задаёт длину поля адреса 3-5 byte
	SETUP_RETR 	= 0x04,	//RW Настройка повторной отправки пакетов при ошибках
	RF_CH				= 0x05,	//RW Регистр задаёт номер радиоканала - частоту несущей с шагом 1Мгц.
	RF_SETUP		= 0x06,	//RW настройка радиоканала: скорость, мощность, коэффициент усиления
	STATUS			= 0x07,	//RW Регистр статуса (регистры прерывания, состояние буферов приема/передачи и тд
	OBSERVE_TX	= 0x08,	//RW
	RPD					= 0x09,	//RW
	RX_ADDR_P0	= 0x0A,	//RW 3-5 байт (начиная с младшего байта). Адрес канала 0 приёмника
	RX_ADDR_P1	= 0x0B,	//RW 3-5 байт (начиная с младшего байта). Адрес канала 1 приёмника
	RX_ADDR_P2	= 0x0C,	//RW 3-5 байт (начиная с младшего байта). Адрес канала 2 приёмника
	RX_ADDR_P3	= 0x0D,	//RW 3-5 байт (начиная с младшего байта). Адрес канала 3 приёмника
	RX_ADDR_P4	= 0x0E,	//RW 3-5 байт (начиная с младшего байта). Адрес канала 4 приёмника
	RX_ADDR_P5	= 0x0F,	//RW 3-5 байт (начиная с младшего байта). Адрес канала 5 приёмника
	TX_ADDR			= 0x10, //RW 3-5 байт (начиная с младшего байта). Адрес канала передатчика
	RX_PW_P0		= 0x11,	//RW допустимое число байт в канале 0: от 1 до 32. 0 - канал не используется
	RX_PW_P1		= 0x12,	//RW допустимое число байт в канале 0: от 1 до 32. 0 - канал не используется
	RX_PW_P2		= 0x13,	//RW допустимое число байт в канале 0: от 1 до 32. 0 - канал не используется
	RX_PW_P3		= 0x14,	//RW допустимое число байт в канале 0: от 1 до 32. 0 - канал не используется
	RX_PW_P4		= 0x15,	//RW допустимое число байт в канале 0: от 1 до 32. 0 - канал не используется
	RX_PW_P5		= 0x16,	//RW допустимое число байт в канале 0: от 1 до 32. 0 - канал не используется
	FIFO_STATUS = 0x17,	//RO Состояние очередей FIFO приёмника и передатчика
	DYNPD				= 0x1C,	//RW Выбор каналов приёмника для которых используется произвольная длина пакетов
	FEATURE			= 0x1D	//RW Регистр опций
};
//========================================= TYPEDEF's for CONFIG  ===================================
typedef enum Enable {
	OFF = 0x00,
	ON 	= 0x01
} Enable_t;

typedef enum ModePTT {
	modeTX 	= 0x00,
	modeRX	= 0x01
} ModePTT_t;

typedef enum nRF_CRC {
	crc_off	= 0x00,//
	crc_1b 	= 0x08,
	crc_2b 	= 0x0C
} nRF_CRC_t;

typedef struct CONFIG_SET {
	Enable_t RX_DR_IRQ;	//6 bit RW включает прерывание по RX_DR (получение пакета) 1 - OFF | 0 - ON
	Enable_t TX_DS_IRQ;	//5 bit RW включает прерывание по TX_DS (отправка пакета или получение подтверждения о доставке) 1-OFF | 0-ON
	Enable_t MAX_TX_IRQ;//4 bit RW включает прерывание по MAX_RT (превышение числа повторных попыток отправки) 1-OFF | 0-ON
	nRF_CRC_t crc;		//3&2 bits RW Размер поля CRC: 0 - 1 байт; 1 - 2 байта
	Enable_t power;		//1 bit RW Включение питания 1 - Power Up; 0 - Power Down, помни про временные задержки на включение
	ModePTT_t mode;		//0 bit RW Выбор режима: 0 - TX (передатчик); 1 - RX (приёмник)
} CONFIG_SET_t;
//========================================= TYPEDEF's for ADDR WIDTH  =================================
typedef enum AddrWidth {
	addrWidth_3b = 0x01,
	addrWidth_4b = 0x02,
	addrWidth_5b = 0x03
} AddrWidth_t;
//========================================= TYPEDEF's for RF SETUP  ===================================
typedef enum RF_Speed {
	nRF_1Mbps		= 0x00,	//RF_DR_LOW = 0 & RF_DR_HIGH = 0
	nRF_2Mbps		= 0x08,	//RF_DR_LOW = 0 & RF_DR_HIGH = 1
	nRF_250kbps	= 0x20	//RF_DR_LOW = 1 & RF_DR_HIGH = 0
} RF_Speed_t;

typedef enum RF_Power {
	RF_PWR_m18db	= 0x00,	//2бита мощность: 00 -18dBm;
	RF_PWR_m12db	= 0x02,	//2бита мощность: 01 -16dBm;
	RF_PWR_m6db		= 0x04,	//2бита мощность: 10 -6dBm;
	RF_PWR_0db		= 0x06	//2бита мощность: 11  0dBm
} RF_Power_t;

typedef struct RF_SET {
	Enable_t CONT_WAVE;
	Enable_t PLL_LOCK;
	RF_Speed_t speed;
	RF_Power_t radiance;
} RF_SET_t;
//========================================= TYPEDEF's for STATUS  =====================================
typedef enum Line {
	line0 = 0x00,
	line1 = 0x01,
	line2 = 0x02,
	line3 = 0x03,
	line4 = 0x04,
	line5 = 0x05,
	empty = 0x07
} Line_t;

enum Status_Bitmask {
	RX_DR 	= 0x40,
	TX_DS 	= 0x20,
	MAX_RT 	= 0x10,
	TX_FULL = 0x01
};
//========================================= TYPEDEF's for FIFO  =====================================
enum FIFO_Bitmask {
	TX_FIFO_REUSE	= 0x40, //RO индикация повторного ипользования последнего пакета для повторной отправки.
	TX_FIFO_FULL	= 0x20, //RO Флаг переполнения FIFO очереди передатчика. 1 - full, 0 - empty
	TX_FIFO_EMPTY	= 0x10, //RO Флаг освобождения FIFO очереди передатчика. 1 - empty, 0 - data in buffer
	RX_FIFO_FULL	= 0x02, //RO Флаг переполнения FIFO очереди приёмника. 1 - full, 0 - empty
	RX_FIFO_EMPTY	= 0x01 	//RO Флаг освобождения FIFO очереди приёмника. 1 - empty, 0 - data in buffer
};
//========================================= TYPEDEF's for FEATURE  =====================================
typedef struct FEATURE_Set {
	Enable_t EN_DPL; 	//RW Включает поддержку приёма и передачи пакетов произвольной длины
	Enable_t EN_ACK_PAY;//RW Разрешает передачу данных с пакетами подтверждения приёма
	Enable_t EN_DYN_ACK;//RW Разрешает использование W_TX_PAYLOAD_NOACK (отправка пакета без подтверждения)
} FEATURE_Set_t;
//=======================================================================================================
