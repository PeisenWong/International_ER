/************************************************
 * Title   : System
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 10/12/2020
 * **********************************************
 * Descriptions: System functions and pins
 * assigned to peripherals
 *
 *
 * Version History:
 * 1.0 - implemented using hal library
 *
 * Bugs:
 *
 ************************************************/
#ifndef INC_SYSTEM_H_
#define INC_SYSTEM_H_

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include "priorities.h"
void Error_Handler(void);
void SystemClock_Config(void);
char data[100];
/**************************************************
 * 					DEFINES					  	  *
 *************************************************/
#define var(name) #name

#define Bit0 	((uint8_t)0b00000001)
#define Bit1	((uint8_t)0b00000010)
#define Bit2	((uint8_t)0b00000100)
#define Bit3	((uint8_t)0b00001000)
#define Bit4	((uint8_t)0b00010000)
#define Bit5	((uint8_t)0b00100000)
#define Bit6	((uint8_t)0b01000000)
#define Bit7	((uint8_t)0b10000000)

/**************************************************
 * 		Structure							  	  *
 *************************************************/
typedef struct{
	union{
		uint8_t Byte;
		struct{
			unsigned bit0		:	1;
			unsigned bit1		:	1;
			unsigned bit2		:	1;
			unsigned bit3		:	1;
			unsigned bit4		:	1;
			unsigned bit5		:	1;
			unsigned bit6		:	1;
			unsigned bit7		:	1;
		};
	};
}byte_t;

typedef struct{
	union{
		uint16_t Halfword;
		struct{
			unsigned bit0		:	1;
			unsigned bit1		:	1;
			unsigned bit2		:	1;
			unsigned bit3		:	1;
			unsigned bit4		:	1;
			unsigned bit5		:	1;
			unsigned bit6		:	1;
			unsigned bit7		:	1;
			unsigned bit8		:	1;
			unsigned bit9		:	1;
			unsigned bit10		:	1;
			unsigned bit11		:	1;
			unsigned bit12		:	1;
			unsigned bit13		:	1;
			unsigned bit14		:	1;
			unsigned bit15		:	1;
		};
	};
}halfword_t;

typedef struct{
	union{
		uint32_t Word;
		struct{
			unsigned bit0		:	1;
			unsigned bit1		:	1;
			unsigned bit2		:	1;
			unsigned bit3		:	1;
			unsigned bit4		:	1;
			unsigned bit5		:	1;
			unsigned bit6		:	1;
			unsigned bit7		:	1;
			unsigned bit8		:	1;
			unsigned bit9		:	1;
			unsigned bit10		:	1;
			unsigned bit11		:	1;
			unsigned bit12		:	1;
			unsigned bit13		:	1;
			unsigned bit14		:	1;
			unsigned bit15		:	1;
			unsigned bit16		:	1;
			unsigned bit17		:	1;
			unsigned bit18		:	1;
			unsigned bit19		:	1;
			unsigned bit20		:	1;
			unsigned bit21		:	1;
			unsigned bit22		:	1;
			unsigned bit23		:	1;
			unsigned bit24		:	1;
			unsigned bit25		:	1;
			unsigned bit26		:	1;
			unsigned bit27		:	1;
			unsigned bit28		:	1;
			unsigned bit29		:	1;
			unsigned bit30		:	1;
			unsigned bit31		:	1;
		};
	};
}word_t;



#define QEI1_PLUSEA_PIN		GPIOE , GPIO_PIN_9
#define QEI1_PLUSEB_PIN		GPIOE , GPIO_PIN_11

#define QEI2_PLUSEA_PIN		GPIOA , GPIO_PIN_5
#define QEI2_PLUSEB_PIN		GPIOB , GPIO_PIN_3

#define QEI3_PLUSEA_PIN		GPIOA , GPIO_PIN_6
#define QEI3_PLUSEB_PIN		GPIOA , GPIO_PIN_7

#define QEI4_PLUSEA_PIN		GPIOD , GPIO_PIN_12
#define QEI4_PLUSEB_PIN		GPIOD , GPIO_PIN_13

#define QEI5_PLUSEA_PIN		GPIOA , GPIO_PIN_0
#define QEI5_PLUSEB_PIN		GPIOA , GPIO_PIN_1

#define QEI6_PLUSEA_PIN		GPIOC , GPIO_PIN_6
#define QEI6_PLUSEB_PIN		GPIOC , GPIO_PIN_7

#endif /* INC_SYSTEM_H_ */
