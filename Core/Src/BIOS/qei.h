/************************************************
 * Title   : QEI
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 10/12/2020
 * **********************************************
 * Descriptions: Read Data from encoders using QEI
 *
 *
 * Version History:
 * 1.0 - implemented using hal library
 *
 * Bugs:
 *
 ************************************************/

#ifndef BIOS_QEI_H_
#define BIOS_QEI_H_

/***************************************
 * 		Include Libraries 			   *
 **************************************/

#include "bios.h"



/**************************************************
 * 		STRUCTURE DEFINES					  	  *
 *************************************************/

/**************************************************
 * 		DEFINES								  	  *
 *************************************************/


/**************************************************
 * 		Structure							  	  *
 *************************************************/

typedef struct{

	union{
		int32_t count;
		struct{
			uint16_t poscnt;
			int16_t  signbit;
		};
	};

}qei_TypeDef;

typedef struct{
	union{
		signed long LONG;
		struct{
			char BYTE1;
			char BYTE2;
			char BYTE3;
			signed char BYTE4;
		};
	}l[6];
}enc_TypeDef;

/**************************************************
 * 		Enumerator							  	  *
 *************************************************/
typedef enum {
	QEI1 = 1,
	QEI2 = 2,
	QEI3 = 3,
	QEI4 = 4,
	QEI5 = 5,
	QEI6 = 6
}QEI_TypeDef;

typedef enum{
	QEI_No_Swap = 0,
	QEI_Swap
}QEI_Direction_TypeDef;

/**************************************************
 * 		Extern	variables					  	  *
 *************************************************/
extern qei_TypeDef BIOS_QEI1;
extern qei_TypeDef BIOS_QEI2;
extern qei_TypeDef BIOS_QEI3;
extern qei_TypeDef BIOS_QEI4;
extern qei_TypeDef BIOS_QEI5;
extern qei_TypeDef BIOS_QEI6;
extern enc_TypeDef qei;

/**************************************************
 * 		Function Prototype			  			  *
 *************************************************/

void QEIInit(TIM_HandleTypeDef* htimx);
void QEIPrintValueStore(UART_HandleTypeDef* huartx);
void QEIDelay(uint8_t value);
void QEIWrite(QEI_TypeDef QEIx, int32_t value);
void QEIReset(QEI_TypeDef QEIx);
void QEISwap(QEI_TypeDef QEIx, QEI_Direction_TypeDef swap);
int32_t QEIRead(QEI_TypeDef QEIx);
uint32_t QEIReadRaw(QEI_TypeDef QEIx);

#endif /* BIOS_QEI_H_ */
