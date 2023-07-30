/************************************************
 * Title   : BDC
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 10/12/2020
 * **********************************************
 * Descriptions: control motors or cylinders using pwm
 *
 *
 * Version History:
 * 1.0 - implemented using hal library
 *
 * Bugs:
 *
 ************************************************/

#ifndef BIOS_BDC_H_
#define BIOS_BDC_H_

/***************************************
 * 		Include Libraries 			   *
 **************************************/

#include "bios.h"

/**************************************************
 * 		Enumerator							  	  *
 *************************************************/

typedef enum{
	SHIFTREG,
	GPIO
}BCDDIRPINType;


/**************************************************
 * 		Structure							  	  *
 *************************************************/

typedef struct{

	/*Type of direction pin*/

	BCDDIRPINType Dirpintype;

	/*GPIO DIRECTION PIN*/

	GPIO_TypeDef *GPIOx_DIR1;
	uint16_t GPIO_Pin_DIR1;

	GPIO_TypeDef *GPIOx_DIR2;
	uint16_t GPIO_Pin_DIR2;

	/*Shift register direction pin*/

	byte_t* shiftreg;
	uint8_t dir1;
	uint8_t dir2;

	/*register for PWM duty cycle*/

	//__IO uint32_t* speed;

	TIM_HandleTypeDef* htim;
	uint32_t Channel;

}BDC_t;

/**************************************************
 * 		Function Prototype			  			  *
 *************************************************/

void BDCInit(BDC_t* bdc,TIM_HandleTypeDef* htimx,uint32_t Channel, BCDDIRPINType Dirpintype, ...);
void WriteBDC(BDC_t* bdc, int32_t pwm);
void StopBDC(BDC_t* bdc);

#endif /* BIOS_BDC_H_ */
