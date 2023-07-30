/************************************************
 * Title   : Shift Register
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 10/12/2020
 * **********************************************
 * Descriptions: Intialize and Shift shift registers
 *
 *
 * Version History:
 * 1.0 - implemented using hal library
 *
 * Bugs:
 *
 ************************************************/

#ifndef BIOS_SHIFTREG_H_
#define BIOS_SHIFTREG_H_

/***************************************
 * 		Include Libraries 			   *
 **************************************/
#include "system.h"
#include "gpio.h"

/**************************************************
 * 		Enumerator							  	  *
 *************************************************/

typedef enum {

	NO_CASCADE = 0,		/* using 1 shift register */
	CASCADE_1,			/* using 2 shift register */
	CASCADE_2,			/* ... */
	CASCADE_3,
	CASCADE_4,
	CASCADE_5,

}fSR;

/**************************************************
 * 		Structure							  	  *
 *************************************************/

typedef struct{

	GPIO_TypeDef *GPIOx_sck;
	uint16_t GPIO_Pin_sck;

	GPIO_TypeDef *GPIOx_rck;
	uint16_t GPIO_Pin_rck;

	GPIO_TypeDef *GPIOx_si;
	uint16_t GPIO_Pin_si;

	fSR flag;

	byte_t cast[6];

}shiftreg_t;

/**************************************************
 * 		Function Prototype			  			  *
 *************************************************/

void SHIFTREGInit (shiftreg_t* shiftreg, fSR cascade, GPIO_TypeDef *GPIOx_sck, uint16_t GPIO_Pin_sck,
		           GPIO_TypeDef *GPIOx_rck , uint16_t GPIO_Pin_rck,GPIO_TypeDef *GPIOx_si, uint16_t GPIO_Pin_si);
void SHIFTREGShift(shiftreg_t* shiftreg);

#endif /* BIOS_SHIFTREG_H_ */
