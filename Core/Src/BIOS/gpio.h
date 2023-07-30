/************************************************
 * Title   : GPIO
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 10/12/2020
 * **********************************************
 * Descriptions: initialize pins for usage
 *
 *
 * Version History:
 * 1.0 - implemented using hal library
 *
 * Bugs:
 *
 ************************************************/
#ifndef BIOS_GPIO_H_
#define BIOS_GPIO_H_

/***************************************
 * 		Include Libraries 			   *
 **************************************/
#include "system.h"

/***************************************
 * 		Defines			 			   *
 **************************************/



#define GPIOA_OUT			((word_t*) &GPIOA->ODR)
#define GPIOB_OUT			((word_t*) &GPIOB->ODR)
#define GPIOC_OUT			((word_t*) &GPIOC->ODR)
#define GPIOD_OUT			((word_t*) &GPIOD->ODR)
#define GPIOE_OUT			((word_t*) &GPIOE->ODR)
#define GPIOF_OUT			((word_t*) &GPIOF->ODR)
#define GPIOG_OUT			((word_t*) &GPIOG->ODR)

#define GPIOA_IN			((word_t*) &GPIOA->IDR)
#define GPIOB_IN			((word_t*) &GPIOB->IDR)
#define GPIOC_IN			((word_t*) &GPIOC->IDR)
#define GPIOD_IN			((word_t*) &GPIOD->IDR)
#define GPIOE_IN			((word_t*) &GPIOE->IDR)
#define GPIOF_IN			((word_t*) &GPIOF->IDR)
#define GPIOG_IN			((word_t*) &GPIOG->IDR)

/**************************************************
 * 		Function Prototype			  			  *
 *************************************************/

void GPIOClockSet (void);
void GPIOPinsInit (GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin,uint32_t Mode, uint32_t GPIO_Speed, uint32_t GPIO_PuPd);


#endif /* BIOS_GPIO_H_ */
