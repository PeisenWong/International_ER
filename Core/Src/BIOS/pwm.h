/************************************************
 * Title   : PWM
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 10/12/2020
 * **********************************************
 * Descriptions: Initialize PWM Timers
 *
 *
 * Version History:
 * 1.0 - implemented using hal library
 *
 * Bugs:
 *
 ************************************************/

#ifndef BIOS_PWM_H_
#define BIOS_PWM_H_

/***************************************
 * 		Include Libraries 			   *
 **************************************/

#include "bios.h"


/**************************************************
 * 		Enumerator							  	  *
 *************************************************/

typedef enum{
	PWM_CHANNEL_1,
	PWM_CHANNEL_2,
	PWM_CHANNEL_3,
	PWM_CHANNEL_4
}PWMChannel;

/**************************************************
 * 		Function Prototype						 *
 *************************************************/


void PWMTimeBaseInit(TIM_HandleTypeDef* htimx, uint32_t Period, uint32_t Prescaler);
void PWMChannelConfig(TIM_HandleTypeDef* htimx,uint32_t  Channel , GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_x);

#endif /* BIOS_PWM_H_ */
