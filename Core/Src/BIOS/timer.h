/************************************************
 * Title   : Timer
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 10/12/2020
 * **********************************************
 * Descriptions: Initialize timers
 *
 *
 * Version History:
 * 1.0 - implemented using hal library
 *
 * Bugs:
 *
 ************************************************/
#ifndef BIOS_TIMER_H_
#define BIOS_TIMER_H_

/***************************************
 * 		Include Libraries 			   *
 **************************************/
#include "system.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;
/**************************************************
 * 		Function Prototype			  			  *
 *************************************************/
void TIMxInit(TIM_HandleTypeDef* htimx, uint16_t period, uint16_t prescaler);
void BIOSTIMx_Delayus(TIM_HandleTypeDef* htimx, uint16_t Delayus);


#endif /* BIOS_TIMER_H_ */
