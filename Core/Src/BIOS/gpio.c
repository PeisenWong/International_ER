/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "gpio.h"

/************************************************/
/*		 	 	Functions		       		  	*/
/************************************************/

/*
 * Function Name		: GPIOClockSet
 * Function Description : This function is called to enable all the Peripheral clock of GPIOs.
 * Function Remarks		: Do not called this function when not using all the GPIO Groups for power saving purpose.
 * Function Arguments	: None
 * Function Return		: None
 * Function Example		: GPIOClockSet();
 */

void GPIOClockSet (void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
}

/*
 * Function Name		: GPIOPinsInit
 * Function Description : This function is called to configure the GPIO Pins of STM32F4 Discovery Board.
 * Function Remarks		:
 * Function Arguments	: GPIOx							where x can be A to I.
 * 						  GPIO_Pin						The desired pin that you want to configure in group GPIOx. This parameter
 * 						  								can be combination of GPIO_Pin_x where x can be (0..15)
 * 						  GPIO_Mode						the value can be one of the following value:
 * 						  								GPIO_MODE_INPUT
 * 						  								GPIO_MODE_OUTPUT_PP
 * 						  								GPIO_MODE_OUTPUT_OD
 * 						  								GPIO_MODE_AF_PP
 * 						  								GPIO_MODE_AF_OD
 * 						  								GPIO_MODE_ANALOG
 * 						  								GPIO_MODE_IT_RISING
 * 						  								GPIO_MODE_IT_FALLING
 * 						  								GPIO_MODE_IT_RISING_FALLING
 * 						  								GPIO_MODE_EVT_RISING
 * 						  								GPIO_MODE_EVT_FALLING
 * 						  								GPIO_MODE_EVT_RISING_FALLING
 *						  GPIO_Speed					Specifies the speed for the selected pins:
 *						  								GPIO_SPEED_FREQ_LOW       2MHz
 *														GPIO_SPEED_FREQ_MEDIUM	  12,5 MHz to 50 MHz
 * 														GPIO_SPEED_FREQ_HIGH      25 MHz to 100 MHz
 * 														GPIO_SPEED_FREQ_VERY_HIGH 50 MHz to 200 MHz
 * 						  GPIO_PuPd						operating Pull-up/Pull down for the selected pins
 * 						  								GPIO_NOPULL
 *														GPIO_PULLUP
 *														GPIO_PULLDOWN
 * Function Return		: None
 * Function Example		: GPIOPinsInit(GPIOA, GPIO_Pin_2, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_Speed_50MHz, GPIO_PuPd_UP);
 */

void GPIOPinsInit (GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin,uint32_t Mode, uint32_t GPIO_Speed,  uint32_t GPIO_PuPd)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	switch((uint32_t)GPIOx){

	case GPIOA_BASE:	  __HAL_RCC_GPIOA_CLK_ENABLE();
	break;

	case GPIOB_BASE:	__HAL_RCC_GPIOB_CLK_ENABLE();
	break;

	case GPIOC_BASE:	__HAL_RCC_GPIOC_CLK_ENABLE();
	break;

	case GPIOD_BASE:	__HAL_RCC_GPIOD_CLK_ENABLE();
	break;

	case GPIOE_BASE:	__HAL_RCC_GPIOE_CLK_ENABLE();
	break;

	case GPIOF_BASE:	__HAL_RCC_GPIOF_CLK_ENABLE();
	break;

	case GPIOG_BASE:	__HAL_RCC_GPIOG_CLK_ENABLE();
	break;

	case GPIOH_BASE:	__HAL_RCC_GPIOH_CLK_ENABLE();
	break;

	case GPIOI_BASE:	__HAL_RCC_GPIOI_CLK_ENABLE();
	break;

	default: break;
	}


	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = Mode;
	GPIO_InitStruct.Pull = GPIO_PuPd;
	GPIO_InitStruct.Speed = GPIO_Speed;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

}




