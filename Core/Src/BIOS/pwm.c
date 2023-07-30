/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "pwm.h"

/************************************************/
/*		 	 	Functions		       		  	*/
/************************************************/

/*
 * Function Name		: PWMTimeBaseInit
 * Function Description : This function is called to initialize time base of timer for PWM .
 * Function Remarks		: User can set the frequency of PWM by using the formula:
 *
 * 						  			frequency in Hz = (Timer_Clock)/((period)*(prescaler))
 *
 * Function Arguments	: htimx 		Pointer to timer handle
 * 						  Period		Period value (1 to 65535)
 * 						  Prescaler 	Prescaler value to divide TIM clock (1 to 65535)
 * Function Return		: None
 * Function Example		: PWMTimeBaseInit(&htim3, 20000, 84);
 */
void PWMTimeBaseInit(TIM_HandleTypeDef* htimx, uint32_t Period, uint32_t Prescaler)
{

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};


	if(htimx == &htim1){
		htimx->Instance = TIM1;
	}
	else if(htimx == &htim2)	{
		htimx->Instance = TIM2;
	}
	else if(htimx == &htim3)	{
		htimx->Instance = TIM3;
	}
	else if(htimx == &htim4)	{
		htimx->Instance = TIM4;
	}
	else if(htimx == &htim5)	{
		htimx->Instance = TIM5;
	}
	else if(htimx == &htim8)	{
		htimx->Instance = TIM8;
	}
	else if(htimx == &htim9)	{
		htimx->Instance = TIM9;
	}
	else if(htimx == &htim10)	{
		htimx->Instance = TIM10;
	}
	else if(htimx == &htim11)	{
		htimx->Instance = TIM11;
	}
	else if(htimx == &htim12)	{
		htimx->Instance = TIM12;
	}
	else if(htimx == &htim13)	{
		htimx->Instance = TIM13;
		}
	else if(htimx == &htim14)	{
		htimx->Instance = TIM14;
		}

	htimx->Init.Prescaler = Prescaler - 1;
	htimx->Init.CounterMode = TIM_COUNTERMODE_UP;
	htimx->Init.Period = Period - 1;
	htimx->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htimx->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	 if (HAL_TIM_Base_Init(htimx) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(htimx, &sClockSourceConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	if (HAL_TIM_PWM_Init(htimx) != HAL_OK)
		{
			Error_Handler();
		}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(htimx, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}


    HAL_TIM_Base_Start(htimx);

}


/*
 * Function Name		: PWMChannelConfig
 * Function Description : This function is called to configure the channel of PWM.
 * Function Remarks		: None
 * Function Arguments	: htimx 		pointer to timer handle
 * 						  Channel		Channel of PWM:
 * 						  				TIM_CHANNEL_1
 * 						  				TIM_CHANNEL_2
 * 						  				TIM_CHANNEL_3
 * 						  				TIM_CHANNEL_4
 * 						  GPIOx			GPIOx group of PWM pin(x = A,B,C,D or E)
 * 						  GPIO_Pin_x	GPIO_Pin_x of PWM pin(x = 0,1,2,...or 15)
 * Function Return		: None
 * Function Example		: PWMChannelConfig(&htim3, TIM_CHANNEL_3, TIM3_CHANNEL3_PIN);
 */

void PWMChannelConfig(TIM_HandleTypeDef* htimx, uint32_t Channel , GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_x){

	TIM_OC_InitTypeDef sConfigOC = {0};
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	if (HAL_TIM_PWM_ConfigChannel(htimx, &sConfigOC, Channel) != HAL_OK)
	{
		Error_Handler();
	}

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

	    GPIO_InitStruct.Pin = GPIO_Pin_x;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_PULLUP;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		if(htimx == &htim1){
			GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
		}
		else if(htimx == &htim2)	{
			GPIO_InitStruct.Alternate = GPIO_AF1_TIM2 ;
		}
		else if(htimx == &htim3)	{
			GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
		}
		else if(htimx == &htim4)	{
			GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
		}
		else if(htimx == &htim5)	{
			GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
		}
		else if(htimx == &htim8)	{
			GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
		}
		else if(htimx == &htim9)	{
			GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
		}
		else if(htimx == &htim10)	{
			GPIO_InitStruct.Alternate = GPIO_AF3_TIM10;
		}
		else if(htimx == &htim11)	{
			GPIO_InitStruct.Alternate = GPIO_AF3_TIM11;
		}
		else if(htimx == &htim12)	{
			GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
		}
		else if(htimx == &htim13)	{
			GPIO_InitStruct.Alternate = GPIO_AF9_TIM13;
			}
		else if(htimx == &htim14)	{
			GPIO_InitStruct.Alternate = GPIO_AF9_TIM14;
			}

	    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);


	    HAL_TIM_PWM_Start(htimx,Channel);

}
