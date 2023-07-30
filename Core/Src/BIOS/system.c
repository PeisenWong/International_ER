/*
 * system.c
 *
 *  Created on: Dec 4, 2020
 *      Author: Anas Amer
 */

/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "system.h"


void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                                            /**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}




/**
* @brief ADC MSP Initialization
* This function configures the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{

  if(hadc->Instance==ADC1)
  {
    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

  }else if(hadc->Instance==ADC2 ){
	  /* Peripheral clock enable */
	  __HAL_RCC_ADC2_CLK_ENABLE();

  }else if (hadc->Instance==ADC3){

	  /* Peripheral clock enable */
	  __HAL_RCC_ADC3_CLK_ENABLE();

  }

}

/**
* @brief ADC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);

  }else if (hadc->Instance==ADC2){

	  /* Peripheral clock disable */
	  __HAL_RCC_ADC2_CLK_DISABLE();

	  /* ADC2 DMA DeInit */
	  HAL_DMA_DeInit(hadc->DMA_Handle);

  }else if(hadc->Instance==ADC3){

	  /* Peripheral clock disable */
	  __HAL_RCC_ADC3_CLK_DISABLE();

	  /* ADC2 DMA DeInit */
	  HAL_DMA_DeInit(hadc->DMA_Handle);

  }

}



static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

/**
* @brief CAN MSP Initialization
* This function configures the hardware resources used in this example
* @param hcan: CAN handle pointer
* @retval None
*/
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* Peripheral clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(hcan->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }

}

/**
* @brief CAN MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hcan: CAN handle pointer
* @retval None
*/
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan)
{
  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(hcan->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }

}

/**
* @brief I2C MSP Initialization
* This function configures the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
  else if(hi2c->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
  else if(hi2c->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspInit 0 */

  /* USER CODE END I2C3_MspInit 0 */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**I2C3 GPIO Configuration
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C3_CLK_ENABLE();
  /* USER CODE BEGIN I2C3_MspInit 1 */

  /* USER CODE END I2C3_MspInit 1 */
  }

}

/**
* @brief I2C MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);

    /* I2C1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
  else if(hi2c->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

    /* I2C2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
  else if(hi2c->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspDeInit 0 */

  /* USER CODE END I2C3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C3_CLK_DISABLE();

    /**I2C3 GPIO Configuration
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

    /* I2C3 interrupt DeInit */
    HAL_NVIC_DisableIRQ(I2C3_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C3_ER_IRQn);
  /* USER CODE BEGIN I2C3_MspDeInit 1 */

  /* USER CODE END I2C3_MspDeInit 1 */
  }

}


void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
	if(htim_pwm->Instance==TIM1)
		{
			/* Peripheral clock enable */
			__HAL_RCC_TIM1_CLK_ENABLE();
		}

		else if(htim_pwm->Instance==TIM2)
		{
			/* Peripheral clock enable */
			__HAL_RCC_TIM2_CLK_ENABLE();
		}

		else if(htim_pwm->Instance==TIM3)
		{
			/* Peripheral clock enable */
			__HAL_RCC_TIM3_CLK_ENABLE();
		}

		else if(htim_pwm->Instance==TIM4)
		{
			/* Peripheral clock enable */
			__HAL_RCC_TIM4_CLK_ENABLE();
		}

	  else if(htim_pwm->Instance==TIM5)
	  {
	    /* Peripheral clock enable */
	    __HAL_RCC_TIM5_CLK_ENABLE();
	  }

	  else if(htim_pwm->Instance==TIM6)
	  {
	    /* Peripheral clock enable */
	    __HAL_RCC_TIM6_CLK_ENABLE();
	  }


	  else if(htim_pwm->Instance==TIM7)
	   {
	     /* Peripheral clock enable */
	     __HAL_RCC_TIM7_CLK_ENABLE();
	   }

	  else if(htim_pwm->Instance==TIM8)
	   {
	     /* Peripheral clock enable */
	     __HAL_RCC_TIM8_CLK_ENABLE();
	   }


	  else if(htim_pwm->Instance==TIM9)
	  {
	    /* Peripheral clock enable */
	    __HAL_RCC_TIM9_CLK_ENABLE();
	  }

	  else if(htim_pwm->Instance==TIM10)
	  {
	    /* Peripheral clock enable */
	    __HAL_RCC_TIM10_CLK_ENABLE();
	  }


	  else if(htim_pwm->Instance==TIM11)
	  {
	    /* Peripheral clock enable */
	    __HAL_RCC_TIM11_CLK_ENABLE();
	  }

	  else if(htim_pwm->Instance==TIM12)
	  {
	    /* Peripheral clock enable */
	    __HAL_RCC_TIM12_CLK_ENABLE();
	  }

	  else if(htim_pwm->Instance==TIM13)
	  {
	    /* Peripheral clock enable */
	    __HAL_RCC_TIM13_CLK_ENABLE();
	  }

	  else if(htim_pwm->Instance==TIM14)
	  {
	    /* Peripheral clock enable */
	    __HAL_RCC_TIM14_CLK_ENABLE();
	  }

}

/**
* @brief TIM_PWM MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_pwm: TIM_PWM handle pointer
* @retval None
*/
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{
	if(htim_pwm->Instance==TIM1)
		  {
		    /* Peripheral clock disable */
		    __HAL_RCC_TIM1_CLK_DISABLE();
		  }

		if(htim_pwm->Instance==TIM2)
		  {
		    /* Peripheral clock disable */
		    __HAL_RCC_TIM2_CLK_DISABLE();
		  }

		else if(htim_pwm->Instance==TIM3)
	  {
	    /* Peripheral clock disable */
	    __HAL_RCC_TIM3_CLK_DISABLE();
	  }

		else if(htim_pwm->Instance==TIM4)
		  {
		    /* Peripheral clock disable */
		    __HAL_RCC_TIM4_CLK_DISABLE();
		  }

	  else if(htim_pwm->Instance==TIM5)
	  {
	    /* Peripheral clock disable */
	    __HAL_RCC_TIM5_CLK_DISABLE();
	  }
	  else if(htim_pwm->Instance==TIM6)
	    {
	      /* Peripheral clock disable */
	      __HAL_RCC_TIM6_CLK_DISABLE();
	    }
	  else if(htim_pwm->Instance==TIM7)
	    {
	      /* Peripheral clock disable */
	      __HAL_RCC_TIM7_CLK_DISABLE();
	    }
	  else if(htim_pwm->Instance==TIM8)
	    {
	      /* Peripheral clock disable */
	      __HAL_RCC_TIM8_CLK_DISABLE();
	    }
	  else if(htim_pwm->Instance==TIM9)
	  {
	    /* Peripheral clock disable */
	    __HAL_RCC_TIM9_CLK_DISABLE();
	  }
	  else if(htim_pwm->Instance==TIM10)
	    {
	      /* Peripheral clock disable */
	      __HAL_RCC_TIM10_CLK_DISABLE();
	    }
	  else if(htim_pwm->Instance==TIM11)
	    {
	      /* Peripheral clock disable */
	      __HAL_RCC_TIM11_CLK_DISABLE();
	    }
	  else if(htim_pwm->Instance==TIM12)
	    {
	      /* Peripheral clock disable */
	      __HAL_RCC_TIM12_CLK_DISABLE();
	    }
	  else if(htim_pwm->Instance==TIM13)
	    {
	      /* Peripheral clock disable */
	      __HAL_RCC_TIM13_CLK_DISABLE();
	    }
	  else if(htim_pwm->Instance==TIM14)
	    {
	      /* Peripheral clock disable */
	      __HAL_RCC_TIM14_CLK_DISABLE();
	    }


}


/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

	if(htim_base->Instance==TIM1)
	{
		/* Peripheral clock enable */
		__HAL_RCC_TIM1_CLK_ENABLE();
	}

	else if(htim_base->Instance==TIM2)
	{
		/* Peripheral clock enable */
		__HAL_RCC_TIM2_CLK_ENABLE();
	}

	else if(htim_base->Instance==TIM3)
	{
		/* Peripheral clock enable */
		__HAL_RCC_TIM3_CLK_ENABLE();
	}

	else if(htim_base->Instance==TIM4)
	{
		/* Peripheral clock enable */
		__HAL_RCC_TIM4_CLK_ENABLE();
	}

  else if(htim_base->Instance==TIM5)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();
  }

  else if(htim_base->Instance==TIM6)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();
  }


  else if(htim_base->Instance==TIM7)
   {
     /* Peripheral clock enable */
     __HAL_RCC_TIM7_CLK_ENABLE();
   }

  else if(htim_base->Instance==TIM8)
   {
     /* Peripheral clock enable */
     __HAL_RCC_TIM8_CLK_ENABLE();
   }


  else if(htim_base->Instance==TIM9)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM9_CLK_ENABLE();
  }

  else if(htim_base->Instance==TIM10)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM10_CLK_ENABLE();
  }


  else if(htim_base->Instance==TIM11)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM11_CLK_ENABLE();
  }

  else if(htim_base->Instance==TIM12)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM12_CLK_ENABLE();
  }

  else if(htim_base->Instance==TIM13)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM13_CLK_ENABLE();
  }

  else if(htim_base->Instance==TIM14)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM14_CLK_ENABLE();
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{

}
/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

	if(htim_base->Instance==TIM1)
	  {
	    /* Peripheral clock disable */
	    __HAL_RCC_TIM1_CLK_DISABLE();
	  }

	if(htim_base->Instance==TIM2)
	  {
	    /* Peripheral clock disable */
	    __HAL_RCC_TIM2_CLK_DISABLE();
	  }

	else if(htim_base->Instance==TIM3)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  }

	else if(htim_base->Instance==TIM4)
	  {
	    /* Peripheral clock disable */
	    __HAL_RCC_TIM4_CLK_DISABLE();
	  }

  else if(htim_base->Instance==TIM5)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();
  }
  else if(htim_base->Instance==TIM6)
    {
      /* Peripheral clock disable */
      __HAL_RCC_TIM6_CLK_DISABLE();
    }
  else if(htim_base->Instance==TIM7)
    {
      /* Peripheral clock disable */
      __HAL_RCC_TIM7_CLK_DISABLE();
    }
  else if(htim_base->Instance==TIM8)
    {
      /* Peripheral clock disable */
      __HAL_RCC_TIM8_CLK_DISABLE();
    }
  else if(htim_base->Instance==TIM9)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM9_CLK_DISABLE();
  }
  else if(htim_base->Instance==TIM10)
    {
      /* Peripheral clock disable */
      __HAL_RCC_TIM10_CLK_DISABLE();
    }
  else if(htim_base->Instance==TIM11)
    {
      /* Peripheral clock disable */
      __HAL_RCC_TIM11_CLK_DISABLE();
    }
  else if(htim_base->Instance==TIM12)
    {
      /* Peripheral clock disable */
      __HAL_RCC_TIM12_CLK_DISABLE();
    }
  else if(htim_base->Instance==TIM13)
    {
      /* Peripheral clock disable */
      __HAL_RCC_TIM13_CLK_DISABLE();
    }
  else if(htim_base->Instance==TIM14)
    {
      /* Peripheral clock disable */
      __HAL_RCC_TIM14_CLK_DISABLE();
    }

}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if(huart->Instance==USART1)
    {
	  __HAL_RCC_USART1_CLK_ENABLE();

	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  /**USART1 GPIO Configuration
	      PA9     ------> USART1_TX
	      PA10     ------> USART1_RX
	   */
	  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
  else if(huart->Instance==USART2)
    {
    /* USER CODE BEGIN USART2_MspInit 0 */

    /* USER CODE END USART2_MspInit 0 */
      /* Peripheral clock enable */
      __HAL_RCC_USART2_CLK_ENABLE();

      __HAL_RCC_GPIOD_CLK_ENABLE();
      /**USART2 GPIO Configuration
      PD5     ------> USART2_TX
      PD6     ------> USART2_RX
      */
      GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
      HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USER CODE BEGIN USART2_MspInit 1 */

    /* USER CODE END USART2_MspInit 1 */
    }

  else if(huart->Instance==USART3)
    {
    /* USER CODE BEGIN USART3_MspInit 0 */

    /* USER CODE END USART3_MspInit 0 */
      /* Peripheral clock enable */
      __HAL_RCC_USART3_CLK_ENABLE();

      __HAL_RCC_GPIOD_CLK_ENABLE();
      /**USART3 GPIO Configuration
      PD8     ------> USART3_TX
      PD9     ------> USART3_RX
      */
      GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
      HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USER CODE BEGIN USART3_MspInit 1 */

    /* USER CODE END USART3_MspInit 1 */
    }

  else if(huart->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN UART4_MspInit 1 */

  /* USER CODE END UART4_MspInit 1 */
  }

  else if(huart->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspInit 0 */

  /* USER CODE END UART5_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN UART5_MspInit 1 */

  /* USER CODE END UART5_MspInit 1 */
  }

  else if(huart->Instance==USART6)
   {
   /* USER CODE BEGIN USART6_MspInit 0 */

   /* USER CODE END USART6_MspInit 0 */
     /* Peripheral clock enable */
     __HAL_RCC_USART6_CLK_ENABLE();

     __HAL_RCC_GPIOC_CLK_ENABLE();
     /**USART6 GPIO Configuration
     PC6     ------> USART6_TX
     PC7     ------> USART6_RX
     */
     GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
     GPIO_InitStruct.Pull = GPIO_NOPULL;
     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
     GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
     HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

   /* USER CODE BEGIN USART6_MspInit 1 */

   /* USER CODE END USART6_MspInit 1 */
   }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

	if(huart->Instance==USART1)
	{
		/* USER CODE BEGIN UART4_MspDeInit 0 */

		/* USER CODE END UART4_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_USART1_CLK_DISABLE();
		/**USART1 GPIO Configuration
			      PA9     ------> USART1_TX
			      PA10     ------> USART1_RX
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

		/* USER CODE BEGIN UART4_MspDeInit 1 */

		/* USER CODE END UART4_MspDeInit 1 */
	}

	else if(huart->Instance==USART2)
	  {
	  /* USER CODE BEGIN USART2_MspDeInit 0 */

	  /* USER CODE END USART2_MspDeInit 0 */
	    /* Peripheral clock disable */
	    __HAL_RCC_USART2_CLK_DISABLE();

	    /**USART2 GPIO Configuration
	    PD5     ------> USART2_TX
	    PD6     ------> USART2_RX
	    */
	    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5|GPIO_PIN_6);

	  /* USER CODE BEGIN USART2_MspDeInit 1 */

	  /* USER CODE END USART2_MspDeInit 1 */
	  }

	else if(huart->Instance==USART3)
	  {
	  /* USER CODE BEGIN USART3_MspDeInit 0 */

	  /* USER CODE END USART3_MspDeInit 0 */
	    /* Peripheral clock disable */
	    __HAL_RCC_USART3_CLK_DISABLE();

	    /**USART3 GPIO Configuration
	    PD8     ------> USART3_TX
	    PD9     ------> USART3_RX
	    */
	    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8|GPIO_PIN_9);

	  /* USER CODE BEGIN USART3_MspDeInit 1 */

	  /* USER CODE END USART3_MspDeInit 1 */
	  }

	else if(huart->Instance==UART4)
	{
		/* USER CODE BEGIN UART4_MspDeInit 0 */

		/* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
  }

  else if(huart->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspDeInit 0 */

  /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();

    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

  /* USER CODE BEGIN UART5_MspDeInit 1 */

  /* USER CODE END UART5_MspDeInit 1 */
  }

  else if(huart->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }

}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_encoder->Instance==TIM1)
  {

    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();


    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  }else if(htim_encoder->Instance==TIM2){

	  /* Peripheral clock enable */
	  __HAL_RCC_TIM2_CLK_ENABLE();

	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  GPIO_InitStruct.Pin = GPIO_PIN_15;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = GPIO_PIN_3;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  }else if(htim_encoder->Instance==TIM3){

	  /* Peripheral clock enable */
	  __HAL_RCC_TIM3_CLK_ENABLE();

	  __HAL_RCC_GPIOA_CLK_ENABLE();

	  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  }else if(htim_encoder->Instance==TIM4){

	  /* Peripheral clock enable */
	  __HAL_RCC_TIM4_CLK_ENABLE();

	  __HAL_RCC_GPIOD_CLK_ENABLE();

	  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  }else if(htim_encoder->Instance==TIM5){

	  /* Peripheral clock enable */
	  __HAL_RCC_TIM5_CLK_ENABLE();

	  __HAL_RCC_GPIOA_CLK_ENABLE();

	  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  }else if(htim_encoder->Instance==TIM8){

	  /* Peripheral clock enable */
	  __HAL_RCC_TIM8_CLK_ENABLE();

	  __HAL_RCC_GPIOC_CLK_ENABLE();

	  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }

}

void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* htim_encoder)
{
  if(htim_encoder->Instance==TIM1)
  {

    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_9|GPIO_PIN_11);



  } else if(htim_encoder->Instance==TIM2){

	  /* Peripheral clock disable */
	  __HAL_RCC_TIM2_CLK_DISABLE();



	  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);
	  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3);

  } else if(htim_encoder->Instance==TIM3){

	  /* Peripheral clock disable */
	  __HAL_RCC_TIM3_CLK_DISABLE();


	  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6|GPIO_PIN_7);

  }else if(htim_encoder->Instance==TIM4){

	  /* Peripheral clock disable */
	  __HAL_RCC_TIM4_CLK_DISABLE();


	  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12|GPIO_PIN_13);

  }else if(htim_encoder->Instance==TIM5){

	  /* Peripheral clock disable */
	  __HAL_RCC_TIM5_CLK_DISABLE();


	  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);

  }else if(htim_encoder->Instance==TIM8){

	  /* Peripheral clock disable */
	  __HAL_RCC_TIM8_CLK_DISABLE();


	  HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI1)
  {

    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  }else if (hspi->Instance==SPI2){

	  /* Peripheral clock enable */
	  __HAL_RCC_SPI2_CLK_ENABLE();

	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  /**SPI2 GPIO Configuration
	      PB13     ------> SPI2_SCK
	      PB14     ------> SPI2_MISO
	      PB15     ------> SPI2_MOSI
	   */
	  GPIO_InitStruct.Pin = GPIO_PIN_13;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  }else {

	  /* Peripheral clock enable */
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  /**SPI3 GPIO Configuration
	 	      PB3     ------> SPI3_SCK
	 	      PB4     ------> SPI3_MISO
	 	      PC12     ------> SPI3_MOSI
	   */
	  GPIO_InitStruct.Pin = GPIO_PIN_3;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = GPIO_PIN_4;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = GPIO_PIN_12;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


  }

}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
	if(hspi->Instance==SPI1)
	{
		/* Peripheral clock disable */
		__HAL_RCC_SPI1_CLK_DISABLE();

		/**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

	}else if (hspi->Instance==SPI1){

		/* Peripheral clock disable */
		__HAL_RCC_SPI2_CLK_DISABLE();

		/**SPI2 GPIO Configuration
		      PB13     ------> SPI2_SCK
		      PB14     ------> SPI2_MISO
		      PB15     ------> SPI2_MOSI
		 */
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

	}else {
		/* Peripheral clock disable */
		__HAL_RCC_SPI3_CLK_DISABLE();

		/**SPI3 GPIO Configuration
			 	      PB3     ------> SPI3_SCK
			 	      PB4     ------> SPI3_MISO
			 	      PC12     ------> SPI3_MOSI
		 */
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|GPIO_PIN_4);
		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

	}
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
