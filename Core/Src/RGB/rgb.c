#include "rgb.h"
static void softwarePwm(RGB_t* rgb, int color, uint32_t dutyCycle);

void RGBInit(RGB_t* rgb, GPIO_TypeDef* gpioR, uint16_t gpioRPin,\
						 GPIO_TypeDef* gpioG, uint16_t gpioGPin,\
						 GPIO_TypeDef* gpioB, uint16_t gpioBPin,\
						 TIM_HandleTypeDef* htimx, uint16_t frequency){

						rgb->GPIO[0] = gpioR;

	rgb->GPIO[2] = gpioG;					rgb->GPIO[1] = gpioB;

	rgb->GPIOPin[0] = gpioRPin;
	rgb->GPIOPin[1] = gpioBPin;
	rgb->GPIOPin[2] = gpioGPin;
	rgb->pwmtick = HAL_GetTick();
	rgb->fadeTick = HAL_GetTick();
	rgb->Period = 0.000001/frequency;//1us time base
	rgb->Hpulse = rgb->Period;
	rgb->Lpulse = 0;
	rgb->fadeSpeed = 20;//ms

	rgb->htim = htimx;
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	uint16_t prescaler;
	if(htimx == &htim1){
		htimx->Instance = TIM1;
		prescaler = 168;
	}
	else if(htimx == &htim2)	{
		htimx->Instance = TIM2;
		prescaler = 84;
	}
	else if(htimx == &htim3)	{
		htimx->Instance = TIM3;
		prescaler = 84;
	}
	else if(htimx == &htim4)	{
		htimx->Instance = TIM4;
		prescaler = 84;
	}
	else if(htimx == &htim5)	{
		htimx->Instance = TIM5;
		prescaler = 84;
	}
	else if(htimx == &htim6)	{
		htimx->Instance = TIM6;
		prescaler = 84;
	}
	else if(htimx == &htim7)	{
		htimx->Instance = TIM7;
		prescaler = 84;
	}
	else if(htimx == &htim8)	{
		htimx->Instance = TIM8;
		prescaler = 168;
	}
	else if(htimx == &htim9)	{
		htimx->Instance = TIM9;
		prescaler = 168;
	}
	else if(htimx == &htim10)	{
		htimx->Instance = TIM10;
		prescaler = 168;
	}
	else if(htimx == &htim11)	{
		htimx->Instance = TIM11;
		prescaler = 168;
	}
	else if(htimx == &htim12)	{
		htimx->Instance = TIM12;
		prescaler = 84;
	}
	else if(htimx == &htim13)	{
		htimx->Instance = TIM13;
		prescaler = 84;
	}
	else if(htimx == &htim14)	{
		htimx->Instance = TIM14;
		prescaler = 84;
	}


	htimx->Init.Prescaler = prescaler-1;
	htimx->Init.CounterMode = TIM_COUNTERMODE_UP;
	htimx->Init.Period = 65535;
	htimx->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(htimx) != HAL_OK)
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

static void softwarePwm(RGB_t* rgb, int color, uint32_t dutyCycle){
	uint32_t curTick = __HAL_TIM_GET_COUNTER(rgb->htim) -rgb->pwmtick;
	//dutyCycle 0-100 in percentage
	if(curTick >= rgb->Period){
		curTick = 0;
		__HAL_TIM_SET_COUNTER(rgb->htim,0);
	}

	uint32_t dutyTick = dutyCycle/100 * rgb->Period;

	if(curTick < dutyTick){
		HAL_GPIO_WritePin(rgb->GPIO[color], rgb->GPIOPin[color], GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(rgb->GPIO[color], rgb->GPIOPin[color], GPIO_PIN_RESET);
	}
}

void RGB_Breath(RGB_t* rgb){
	static uint8_t state = -1;
	if(state == -1){
		__HAL_TIM_SET_COUNTER(rgb->htim,0);
		state = 0;
		rgb->fadeTick = HAL_GetTick();
	}
	uint32_t curTick = HAL_GetTick() - rgb->fadeTick;
	if(curTick >= rgb->fadeSpeed){
		if(state >= 3) state = 0;
		softwarePwm(rgb, state, rgb->Lpulse++);			//0 1 2 0
		if(state+1>=3){
			softwarePwm(rgb, 0, rgb->Hpulse--);
		}else{
			softwarePwm(rgb, state+1, rgb->Hpulse--);	//1 2 0 1
		}
		if(rgb->Hpulse <= 0){
			state ++;
			rgb->Hpulse = rgb->Period;
			rgb->Lpulse = 0;
		}
	}
}
