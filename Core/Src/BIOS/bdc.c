/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "bdc.h"

/************************************************/
/*		 	 	Functions		       		  	*/
/************************************************/

/*
 * Function Name		: BDCInit
 * Function Description : This function is called to initialize desired BDC.
 * Function Remarks		: User need to initialize PWM first before calling this function.
 * Function Arguments	: bdc					pointer to structure BDC_t
 * 						  htimx                 pointer to timer handle
 * 						  channel               timer channel for pwm can be TIM_CHANNEL_1,
 * 						  						TIM_CHANNEL_2,TIM_CHANNEL_3, or TIM_CHANNEL_4
 * 						  Dirpintype 			type of direction pin (GPIO or SHIFTREG)
 * 						  ...					direction pin
 * Function Return		: None
 * Function Example		: BDCInit(&BDC1, &htim3, TIM_CHANNEL_4, SHIFTREG, &(SR.cast[1]), Bit6, Bit7);
 * 						  BDCInit(&BDC2, &htim3, TIM_CHANNEL_1, GPIO, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10);
 */

void BDCInit(BDC_t* bdc,TIM_HandleTypeDef* htimx,uint32_t Channel, BCDDIRPINType Dirpintype, ...){

	bdc->htim = htimx;
	bdc->Channel = Channel;
	bdc->Dirpintype =  Dirpintype;

	va_list pinconfig;
	va_start(pinconfig, Dirpintype);

	if( bdc->Dirpintype == GPIO){

		bdc->GPIOx_DIR1 = va_arg(pinconfig, GPIO_TypeDef*);
		bdc->GPIO_Pin_DIR1	= va_arg(pinconfig, int);
		bdc->GPIOx_DIR2 = va_arg(pinconfig, GPIO_TypeDef*);
		bdc->GPIO_Pin_DIR2	= va_arg(pinconfig, int);



		GPIOPinsInit(bdc->GPIOx_DIR1, bdc->GPIO_Pin_DIR1,GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
		GPIOPinsInit(bdc->GPIOx_DIR2, bdc->GPIO_Pin_DIR2,GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);

	}else if(bdc->Dirpintype == SHIFTREG){

		bdc->shiftreg = va_arg(pinconfig, byte_t *);
		bdc->dir1 = va_arg(pinconfig, int);
		bdc->dir2 = va_arg(pinconfig, int);
	}

	va_end(pinconfig);
	__HAL_TIM_SET_COMPARE(bdc->htim, (bdc->Channel), 0);
}

/*
 * Function Name		: WriteBDC
 * Function Description : This function is called to write value to desired BDC.
 * Function Remarks		: None
 * Function Arguments	: bdc					pointer to structure BDC_t
 * 						  pwm					value from -20000 to 20000
 * Function Return		: None
 * Function Example		: WriteBDC(&BDC1,20000);
 */

void WriteBDC(BDC_t* bdc, int32_t pwm)
{

	if(pwm >= 0){

		//*(bdc-> speed) = pwm;
		__HAL_TIM_SET_COMPARE(bdc->htim, (bdc->Channel), pwm);

		if( bdc->Dirpintype == GPIO){
			HAL_GPIO_WritePin(bdc->GPIOx_DIR1,bdc->GPIO_Pin_DIR1,GPIO_PIN_SET);
			HAL_GPIO_WritePin(bdc->GPIOx_DIR2,bdc->GPIO_Pin_DIR2,GPIO_PIN_RESET);


		}else if(bdc->Dirpintype == SHIFTREG){

			bdc->shiftreg->Byte |= bdc->dir1;
			bdc->shiftreg->Byte &= (uint8_t)~((uint8_t)bdc->dir2);
		}

	}else if (pwm < 0){

		//*(bdc-> speed) = pwm*(-1);
		__HAL_TIM_SET_COMPARE(bdc->htim, (bdc->Channel), pwm*(-1));

		if( bdc->Dirpintype == GPIO){

			HAL_GPIO_WritePin(bdc->GPIOx_DIR1,bdc->GPIO_Pin_DIR1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(bdc->GPIOx_DIR2,bdc->GPIO_Pin_DIR2,GPIO_PIN_SET);

		}else if(bdc->Dirpintype == SHIFTREG){

			bdc->shiftreg->Byte &= (uint8_t)~((uint8_t)bdc->dir1);
			bdc->shiftreg->Byte |= bdc->dir2;

		}
	}
}

/*
 * Function Name		: StopBDC
 * Function Description : This function is called to stop desired BDC.
 * Function Remarks		: None
 * Function Arguments	: bdc		pointer to structure BDC_t
 * Function Return		: None
 * Function Example		: StopBDC(&BDC1);
 */

void StopBDC(BDC_t* bdc)
{
//	*(bdc-> speed) = 0;
	__HAL_TIM_SET_COMPARE(bdc->htim, (bdc->Channel), 0);

	if(bdc->Dirpintype == GPIO){

		HAL_GPIO_WritePin(bdc->GPIOx_DIR1,bdc->GPIO_Pin_DIR1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(bdc->GPIOx_DIR2,bdc->GPIO_Pin_DIR2,GPIO_PIN_RESET);

	}else if(bdc->Dirpintype == SHIFTREG){

		bdc->shiftreg->Byte &= (uint8_t)~((uint8_t)bdc->dir1);
		bdc->shiftreg->Byte &= (uint8_t)~((uint8_t)bdc->dir2);
	}
}
