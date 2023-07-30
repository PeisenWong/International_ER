/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "shiftreg.h"

/************************************************/
/*		 	 	Functions		       		  	*/
/************************************************/

/*
 * Function Name		: SHIFTREGInit
 * Function Description : Configure shift register pins.
 * Function Remarks		: GPIO Mapping for shift register can be set
 * Function Arguments	: shiftreg			Pointer to structure
 * 						  cascade			Enumeration of shift register type as below:
 *											NO_CASCADE	Using 1 shift register
 *											CASCADE_1	Using 2 shift register
 *											CASCADE_2	Using 3 shift register
 *											CASCADE_3	Using 4 shift register
 *											CASCADE_4	Using 5 shift register
 *											CASCADE_5	Using 6 shift register
 * 						  GPIOx_sck			GPIOx group of shift register sck pin(x = A,B,C,D or E)
 * 						  GPIO_Pin_sck 		GPIO_Pin_x of shift register sck pin(x = 0,1,2,...or 15)
 * 						  GPIOx_rck			GPIOx group of shift register rck pin(x = A,B,C,D or E)
 * 						  GPIO_Pin_rck		GPIO_Pin_x of shift register rck pin(x = 0,1,2,...or 15)
 * 						  GPIOx_si			GPIOx group of shift register si pin(x = A,B,C,D or E)
 * 						  GPIO_Pin_si		GPIO_Pin_x of shift register si pin(x = 0,1,2,...or 15)
 * Function Return		: None
 * Function Example		: SHIFTREGInit(&shiftreg, CASCADE_1, GPIOD, GPIO_Pin_0, GPIOD, GPIO_Pin_1, GPIOD, GPIO_Pin_2);
 */

void SHIFTREGInit (shiftreg_t* shiftreg, fSR cascade, GPIO_TypeDef *GPIOx_sck, uint16_t GPIO_Pin_sck,
		           GPIO_TypeDef *GPIOx_rck , uint16_t GPIO_Pin_rck,GPIO_TypeDef *GPIOx_si, uint16_t GPIO_Pin_si){

	shiftreg->flag = cascade;

	shiftreg->GPIOx_sck = GPIOx_sck;
	shiftreg->GPIO_Pin_sck = GPIO_Pin_sck;

	shiftreg->GPIOx_rck = GPIOx_rck;
	shiftreg->GPIO_Pin_rck = GPIO_Pin_rck;

	shiftreg->GPIOx_si = GPIOx_si;
	shiftreg->GPIO_Pin_si = GPIO_Pin_si;

	GPIOPinsInit(shiftreg->GPIOx_sck, shiftreg->GPIO_Pin_sck, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
	GPIOPinsInit(shiftreg->GPIOx_rck, shiftreg->GPIO_Pin_rck, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
	GPIOPinsInit(shiftreg->GPIOx_si, shiftreg->GPIO_Pin_si , GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);

}

/*
 * Function Name		: SHIFTREGShift
 * Function Description : Shift the data to the output
 * Function Remarks		: None
 * Function Arguments	: shiftreg			pointer to structure
 * Function Return		: None
 * Function Example		: SHIFTREGShift (&shiftreg);
 */

void SHIFTREGShift(shiftreg_t* shiftreg){

	uint8_t out, i, j;

	i = shiftreg->flag;

	HAL_GPIO_WritePin(shiftreg->GPIOx_sck , shiftreg->GPIO_Pin_sck, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(shiftreg->GPIOx_rck , shiftreg->GPIO_Pin_rck, GPIO_PIN_RESET);

	do{
		out = shiftreg->cast[i].Byte;
		for (j = 8; j; j--){
			(out & 0x80) ? HAL_GPIO_WritePin(shiftreg->GPIOx_si , shiftreg->GPIO_Pin_si, GPIO_PIN_SET) : HAL_GPIO_WritePin(shiftreg->GPIOx_si , shiftreg->GPIO_Pin_si, GPIO_PIN_RESET);
			out <<= 1;
			HAL_GPIO_WritePin(shiftreg->GPIOx_sck , shiftreg->GPIO_Pin_sck, GPIO_PIN_SET);
			HAL_GPIO_WritePin(shiftreg->GPIOx_sck , shiftreg->GPIO_Pin_sck, GPIO_PIN_RESET);
		}
	}while(i--);

	HAL_GPIO_WritePin(shiftreg->GPIOx_rck , shiftreg->GPIO_Pin_rck, GPIO_PIN_SET);
	HAL_GPIO_WritePin(shiftreg->GPIOx_rck , shiftreg->GPIO_Pin_rck, GPIO_PIN_RESET);

}
