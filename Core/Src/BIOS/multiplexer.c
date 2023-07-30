/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "multiplexer.h"

/************************************************/
/*		 	 	Functions		       		  	*/
/************************************************/

/*
 * Function Name		: MUXInit
 * Function Description : Configure multiplexer pins.
 * Function Remarks		: None
 * Function Arguments	: mux				Pointer to structure
 * 						  GPIOx_Input		GPIOx group of multiplexer output pin(x = A,B,C,D or E)
 * 						  GPIO_Pin_Input 	GPIO_Pin_x of multiplexer output pin(x = 0,1,2,...or 15)
 * 						  GPIOx_S0			GPIOx group of multiplexer selector0 pin(x = A,B,C,D or E)
 * 						  GPIO_Pin_S0		GPIO_Pin_x of multiplexer selector0 pin(x = 0,1,2,...or 15)
 * 						  GPIOx_S1			GPIOx group of multiplexer selector1 pin(x = A,B,C,D or E)
 * 						  GPIO_Pin_S1		GPIO_Pin_x of multiplexer selector1 pin(x = 0,1,2,...or 15)
 * 						  GPIOx_S2			GPIOx group of multiplexer selector2 pin(x = A,B,C,D or E)
 * 						  GPIO_Pin_S2		GPIO_Pin_x of multiplexer selector2 pin(x = 0,1,2,...or 15)
 * Function Return		: None
 * Function Example		: MUXInit(&mux1, GPIOD, GPIO_Pin_0, GPIOD, GPIO_Pin_1, GPIOD, GPIO_Pin_2, GPIOD, GPIO_Pin_3);
 */
void MUXInit(MUX_t* mux,GPIO_TypeDef *GPIOx_Input,uint16_t GPIO_Pin_Input,GPIO_TypeDef *GPIOx_S0,
		uint16_t GPIO_Pin_S0,GPIO_TypeDef *GPIOx_S1,uint16_t GPIO_Pin_S1,GPIO_TypeDef *GPIOx_S2,
		uint16_t GPIO_Pin_S2)
{
	mux->GPIOx_Input = GPIOx_Input;
	mux->GPIO_Pin_Input = GPIO_Pin_Input;
	mux->GPIOx_S0 = GPIOx_S0;
	mux->GPIO_Pin_S0 = GPIO_Pin_S0;
	mux->GPIOx_S1 = GPIOx_S1;
	mux->GPIO_Pin_S1 = GPIO_Pin_S1;
	mux->GPIOx_S2 = GPIOx_S2;
	mux->GPIO_Pin_S2 = GPIO_Pin_S2;

	GPIOPinsInit(mux->GPIOx_Input,mux->GPIO_Pin_Input,GPIO_MODE_INPUT,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
	GPIOPinsInit(mux->GPIOx_S0,mux->GPIO_Pin_S0, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
	GPIOPinsInit(mux->GPIOx_S1,mux->GPIO_Pin_S1, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
	GPIOPinsInit(mux->GPIOx_S2,mux->GPIO_Pin_S2, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);

}

/*
 * Function Name		: MUXSelect
 * Function Description : This function is called to select input channel.
 * Function Remarks		: None
 * Function Arguments	: mux		pointer to structure
 * 						  select	selected input channel of multiplexer
 * Function Return		: Read data bit from multiplexer
 * Function Example		: mux->bit0 = MUXSelect(&mux, 0);
 */
uint8_t MUXSelect(MUX_t* mux, uint8_t select)
{
	mux->select = select;

	(mux->s0 == 1) ? HAL_GPIO_WritePin(mux->GPIOx_S0, mux->GPIO_Pin_S0, GPIO_PIN_SET) : HAL_GPIO_WritePin(mux->GPIOx_S0, mux->GPIO_Pin_S0, GPIO_PIN_RESET);
	(mux->s1 == 1) ? HAL_GPIO_WritePin(mux->GPIOx_S1, mux->GPIO_Pin_S1, GPIO_PIN_SET) : HAL_GPIO_WritePin(mux->GPIOx_S1, mux->GPIO_Pin_S1, GPIO_PIN_RESET);
	(mux->s2 == 1) ? HAL_GPIO_WritePin(mux->GPIOx_S2, mux->GPIO_Pin_S2, GPIO_PIN_SET) : HAL_GPIO_WritePin(mux->GPIOx_S2, mux->GPIO_Pin_S2, GPIO_PIN_RESET);

	return HAL_GPIO_ReadPin(mux->GPIOx_Input, mux->GPIO_Pin_Input);
}

/*
 * Function Name		: MUXUpdate
 * Function Description : This function is called to open all the IO pins connected to multiplexer
 * 						  and store the data read from each IO pins to the structure.
 * Function Remarks		: None
 * Function Arguments	: mux	pointer to structure
 * Function Return		: None
 * Function Example		: MUXUpdate(&mux1);
 */
void MUXUpdate(MUX_t* mux){

	for(int i = 0; i < 8 ; i++){

		switch(i){

			case 0 : mux->mux_data.bit0 = MUXSelect(mux, i);
				break;

			case 1 : mux->mux_data.bit1 = MUXSelect(mux, i);
				break;

			case 2 : mux->mux_data.bit2 = MUXSelect(mux, i);
				break;

			case 3 : mux->mux_data.bit3 = MUXSelect(mux, i);
				break;

			case 4 : mux->mux_data.bit4 = MUXSelect(mux, i);
				break;

			case 5 : mux->mux_data.bit5 = MUXSelect(mux, i);
				break;

			case 6 : mux->mux_data.bit6 = MUXSelect(mux, i);
				break;

			case 7 : mux->mux_data.bit7 = MUXSelect(mux, i);
				break;

			default:
				break;
		}

	}

}
