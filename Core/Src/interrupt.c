
/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "interrupt.h"
#include "adapter.h"
#include "common.h"
/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
#define USED_QEI1
#define USED_QEI4
//#define USED_QEI6

int count = 0;
int count2 = 0;
int count3 = 0;
int _counter = 0;
/**
 * * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{

}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{

}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{

	while(1){

	}
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void)
{

}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{

}

/**
 * @brief This function handles System service call via SWI instruction.
 */
//void SVC_Handler(void)
//{
//
//}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{

}

/**
 * @brief This function handles Pendable request for system service.
 */
//void PendSV_Handler(void)
//{
//
//}

/**
 * @brief This function handles System tick timer.
 */
//void SysTick_Handler(void)
//{
//
//	HAL_IncTick();
//
//}


void TIM1_UP_TIM10_IRQHandler(void)
{
#ifdef USED_QEI1
	if (htim1.Instance -> CR1 == 129)
	{
		BIOS_QEI1.signbit += 1;
	}
	else if (htim1.Instance ->CR1 == 145)
	{
		BIOS_QEI1.signbit -= 1;
	}
	htim1.Instance -> SR = 0;
	QEIDelay(200);
#else
	HAL_TIM_IRQHandler(&htim1);
#endif
	HAL_TIM_IRQHandler(&htim10);
	return;
}



void TIM4_IRQHandler(void)
{

#ifdef USED_QEI4
	if (htim4.Instance -> CR1 == 129)
	{
		BIOS_QEI4.signbit += 1;
	}
	else if (htim4.Instance ->CR1 == 145)
	{
		BIOS_QEI4.signbit -= 1;
	}
	htim4.Instance -> SR = 0;
	QEIDelay(100);

#else
	HAL_TIM_IRQHandler(&htim4);

	return;
#endif

}



void TIM8_UP_TIM13_IRQHandler(void)
{
#ifdef USED_QEI6
	if (htim8.Instance -> CR1 == 129)
	{
		BIOS_QEI6.signbit += 1;
	}
	else if (htim8.Instance ->CR1 == 145)
	{
		BIOS_QEI6.signbit -= 1;
	}
	htim8.Instance -> SR = 0;
	QEIDelay(200);
#else
	HAL_TIM_IRQHandler(&htim8);
#endif
	HAL_TIM_IRQHandler(&htim13);
	return;
}



void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2) {
		HAL_IncTick();
		//		MUXUpdate(&MUX);
		SHIFTREGShift(&SR);
		counter++;
	}
}


//Callback for I2C RXBuffer
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == ps4.hi2cps4){
		PSxConnectDMA(&ps4);
	}

//	if(hi2c == PITCH_IMU.hi2cimu)
//	{
//		IMU_I2CHandle(&PITCH_IMU);
//	}
}

void I2C1_EV_IRQHandler(void){
	HAL_I2C_EV_IRQHandler(&hi2c1);

}
void I2C1_ER_IRQHandler(void){
	HAL_I2C_ER_IRQHandler(&hi2c1);
	HAL_DMA_DeInit(&hi2c1_rx_dma);
	HAL_I2C_DeInit(&hi2c1);

	I2CX_DMA_RX_Init(&hi2c1, &hi2c1_rx_dma, main_board_1, CLOCK_SPEED_400KHz);
//
//
//
	PSxInitDMA(&ps4, &hi2c1);

}

///*
// * Function Name		: I2C3_ER_IRQHandler
// * Function Description : I2C3 Error interrupt handler.
// * Function Remarks		: This interrupt handle the error event of I2C3.
// * Function Arguments	: None
// * Function Return		: None
// * Function Example		: None
// */
//void I2C3_ER_IRQHandler(void){
//
//	HAL_I2C_DeInit(&hi2c3);
//
//	I2CxInit (&hi2c3,main_board_1, CLOCK_SPEED_100KHz,DISABLE);
//
//	HAL_I2C_ER_IRQHandler(&hi2c3);
//
//}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
