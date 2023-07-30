/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "qei.h"

/************************************************/
/*		 	 	Declarations		  		  	*/
/************************************************/

qei_TypeDef BIOS_QEI1;
qei_TypeDef BIOS_QEI2;
qei_TypeDef BIOS_QEI3;
qei_TypeDef BIOS_QEI4;
qei_TypeDef BIOS_QEI5;
qei_TypeDef BIOS_QEI6;
enc_TypeDef qei;


/************************************************/
/*		 	 	Functions		       		  	*/
/************************************************/

/*
 * Function Name		: QEIInit
 * Function Description : This function is called to initialize desired QEIx.
 * Function Remarks		: QEI1(TIM1)
 * 						  QEI2(TIM2)
 * 						  QEI3(TIM3)
 * 						  QEI4(TIM4)
 * 						  QEI5(TIM5)
 * 						  QEI6(TIM8)
 * 						  Priorities can be set at priorities.h
 * Function Arguments	: htimx					Pointer to timer handle
 * Function Return		: None
 * Function Example		: QEIInit(&htim1,5,5);  	//Initialize QEI1
 */
void QEIInit(TIM_HandleTypeDef* htimx)
{
	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	IRQn_Type nvic;

	if(htimx == &htim1){
		htimx->Instance = TIM1;
		nvic = TIM1_UP_TIM10_IRQn;
	}
	else if(htimx == &htim2)	{
		htimx->Instance = TIM2;
		nvic=TIM2_IRQn;
	}
	else if(htimx == &htim3)	{
		htimx->Instance = TIM3;
		nvic=TIM3_IRQn;
	}
	else if(htimx == &htim4)	{
		htimx->Instance = TIM4;
		nvic=TIM4_IRQn;
	}
	else if(htimx == &htim5)	{
		htimx->Instance = TIM5;
		nvic=TIM5_IRQn;
	}
	else if(htimx == &htim8)	{
		htimx->Instance = TIM8;
		nvic=TIM8_UP_TIM13_IRQn;
	}

	htimx->Init.Prescaler = 0;
	htimx->Init.CounterMode = TIM_COUNTERMODE_UP;
	htimx->Init.Period = 0xFFFF;
	htimx->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htimx->Init.RepetitionCounter = 0;
	htimx->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(htimx, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(htimx, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	__HAL_TIM_CLEAR_IT(htimx, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE_IT(htimx, TIM_IT_UPDATE);

	if(htimx == &htim1){
		HAL_NVIC_SetPriority(nvic, QEI1_IRQ_PRIO, 0);
		HAL_NVIC_ClearPendingIRQ(nvic);
		HAL_NVIC_EnableIRQ(nvic);
	}else if(htimx == &htim4){
		HAL_NVIC_SetPriority(nvic, QEI4_IRQ_PRIO, 0);
		HAL_NVIC_ClearPendingIRQ(nvic);
		HAL_NVIC_EnableIRQ(nvic);
	}else if(htimx == &htim8){
		HAL_NVIC_SetPriority(nvic, QEI6_IRQ_PRIO, 0);
		HAL_NVIC_ClearPendingIRQ(nvic);
		HAL_NVIC_EnableIRQ(nvic);
	}

	HAL_TIM_Encoder_Start(htimx, TIM_CHANNEL_ALL);
}

/*
 * Function Name		: QEIPrintValueStore
 * Function Description : Print QEIs' Values.
 * Function Remarks		: Called this function in while(1) to print QEIs' values, x can be 1 to 5.
 * Function Arguments	: huartx, where x can be 1 to 6.
 * Function Return		: None
 * Function Example		: void main(main)
 * 							{
 * 								while(1)
 * 								{
 * 									 QEIPrintValueStore(UART5);	//Print QEIs' Value using UART5
 * 								}
 * 							}
 */

void QEIPrintValueStore(UART_HandleTypeDef* huartx)
{
	qei.l[0].LONG = TIM1-> CNT;
	qei.l[1].LONG = TIM2-> CNT;
	qei.l[2].LONG = TIM3-> CNT;
	qei.l[3].LONG = TIM4-> CNT;
	qei.l[4].LONG = TIM5-> CNT;
	qei.l[5].LONG = TIM8-> CNT;
	char QEIbuf[100];
	sprintf(QEIbuf,"QEI1= %ld \t QEI2=%ld \t QEI3=%ld \t QEI4=%ld \t QEI5=%ld \t QEI6=%ld \r\n",qei.l[0].LONG, qei.l[1].LONG,qei.l[2].LONG,qei.l[3].LONG,qei.l[4].LONG,qei.l[5].LONG);
	UARTPrintString(huartx, QEIbuf);
}

/*
 * Function Name		: QEIWrite
 * Function Description : This function is called to write value QEI.
 * Function Remarks		: None
 * Function Arguments	: QEIx		where x can be 1 to 5.
 * 						  value		desired value to be set
 * Function Return		: None
 * Function Example		: QEIWrite(QEI1, 10000);
 */

void QEIWrite(QEI_TypeDef QEIx, int32_t value)
{

	switch (QEIx){

	case QEI1:	BIOS_QEI1.count = value;
	TIM1->CNT = BIOS_QEI1.poscnt;
	break;

	case QEI2:	BIOS_QEI2.count = value;
	TIM2->CNT = BIOS_QEI2.poscnt;
	break;

	case QEI3:  BIOS_QEI3.count = value;
	TIM3->CNT = BIOS_QEI3.poscnt;
	break;

	case QEI4:  BIOS_QEI4.count = value;
	TIM4->CNT = BIOS_QEI4.poscnt;
	break;

	case QEI5:	BIOS_QEI5.count = value;
	TIM5->CNT = BIOS_QEI5.poscnt;
	break;

	case QEI6:	BIOS_QEI6.count = value;
	TIM8->CNT = BIOS_QEI6.poscnt;
	break;

	}

}

/*
 * Function Name		: QEIReset
 * Function Description : This function is called to reset value of QEI.
 * Function Remarks		: None
 * Function Arguments	: QEIx		where x can be 1 to 5
 * Function Return		: None
 * Function Example		: QEIReset(QEI1);
 */

void QEIReset(QEI_TypeDef QEIx)
{

	switch (QEIx){

	case QEI1:	TIM1->CNT = 0;
	BIOS_QEI1.count = 0;
	break;

	case QEI2:	TIM2->CNT = 0;
	BIOS_QEI2.count = 0;
	break;

	case QEI3:	TIM3->CNT = 0;
	BIOS_QEI3.count = 0;
	break;

	case QEI4:	TIM4->CNT = 0;
	BIOS_QEI4.count = 0;
	break;

	case QEI5:	TIM5->CNT = 0;
	BIOS_QEI5.count = 0;
	break;

	case QEI6:	TIM8->CNT = 0;
	BIOS_QEI6.count = 0;
	break;

	}
}

/*
 * Function Name		: QEISwap
 * Function Description : This function is called to swap the direction of QEI's value.
 * Function Remarks		: None
 * Function Arguments	: QEIx		where x can be 1 to 5.
 * 						  swap		can be QEI_No_Swap or QEI_Swap
 * Function Return		: None
 * Function Example		: QEISwap(QEI1, QEI_Swap);
 */

void QEISwap(QEI_TypeDef QEIx, QEI_Direction_TypeDef swap)
{

	switch(QEIx){

	case QEI1:	if(swap == QEI_No_Swap){
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);
	}
	else if(swap == QEI_Swap){
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
		break;

	case QEI2:	if(swap == QEI_No_Swap){
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);
	}
	else if(swap == QEI_Swap){
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
	}
	break;

	case QEI3:	if(swap == QEI_No_Swap){
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);
	}
	else if(swap == QEI_Swap){
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
	}
	break;

	case QEI4:	if(swap == QEI_No_Swap){
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);
	}
	else if(swap == QEI_Swap){
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
	}

	break;

	case QEI5:	if(swap == QEI_No_Swap){
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);
	}
	else if(swap == QEI_Swap){
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
	}

	break;

	case QEI6:	if(swap == QEI_No_Swap){
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);
	}
	else if(swap == QEI_Swap){
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
	}

	break;

	default: break;

	}

	}
}

/*
 * Function Name		: QEIDelay
 * Function Description : This function is called to induce delay in timer interrupt.
 * Function Remarks		: This function is called by developer in corresponding timer interrupt for respective QEIx,
 * 						  x can be 1 to 5.
 * Function Arguments	: value							100 for TIM2 to TIM5
 * 														200 for TIM1 and TIM8
 * Function Return		: None
 * Function Example		: QEIDelay(100);
 */

void QEIDelay(uint8_t value)
{
	uint8_t currentvalues;
	value = value * 8400;
	while(currentvalues < value)
	{
		currentvalues++;
	}
	currentvalues = 0;
}

/*
 * Function Name		: QEIRead
 * Function Description : This function is called to raed of QEI's value.
 * Function Remarks		: None
 * Function Arguments	: QEIx		where x can be 1 to 5.
 * Function Return		: QEI's count with 32 bits with sign.
 * Function Example		: QEIRead(QEI1);
 */

int32_t QEIRead(QEI_TypeDef QEIx){

	int32_t value;
	switch(QEIx){

	case QEI1:	BIOS_QEI1.poscnt = TIM1-> CNT;
	value = BIOS_QEI1.count;
	break;

	case QEI2:	BIOS_QEI2.poscnt = TIM2-> CNT;
	value = BIOS_QEI2.count;
	break;

	case QEI3:	BIOS_QEI3.poscnt = TIM3-> CNT;
	value = BIOS_QEI3.count;
	break;

	case QEI4:	BIOS_QEI4.poscnt = TIM4-> CNT;
	value = BIOS_QEI4.count;
	break;

	case QEI5:	BIOS_QEI5.poscnt = TIM5-> CNT;
	value = BIOS_QEI5.count;
	break;

	case QEI6:	BIOS_QEI6.poscnt = TIM8-> CNT;
	value = BIOS_QEI6.count;
	break;

	default: break;
	}

	return value;
}

/*
 * Function Name		: QEIReadRaw
 * Function Description : This function is called to raed of QEI's raw value.
 * Function Remarks		: None
 * Function Arguments	: QEIx		where x can be 1 to 5.
 * Function Return		: QEI's raw count wuth range from 0 until 65535
 * Function Example		: QEIRead(QEI1);
 */

uint32_t QEIReadRaw(QEI_TypeDef QEIx){

	uint32_t value;
	switch(QEIx){

	case QEI1:	value = TIM1->CNT;
	break;

	case QEI2:	value = TIM2->CNT;
	break;

	case QEI3:	value = TIM3->CNT;
	break;

	case QEI4:	value = TIM4->CNT;
	break;

	case QEI5:	value = TIM5->CNT;
	break;

	case QEI6:	value = TIM8->CNT;
	break;

	default: break;
	}

	return value;
}




