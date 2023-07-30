
/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "i2c.h"


/************************************************/
/*		 	 	Functions		       		  	*/
/************************************************/
/* Function Name		: I2CxInit
 * Function Description : This function is called to initialise I2C.
 * Function Remarks		:
 * Function Arguments	:
 *
 *						  hi2cx                         Pointer to I2C handle
 * 						  OwnAdress						address of this device
 * 						  ClockSpeed					Speed of transmission can be CLOCK_SPEED_100KHz or CLOCK_SPEED_400KHz
 * 						  InterruptEnable				interrupt enable for I2C can be ENABLE or DISABLE
 * Function Return		: NONE
 * Function Example		: I2CxInit (&hi2c1,main_board_1, CLOCK_SPEED_400KHz,ENABLE);
 */
void I2CxInit(I2C_HandleTypeDef* hi2cx,uint8_t OwnAddress,uint32_t ClockSpeed,int InterruptEnable)
{

	IRQn_Type nvicER = 0;
	IRQn_Type nvicEV = 0;

	if(hi2cx == &hi2c1){
		hi2cx->Instance = I2C1;
		nvicER = I2C1_ER_IRQn;
		nvicEV = I2C1_EV_IRQn;
	}else if(hi2cx == &hi2c2){
		hi2cx->Instance = I2C2;
		nvicER = I2C2_ER_IRQn;
		nvicEV = I2C2_EV_IRQn;
	}else{
		hi2cx->Instance = I2C3;
		nvicER = I2C3_ER_IRQn;
		nvicEV = I2C3_EV_IRQn;
	}

	hi2cx->Init.ClockSpeed = ClockSpeed;
	hi2cx->Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2cx->Init.OwnAddress1 = OwnAddress<<1;
	hi2cx->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2cx->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2cx->Init.OwnAddress2 = 0;
	hi2cx->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2cx->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;



	if(InterruptEnable){
		if(hi2cx == &hi2c1){
			HAL_NVIC_SetPriority(nvicER, I2C1_ER_IRQ_PRIO, 0);
			HAL_NVIC_EnableIRQ(nvicER);
			HAL_NVIC_SetPriority(nvicEV, I2C1_EV_IRQ_PRIO, 0);
			HAL_NVIC_EnableIRQ(nvicEV);
		}else if(hi2cx == &hi2c2){
			HAL_NVIC_SetPriority(nvicER, I2C2_ER_IRQ_PRIO, 0);
			HAL_NVIC_EnableIRQ(nvicER);
			HAL_NVIC_SetPriority(nvicEV, I2C2_EV_IRQ_PRIO, 0);
			HAL_NVIC_EnableIRQ(nvicEV);
		}else{
			HAL_NVIC_SetPriority(nvicER, I2C3_ER_IRQ_PRIO, 0);
			HAL_NVIC_EnableIRQ(nvicER);
			HAL_NVIC_SetPriority(nvicEV, I2C3_EV_IRQ_PRIO, 0);
			HAL_NVIC_EnableIRQ(nvicEV);
		}
	}


	if (HAL_I2C_Init(hi2cx) != HAL_OK)
	{
		Error_Handler();
	}

}

/************************************************/
/*		 	 	Functions		       		  	*/
/************************************************/
/* Function Name		: I2CX_DMA_RX_Init
 * Function Description : This function is called to initialise I2C With DMA for Receiving.
 * Function Remarks		:
 * Function Arguments	:
 *
 *						  hi2cx                         Pointer to I2C handle
 *						  hdma_i2cx						Pointer to DMA Handle
 * 						  OwnAdress						address of this device
 * 						  ClockSpeed					Speed of transmission can be CLOCK_SPEED_100KHz or CLOCK_SPEED_400KHz
 * Function Return		: NONE
 * Function Example		: I2CX_DMA_RX_Init(&hi2c1, &hi2c1_rx_dma, main_board_1, CLOCK_SPEED_400KHz);
 *
 */
void I2CX_DMA_RX_Init(I2C_HandleTypeDef *hi2cx,DMA_HandleTypeDef* hdma_i2cx,uint8_t OwnAddress,uint32_t ClockSpeed){

	IRQn_Type nvic,nvicER,nvicEV;
	uint32_t DMA_CHANNEL;
	__HAL_RCC_DMA1_CLK_ENABLE();
	if(hi2cx == &hi2c1){

		hi2cx->Instance = I2C1;

		hdma_i2cx->Instance = DMA1_Stream0;
		DMA_CHANNEL= DMA_CHANNEL_1;
		nvic = DMA1_Stream0_IRQn;

		nvicER = I2C1_ER_IRQn;
		nvicEV = I2C1_EV_IRQn;
		__HAL_RCC_DMA1_CLK_ENABLE();
	}else if(hi2cx == &hi2c2){

		hi2cx->Instance = I2C2;

		hdma_i2cx->Instance = DMA1_Stream3;
		DMA_CHANNEL= DMA_CHANNEL_7;
		nvic = DMA1_Stream3_IRQn;
		nvicER = I2C2_ER_IRQn;
		nvicEV = I2C2_EV_IRQn;

	}else{
		hi2cx->Instance = I2C3;

		hdma_i2cx->Instance = DMA1_Stream2;
		DMA_CHANNEL= DMA_CHANNEL_3;
		nvic = DMA1_Stream2_IRQn;
		nvicER = I2C3_ER_IRQn;
		nvicEV = I2C3_EV_IRQn;
	}

	hdma_i2cx->Init.Channel = DMA_CHANNEL;
	hdma_i2cx->Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_i2cx->Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_i2cx->Init.MemInc = DMA_MINC_ENABLE;
	hdma_i2cx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_i2cx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_i2cx->Init.Mode = DMA_NORMAL;
	hdma_i2cx->Init.Priority = DMA_PRIORITY_HIGH;
	hdma_i2cx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	hdma_i2cx->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	hdma_i2cx->Init.MemBurst = DMA_MBURST_INC4;
	hdma_i2cx->Init.PeriphBurst = DMA_PBURST_INC4;
	if (HAL_DMA_Init(hdma_i2cx) != HAL_OK)
	{

		Error_Handler();

	}
	__HAL_LINKDMA(hi2cx,hdmarx,*hdma_i2cx);

	//


	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */

	//	__HAL_DMA_ENABLE_IT(hdma_i2cx,DMA_IT_TC);
	if(hi2cx == &hi2c1){
		HAL_NVIC_SetPriority(nvic, DMA1_Str0__IRQ_PRIO, 0);
		HAL_NVIC_ClearPendingIRQ(nvic);
		HAL_NVIC_EnableIRQ(nvic);
	}else if(hi2cx == &hi2c2){
		HAL_NVIC_SetPriority(nvic, DMA1_Str3__IRQ_PRIO, 0);
		HAL_NVIC_ClearPendingIRQ(nvic);
		HAL_NVIC_EnableIRQ(nvic);
	}else{
		HAL_NVIC_SetPriority(nvic, DMA1_Str2__IRQ_PRIO, 0);
		HAL_NVIC_ClearPendingIRQ(nvic);
		HAL_NVIC_EnableIRQ(nvic);
	}

	hi2cx->Init.ClockSpeed = ClockSpeed;
	hi2cx->Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2cx->Init.OwnAddress1 = OwnAddress<<1;
	hi2cx->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2cx->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2cx->Init.OwnAddress2 = 0;
	hi2cx->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2cx->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	if(hi2cx == &hi2c1){
		HAL_NVIC_SetPriority(nvicER, I2C1_ER_IRQ_PRIO, 0);
		HAL_NVIC_EnableIRQ(nvicER);
		HAL_NVIC_SetPriority(nvicEV, I2C1_EV_IRQ_PRIO, 0);
		HAL_NVIC_EnableIRQ(nvicEV);
	}else if(hi2cx == &hi2c2){
		HAL_NVIC_SetPriority(nvicER, I2C2_ER_IRQ_PRIO, 0);
		HAL_NVIC_EnableIRQ(nvicER);
		HAL_NVIC_SetPriority(nvicEV, I2C2_EV_IRQ_PRIO, 0);
		HAL_NVIC_EnableIRQ(nvicEV);
	}else{
		HAL_NVIC_SetPriority(nvicER, I2C3_ER_IRQ_PRIO, 0);
		HAL_NVIC_EnableIRQ(nvicER);
		HAL_NVIC_SetPriority(nvicEV, I2C3_EV_IRQ_PRIO, 0);
		HAL_NVIC_EnableIRQ(nvicEV);
	}

	if (HAL_I2C_Init(hi2cx) != HAL_OK)
	{

		Error_Handler();
	}
}


/*
 * Function Name		: I2CSend
 * Function Description : Send data in the packet defined by the RBC protocol.
 * Function Remarks		: The data pointed by data can be char, int, long, float, struct, union, array, etc.
 * Function Arguments	: hi2cx 		Pointer to I2C handle
 *						  slave_addr 	receiver�s or target�s address
 *						  len 	 		length of data to be sent
 *						  data 			pointer to data to be sent
 * Function Return		: HAL_StatusTypeDef will return the status of transmission.
 * 											can be HAL_OK, HAL_ERROR, HAL_BUSY or HAL_TIMEOUT.
 * Function Example		: float speed = 123.45;
 *						  I2CSend(I2C1, main_board_1, sizeof(speed), &speed);
 */
HAL_StatusTypeDef I2CSend(I2C_HandleTypeDef* hi2cx,uint32_t slave_addr, uint8_t len, const void *data){

	uint8_t master_snd_buf[256];
	uint8_t index = 0;
	uint8_t tmp_len = len;

	while (tmp_len--) {
		master_snd_buf[index++] = *(uint8_t *)data++;
	}

	return HAL_I2C_Master_Transmit(hi2cx,slave_addr, master_snd_buf,len,I2C_LONG_TIMEOUT);
}

/*
 * Function Name		: I2CSendV
 * Function Description : Send data in the packet defined by the RBC protocol.
 * Function Remarks		: The data in the argument list must have the size of a single char.
 * 						  Longer data must be separated into byte-sized argument as input to the function,
 * 						  and its length must correspond to the second parameter len.
 * Function Arguments	: hi2cx 	Pointer to I2C handle
 *						  addr 		receiver�s or target�s address
 *						  len 		length of data to be sent
 *						  ... 		data to be sent
 * Function Return		: HAL_StatusTypeDef will return the status of transmission.
 * 											can be HAL_OK, HAL_ERROR, HAL_BUSY or HAL_TIMEOUT.
 * Function Example		: long output = 0x90ABCDEF;
 *						  I2CSendV (I2C1, main_board_1, sizeof(long),
 *						  (char)(output&0xff), (char)(output >> 8 & 0xff), (char)(output >> 16 & 0xff), (char)(output >> 24 & 0xff));
 */
HAL_StatusTypeDef I2CSendV(I2C_HandleTypeDef *hi2cx, uint8_t slave_addr, uint8_t len, ...){

	uint8_t master_snd_buf[256];
	uint8_t tmp_len = len;
	uint8_t index = 0;
	va_list vdata;

	va_start(vdata, len);
	while (tmp_len--) {
		master_snd_buf[index++] =  (uint8_t)va_arg(vdata, int);
	}
	va_end(vdata);


	return HAL_I2C_Master_Transmit(hi2cx,slave_addr, master_snd_buf, len, I2C_LONG_TIMEOUT);

}

/* Function Name		: I2CWriteReg8
 * Function Description : This function is called to write to a 8bit register
 * Function Remarks		:
 * Function Arguments	:
 *
 *						  hi2cx                         Pointer to I2C handle
 * 						  slave_addr				    address of target device
 * 						  register_pointer				register address
 * 						  register_value				register value to be written
 * Function Return		: NONE
 * Function Example		:   I2CWriteReg8(srv_drv->hi2cx,srv_drv->_i2caddr,PCA9685_MODE1,oldmode);
 */
void I2CWriteReg8(I2C_HandleTypeDef *hi2cx, uint8_t slave_addr,uint8_t register_pointer, uint8_t register_value)
{

	if(HAL_I2C_Mem_Write(hi2cx,slave_addr<<1,(uint8_t)register_pointer, I2C_MEMADD_SIZE_8BIT, (uint8_t*)(&register_value), 1, I2C_LONG_TIMEOUT) != HAL_OK)
	{
		// Error handling, for example re-initialization of the I2C peripheral
	}
}


/* Function Name		: I2CReadReg8
 * Function Description : This function is called to Read from a 8bit register
 * Function Remarks		:
 * Function Arguments	:
 *
 *						  hi2cx                         Pointer to I2C handle
 * 						  slave_addr				    address of target device
 * 						  register_pointer				register address to read
 * Function Return		: value of the register
 * Function Example		:   uint8_t awake = I2CReadReg8(srv_drv->hi2cx,srv_drv->_i2caddr,PCA9685_MODE1);
 */
uint8_t I2CReadReg8(I2C_HandleTypeDef *hi2cx, uint8_t slave_addr,uint8_t register_pointer)
{
	uint8_t return_value;

	/* Check the communication status */
	if(HAL_I2C_Mem_Read(hi2cx,slave_addr<<1, (uint8_t)register_pointer, I2C_MEMADD_SIZE_8BIT, &return_value, 1, I2C_LONG_TIMEOUT) != HAL_OK)
	{
		//Error handling, for example re-initialization of the I2C peripheral
	}

	return return_value;
}

/*
 * Function Name		: I2C1_EV_IRQHandler
 * Function Description : I2C1 event interrupt handler.
 * Function Remarks		: This interrupt handle slave receive mode, master receive mode and slave transmit mode.
 * Function Arguments	: None
 * Function Return		: None
 * Function Example		: None
 */
//void I2C1_EV_IRQHandler(void){
//
//	HAL_I2C_EV_IRQHandler(&hi2c1);
//
//}
////
///*
// * Function Name		: I2C1_ER_IRQHandler
// * Function Description : I2C1 Error interrupt handler.
// * Function Remarks		: This interrupt handle the error event of I2C1.
// * Function Arguments	: None
// * Function Return		: None
// * Function Example		: None
// */
//void I2C1_ER_IRQHandler(void){
//
//	HAL_DMA_DeInit(&hi2c1_rx_dma);
//	HAL_I2C_DeInit(&hi2c1);
//
//	I2CX_DMA_RX_Init(&hi2c1, &hi2c1_rx_dma, main_board_1, CLOCK_SPEED_400KHz);
//
//	HAL_I2C_ER_IRQHandler(&hi2c1);
//
//	PSxInitDMA(&ps4, &hi2c1);
//
//
//}


/*
 * Function Name		: I2C2_EV_IRQHandler
 * Function Description : I2C2 event interrupt handler.
 * Function Remarks		: This interrupt handle slave receive mode, master receive mode and slave transmit mode.
 * Function Arguments	: None
 * Function Return		: None
 * Function Example		: None
 */
void I2C2_EV_IRQHandler(void){


	HAL_I2C_EV_IRQHandler(&hi2c2);


}

/*
 * Function Name		: I2C2_ER_IRQHandler
 * Function Description : I2C2 Error interrupt handler.
 * Function Remarks		: This interrupt handle the error event of I2C2.
 * Function Arguments	: None
 * Function Return		: None
 * Function Example		: None
 */
void I2C2_ER_IRQHandler(void){

	HAL_I2C_ER_IRQHandler(&hi2c2);


}

/*
 * Function Name		: I2C3_EV_IRQHandler
 * Function Description : I2C3 event interrupt handler.
 * Function Remarks		: This interrupt handle slave receive mode, master receive mode and slave transmit mode.
 * Function Arguments	: None
 * Function Return		: None
 * Function Example		: None
 */
void I2C3_EV_IRQHandler(void){

	//	taskENTER_CRITICAL();
	HAL_I2C_EV_IRQHandler(&hi2c3);
	//	taskEXIT_CRITICAL();

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
//	HAL_I2C_ER_IRQHandler(&hi2c3);
//
//}




//void DMA1_Stream2_IRQHandler(void){
//	if(hi2c2_rx_dma.Instance == I2C2){
//		HAL_DMA_IRQHandler(&hi2c2_rx_dma);
//	}
//
//	if(hi2c3_rx_dma.Instance == I2C3){
//		HAL_DMA_IRQHandler(&hi2c3_rx_dma);
//	}
//}

