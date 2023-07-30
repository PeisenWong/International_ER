
/************************************************/
/*		 	 	Include Header	       		  	*/
/************************************************/

#include "SPI.h"
char uartbufff[100];
/************************************************/
/*		 	 	Functions		       		  	*/
/************************************************/

/*
 * Function Name		: SPI_Init
 * Function Description : This function is called to initialize the SPI peripheral and IMU
 * Function Remarks		: SPI1(APB2, fPCLK=84MHz) SPI2,3(APB1, fPCLK=42MHz)
 * 						  Interrrupt priorities can be set at ../BIOS/priorities.h
 * Function Arguments	:
 *						  hspix				select SPI peripheral (hspi1 or hspi2 or hspi3)
 *						  GPIOx_NSS			GPIOx group of NSS pin(x = A,B,C,D or E)
 * 						  GPIO_Pin_NSS		GPIO_Pin_x of NSS pin(x = 0,1,2,...or 15)
 * 						  Mode              SPI Mode. Can be SPI_MODE_SLAVE or SPI_MODE_MASTER
 *
 * Function Return		: None
 * Function Example		: IMU_SPI_Init(&hspix, GPIOB , GPIO_Pin_12, SPI_MODE_MASTER);
 *
 */
void SPIxInit(SPI_HandleTypeDef* hspix, GPIO_TypeDef* GPIOx_NSS, uint16_t GPIO_Pin_NSS,uint32_t Mode, int InterruptEnable){

	IRQn_Type nvic;
	GPIOPinsInit(GPIOx_NSS, GPIO_Pin_NSS, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);

	if(hspix == &hspi1){
		hspix->Instance = SPI1;
		nvic = SPI1_IRQn;
	}else if(hspix == &hspi2){
		hspix->Instance = SPI2;
		nvic = SPI2_IRQn;
	}
	else{
		hspix->Instance = SPI3;
		nvic = SPI3_IRQn;
	}

	hspix->Init.Mode = Mode;
	hspix->Init.Direction = SPI_DIRECTION_2LINES;
	hspix->Init.DataSize = SPI_DATASIZE_8BIT;
	hspix->Init.CLKPolarity = SPI_POLARITY_LOW;
	hspix->Init.CLKPhase = SPI_PHASE_1EDGE;
//	hspix->Init.CLKPhase = SPI_PHASE_2EDGE;
	hspix->Init.NSS = SPI_NSS_SOFT ;
	hspix->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
//	hspix->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspix->Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspix->Init.TIMode = SPI_TIMODE_DISABLE;
	hspix->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspix->Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}

	if(InterruptEnable){
		HAL_NVIC_SetPriority(nvic, SPI1_RX_IRQ_PRIO, 0);
		HAL_NVIC_EnableIRQ(nvic);
	}

}


void SPIx_DMA_TX_Init(SPI_HandleTypeDef* hspix, DMA_HandleTypeDef* hdma_spix,
		GPIO_TypeDef* GPIOx_NSS, uint16_t GPIO_Pin_NSS,uint32_t Mode){

	__HAL_RCC_DMA2_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, SPI1_TX_IRQ_PRIO, 0);
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream5_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

	SPIxInit(hspix, GPIOx_NSS, GPIO_Pin_NSS, Mode, DISABLE);

    hdma_spix->Instance = DMA2_Stream5;
    hdma_spix->Init.Channel = DMA_CHANNEL_3;
    hdma_spix->Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spix->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spix->Init.MemInc = DMA_MINC_ENABLE;
    hdma_spix->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spix->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spix->Init.Mode = DMA_NORMAL;
    hdma_spix->Init.Priority = DMA_PRIORITY_LOW;
    hdma_spix->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(hdma_spix) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspix,hdmatx,*hdma_spix);
}

void SPIx_DMA_RX_Init(SPI_HandleTypeDef* hspix, DMA_HandleTypeDef* hdma_spix,
		GPIO_TypeDef* GPIOx_NSS, uint16_t GPIO_Pin_NSS,uint32_t Mode){

	SPIxInit(hspix, GPIOx_NSS, GPIO_Pin_NSS, Mode, DISABLE);

    hdma_spix->Instance = DMA2_Stream2;
    hdma_spix->Init.Channel = DMA_CHANNEL_3;
    hdma_spix->Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spix->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spix->Init.MemInc = DMA_MINC_ENABLE;
    hdma_spix->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spix->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spix->Init.Mode = DMA_NORMAL;
    hdma_spix->Init.Priority = DMA_PRIORITY_LOW;
    hdma_spix->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(hdma_spix) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_LINKDMA(hspix,hdmarx,*hdma_spix);

    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, SPI1_RX_IRQ_PRIO, 0);
	HAL_NVIC_ClearPendingIRQ(DMA2_Stream2_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}
/*
 * Function Name		: SPITransmit
 * Function Description : This function is used to send data using SPI
 * Function Remarks		: This function is not for the use of user.
 * Function Arguments	: hspix			Pointer to SPI handle
 *						  data			data to be sent
 * Function Return		: None
 * Function Example		: None
 *
 */
void SPITransmit(SPI_HandleTypeDef* hspix, uint8_t *data){

	HAL_SPI_Transmit(hspix, data, sizeof(data), 100);

}

/*
 * Function Name		: SPITransmit
 * Function Description : This function is used to send data using SPI
 * Function Remarks		: This function is not for the use of user.
 * Function Arguments	: hspix			Pointer to SPI handle
 *						  data			data to be received
 * Function Return		: None
 * Function Example		: None
 *
 */
void SPIReceive(SPI_HandleTypeDef* hspix, uint8_t *data){

	HAL_SPI_Receive(hspix, data, sizeof(&data), 1000);

}

/*
 * Function Name		: SPITransmit
 * Function Description : This function is used to send data using SPI
 * Function Remarks		: This function is not for the use of user.
 * Function Arguments	: hspix			Pointer to SPI handle
 *						  txdata		data to be sent
 *						  rxdata        data to be received
 * Function Return		: None
 * Function Example		: None
 *
 */
void SPI_Transmit_Receive(SPI_HandleTypeDef* hspix, uint8_t *txdata, uint8_t *rxdata){
	HAL_SPI_TransmitReceive(hspix, txdata, rxdata, sizeof(rxdata), 1000);
}

/*
 * Function Name		: SPI1_IRQHandler
 * Function Description : SPI1 event interrupt handler.
 * Function Remarks		: This interrupt handle slave receive mode, master receive mode and slave transmit mode.
 * Function Arguments	: None
 * Function Return		: None
 * Function Example		: None
 */
void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi1);
}

/*
 * Function Name		: SPI2_IRQHandler
 * Function Description : SPI2 event interrupt handler.
 * Function Remarks		: This interrupt handle slave receive mode, master receive mode and slave transmit mode.
 * Function Arguments	: None
 * Function Return		: None
 * Function Example		: None
 */
void SPI2_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi2);
}

/*
 * Function Name		: SPI3_IRQHandler
 * Function Description : SPI3 event interrupt handler.
 * Function Remarks		: This interrupt handle slave receive mode, master receive mode and slave transmit mode.
 * Function Arguments	: None
 * Function Return		: None
 * Function Example		: None
 */
void SPI3_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi3);
}
