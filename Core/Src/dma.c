/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "dma.h"

/*I2C*/

void DMA1_Stream0_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hi2c1_rx_dma);
//	HAL_DMA_IRQHandler(&hdma_uart5_rx);

}

void DMA1_Stream3_IRQHandler(void)
{
//	HAL_DMA_IRQHandler(&hdma_usart3_rx);
	HAL_DMA_IRQHandler(&hi2c2_rx_dma);
}

void DMA1_Stream2_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_uart4_rx);
//	HAL_DMA_IRQHandler(&hi2c2_rx_dma);
//	HAL_DMA_IRQHandler(&hi2c3_rx_dma);
}

/*UART*/
void DMA1_Stream1_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_usart3_rx);
}

void DMA1_Stream4_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_uart4_tx);
//	HAL_DMA_IRQHandler(&hdma_usart3_tx);
//	HAL_DMA_IRQHandler(&hi2c3_tx_dma);
}

void DMA1_Stream5_IRQHandler(void)
{
//	HAL_DMA_IRQHandler(&hi2c1_rx_dma);
	HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

void DMA1_Stream6_IRQHandler(void)
{

//	HAL_DMA_IRQHandler(&hi2c1_tx_dma);
	HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

void DMA1_Stream7_IRQHandler(void){
//	HAL_DMA_IRQHandler(&hi2c1_tx_dma);
//	HAL_DMA_IRQHandler(&hi2c2_tx_dma);
	HAL_DMA_IRQHandler(&hdma_uart5_tx);
}


void DMA2_Stream0_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_adc1);
}

void DMA2_Stream1_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_adc3);
}

//void DMA2_Stream2_IRQHandler(void)
//{
//	HAL_DMA_IRQHandler(&hdma_spi1_rx);
//}

void DMA2_Stream3_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_adc2);
}

void DMA2_Stream5_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

