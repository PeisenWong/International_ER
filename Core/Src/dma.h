/************************************************
 * Title   : Priorities
 * Author  : Lok Charming
 * Version : 1.1
 * Date    : 28/2/2022
 * **********************************************
 * Descriptions: All interrupt and DMA priorities
 *
 *
 * Version History:
 * 1.0 - implemented using hal library
 * 1.1 - Added dma.h
 * Bugs:
 *
 ************************************************/

#ifndef INC_DMA_H_
#define INC_DMA_H_
#include "adapter.h"
#include "common.h"
/**************************************************
 * 					DEFINES					  	  *
 *************************************************/

/*
 * DMA1_STREAM0	I2C1_RX, 			UART5_RX
 * DMA1_STREAM1 					USART3_RX
 * DMA1_STREAM2 I2C2_RX, I2C3_RX 	UART4_RX
 * DMA1_STREAM3 I2C2_RX				USART3_TX
 * DMA1_STREAM4 I2C3_TX, 			UART4_TX, USART3_TX
 * DMA1_STREAM5 I2C1_RX,			USART2_RX
 * DMA1_STREAM6 I2C1_TX, 			USART2_TX
 * DMA1_STREAM7 I2C1_TX, I2C2_TX	UART5_TX
 *
 * D10 U5R
 * D15 I1R/
 *
 * DMA2_STREAM0 ADC1
 * DMA2_STREAM1	ADC3
 * DMA2_STREAM2 SPI1_RX
 * DMA2_STREAM3 ADC2
 * DMA2_STREAM4
 * DMA2_STREAM5 SPI1_TX
 * DMA2_STREAM6
 * DMA2_STREAM7
 */

#endif/*INC_DMA_H_*/
