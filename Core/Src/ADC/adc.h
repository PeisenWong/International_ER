/*
 * adc.h
 *
 *  Applied to HAL by Anas Amer 10/12/2020
 */
/*				Pin		Channel
 * ADC1/2/3		PA0		0
 * 				PA1		1
 * 				PA2		2
 * 				PA3		3
 * 				PC0		10
 * 				PC1		11
 * 				PC2		12
 * 				PC3		13
 * ADC1/2		PA4		4
 * 				PA5		5
 * 				PA6		6
 * 				PA7		7
 * 				PB0		8
 * 				PB1		9
 * 				PC4		14
 * 				PC5		15
 *
 *				Pin		Channel
 * ADC1			PA0		0
 *				PA1		1
 *				PA2		2
 *				PA3		3
 *				PA4		4
 *				PA5		5
 *				PA6		6
 *				PA7		7
 *				PB0		8
 *				PB1		9
 *				PC0		10
 *				PC1		11
 *				PC2		12
 *				PC3		13
 *				PC4		14
 *				PC5		15
 *
 *
 *
 * */

#ifndef ADC_ADC_H_
#define ADC_ADC_H_

#include "../BIOS/bios.h"

typedef struct{
	ADC_HandleTypeDef* hadcx;
	uint8_t 	ADC_number;
	uint8_t 	ADC_rank;
	uint16_t*	ADC_valuep;
	uint16_t    ADC_value[7];
}ADC_t;

ADC_t adc;

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;


void ADC_DMAxInit(ADC_t* ADC_t,ADC_HandleTypeDef* hadcx,DMA_HandleTypeDef* hdma_adcx, uint32_t ChannelNo);

void ADC_Channel_Config(ADC_t* ADC_t,uint32_t ADC_Channel, GPIO_TypeDef* DMA_GPIOx, uint16_t DMA_GPIO_Pin);
#endif /* ADC_ADC_H_ */
