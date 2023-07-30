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
 * 1.1 - Added priorities.h
 * Bugs:
 *
 ************************************************/

#ifndef INC_PRIORITIES_H_
#define INC_PRIORITIES_H_

/**************************************************
 * 					DEFINES					  	  *
 *************************************************/

/* From ../../Inc/FreeRTOSConfig.h */

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */

/*  Also From https://www.freertos.org/RTOS-Cortex-M3-M4.html */
/*  If you are using an STM32 with the STM32 driver library then ensure all the
 * 	priority bits are assigned to be preempt priority bits by calling
 * 	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 ); before the RTOS is started.
 */

/*  Which means we can only assign 4 bits (0 - 15) for preemption priority
 *  and 0 bits for subpriority
 *  0 is the highest priority
 *  15 is the priority of systick interrupt
 *  Interrrupt with priority from 0 - 4 cannot call RTOS API Function
 *
 *  For communication interrupts (CAN, I2C, UART, SPI),
 *  analyze the frame rate of each comm
 *  high frame rate should have lower priority
 *  to avoid high frame rate blocking low frame rate.
 *  For example if ps4 signal(I2C) is sent to mainboard every 47us,
 *  while uart signal imu sent to mainboard every 10ms
 *  uart should have higher priority
 */

/*********** CUSTOM AREA ***********/

#define UART4_RX_IRQ_PRIO		0
#define I2C1_ER_IRQ_PRIO	 	0
#define I2C1_EV_IRQ_PRIO 	 	0     //47us
#define UART5_RX_IRQ_PRIO		14
#define CAN1_FIFO0_IRQ_PRIO		2
#define CAN2_FIFO1_IRQ_PRIO		2
#define UART3_RX_IRQ_PRIO		4
#define UART2_RX_IRQ_PRIO		5     //When needed
#define UART4_TX_IRQ_PRIO		6   //30ms
#define TIM6_IRQ_PRIO    		7     //20ms
#define DMA1_Str0__IRQ_PRIO 	10
#define UART2_TX_IRQ_PRIO		11
#define SPI1_TX_IRQ_PRIO		13

/******** CUSTOM AREA END***********/


/********** DEFAULT AREA ***********/

//#define I2C1_EV_IRQ_PRIO 	 	14
//#define I2C1_ER_IRQ_PRIO	 	14
//#define DMA1_Str0__IRQ_PRIO 	14
#define I2C2_EV_IRQ_PRIO 		14
#define I2C2_ER_IRQ_PRIO 	 	14
#define DMA1_Str3__IRQ_PRIO 	14
#define I2C3_EV_IRQ_PRIO 		14
#define I2C3_ER_IRQ_PRIO 	 	14
#define DMA1_Str2__IRQ_PRIO 	14

#define CAN2_FIFO0_IRQ_PRIO		1
#define CAN1_FIFO1_IRQ_PRIO		1

#define QEI1_IRQ_PRIO			14
#define QEI4_IRQ_PRIO			14
#define QEI6_IRQ_PRIO   		14

//#define TIM6_IRQ_PRIO    		14
#define TIM7_IRQ_PRIO   		14
#define TIM10_IRQ_PRIO  		14
#define TIM11_IRQ_PRIO  		14
#define TIM12_IRQ_PRIO  		14
#define TIM13_IRQ_PRIO  		14
#define TIM14_IRQ_PRIO  		14

#define ADC1_DMA_IRQ_PRIO		14
#define ADC2_DMA_IRQ_PRIO		14
#define ADC3_DMA_IRQ_PRIO		14

//#define UART2_RX_IRQ_PRIO		14
//#define UART3_RX_IRQ_PRIO		14
//#define UART4_RX_IRQ_PRIO		14

//#define UART2_TX_IRQ_PRIO		14
#define UART3_TX_IRQ_PRIO		14
//#define UART4_TX_IRQ_PRIO		14
#define UART5_TX_IRQ_PRIO		14

//#define SPI1_TX_IRQ_PRIO		14
#define SPI1_RX_IRQ_PRIO		14

#ifndef QEI1_IRQ_PRIO
#define TIM1_IRQ_PRIO   14
#endif
#ifndef QEI4_IRQ_PRIO
#define TIM4_IRQ_PRIO   14
#endif
#ifndef QEI6_IRQ_PRIO
#define TIM8_IRQ_PRIO   14
#endif

#define USED_BDC
#ifndef USED_BDC
#define TIM3_IRQ_PRIO   14
#define TIM5_IRQ_PRIO   14
#define TIM9_IRQ_PRIO   14
#endif

#define FREERTOS_USED
#ifndef FREERTOS_USED
#define TIM2_IRQ_PRIO   14
#endif

/******** DEFAULT AREA END **********/




#endif /* INC_PRIORITIES_H_ */
