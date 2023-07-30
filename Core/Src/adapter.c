

/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "adapter.h"


//char uartbuff[500];
uint8_t mailbox = 0, buf2_flag = 0, buf2_flagC2 = 0;

/************************************************/
/*		 	 	Functions		       		  	*/
/************************************************/
void Initialize(){
	/* MCU Configuration--------------------------------------------------------*/
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	GPIOPinsInit (LED1_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
	GPIOPinsInit (LED2_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
	GPIOPinsInit (LED3_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);

	GPIOPinsInit (PB1_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
	GPIOPinsInit (PB2_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);

	/*Normal IOs*/
	GPIOPinsInit (IP1_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP2_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP3_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP4_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP5_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP6_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP7_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP8_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP9_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP10_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP11_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP12_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP13_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP14_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP15_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);

	GPIOPinsInit (IP16_Analog1_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP17_Analog2_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP18_Analog3_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP19_Analog4_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP20_Analog5_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP21_Analog6_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);

	//Unused peripheral pins can be used as GPIO Input or Output
//	GPIOPinsInit (UART3_Rx, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);

//	MUXInit(&MUX, MUX1_INPUT_PIN, MUX1_S0_PIN, MUX1_S1_PIN, MUX1_S2_PIN);
	SHIFTREGInit (&SR, CASCADE_1, SR_SCK_PIN, SR_RCK_PIN, SR_SI_PIN);

	//https://stackoverflow.com/questions/50243996/what-are-valid-values-of-hal-nvic-setpriority-when-using-stm32-and-freertos
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);


	/*************** Set Interrupt Priorities in BIOS/priorities.h ***************/

	I2CX_DMA_RX_Init(&hi2c1, &hi2c1_rx_dma, main_board_1, CLOCK_SPEED_400KHz);
	I2CxInit (&hi2c2,main_board_1, CLOCK_SPEED_100KHz,ENABLE);
//	I2CX_DMA_RX_Init(&hi2c2, &hi2c2_rx_dma, main_board_1, CLOCK_SPEED_400KHz);
//	I2CxInit(&hi2c2, main_board_1, CLOCK_SPEED_100KHz, ENABLE);

	//Servo Driver - recommended to use 100KHz I2C as 400KHz hang frequently
//	I2CxInit (&hi2c3,main_board_1, CLOCK_SPEED_100KHz,DISABLE);
//	ServoDriverInit(&srv_drv,&hi2c3,0x40);

	UARTx_DMA_Rx_Init(&huart2, &hdma_usart2_rx, 115200);//Bluebee Tuning
//	UARTx_DMA_Rx_Init(&huart4, &hdma_uart4_rx, 115200); //Jetson
	UARTInit(&huart3, 115200, ENABLE); // ROS UART
	UARTInit(&huart4, 115200, ENABLE); // ROS UART
	UARTInit(&huart5, 115200, ENABLE);

	QEIInit(&htim1);
	QEIInit(&htim4);
	QEIInit(&htim8);

	CANxInit(&hcan1,4,CAN_FILTER_FIFO0,0,0,0,CAN_500KHz);
	CANxInit(&hcan2,4,CAN_FILTER_FIFO1,0,0,14,CAN_500KHz);

	PWMTimeBaseInit(&htim3, 19999, 83);
	PWMChannelConfig(&htim3, TIM_CHANNEL_3, TIM3_CHANNEL3_PIN);
	PWMChannelConfig(&htim3, TIM_CHANNEL_4 , TIM3_CHANNEL4_PIN);

	PWMTimeBaseInit(&htim5, 19999, 83);
	PWMChannelConfig(&htim5, TIM_CHANNEL_1, TIM5_CHANNEL1_PIN);
	PWMChannelConfig(&htim5, TIM_CHANNEL_2, TIM5_CHANNEL2_PIN);
	PWMChannelConfig(&htim5, TIM_CHANNEL_3, TIM5_CHANNEL3_PIN);
	PWMChannelConfig(&htim5, TIM_CHANNEL_4, TIM5_CHANNEL4_PIN);

	PWMTimeBaseInit(&htim9, 19999, 167);
	PWMChannelConfig(&htim9, TIM_CHANNEL_1, TIM9_CHANNEL1_PIN);
	PWMChannelConfig(&htim9, TIM_CHANNEL_2, TIM9_CHANNEL2_PIN);


	BDCInit(&BDC1, &htim3, TIM_CHANNEL_4, SHIFTREG, &(SR.cast[1]), Bit6, Bit7);
	BDCInit(&BDC2, &htim3, TIM_CHANNEL_3, SHIFTREG, &(SR.cast[1]), Bit4, Bit5);
	BDCInit(&BDC3, &htim9, TIM_CHANNEL_1, SHIFTREG, &(SR.cast[1]), Bit2, Bit3);
	BDCInit(&BDC4, &htim9, TIM_CHANNEL_2, SHIFTREG, &(SR.cast[1]), Bit0, Bit1);
	BDCInit(&BDC5, &htim5, TIM_CHANNEL_4, SHIFTREG, &(SR.cast[0]), Bit6, Bit7);
	BDCInit(&BDC6, &htim5, TIM_CHANNEL_3, SHIFTREG, &(SR.cast[0]), Bit4, Bit5);
	BDCInit(&BDC7, &htim5, TIM_CHANNEL_2, SHIFTREG, &(SR.cast[0]), Bit2, Bit3);
	BDCInit(&BDC8, &htim5, TIM_CHANNEL_1, SHIFTREG, &(SR.cast[0]), Bit0, Bit1);

	//Laser
//	ADC_DMAxInit(&adc,&hadc2,&hdma_adc1,2);
//	ADC_Channel_Config(&adc,ADC_CHANNEL_10,IP16_Analog1_PIN);
//	ADC_Channel_Config(&adc,ADC_CHANNEL_11,IP17_Analog2_PIN);

//	VESCInit(31920, 7, 0.0037, VESC1, &vesc1);

//	SPIx_DMA_TX_Init(&hspi1, &hdma_spi1_tx, SPI1_NSS_PIN, SPI_MODE_MASTER);
//	GPIOPinsInit (SPI1_MISO_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
}



void CAN1_RX0_IRQHandler()
{
	HAL_CAN_IRQHandler(&hcan1);

}

void CAN2_RX1_IRQHandler()
{

	HAL_CAN_IRQHandler(&hcan2);

}


void CAN_PROCESS(PACKET_t packet_src){

	switch(packet_src){

	case VESC_PACKET:
		decode_VESC();
		break;

	case ODRIVE_PACKET:
		OdriveCAN_Handler();
		break;

	case RNS_PACKET:

		if(insData_receive[0] == 1){
			rns.RNS_data.common_instruction = insData_receive[1];
			insData_receive[0]=2;
		}
		if(insData_receive[0] == 17){
			if(buf2_flag == 1){
				rns.RNS_data.common_instruction = insData_receive[1];
				rns.RNS_data.common_buffer[0].data = buf1_receive[0].data;
				rns.RNS_data.common_buffer[1].data = buf1_receive[1].data;
				rns.RNS_data.common_buffer[2].data = buf2_receive[0].data;
				rns.RNS_data.common_buffer[3].data = buf2_receive[1].data;
				insData_receive[0]=3;
			}
		}
		break;
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	PACKET_t source;
	uint8_t aData[8];
	uint8_t rns_can = 1;

	if(hcan == &hcan1){
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0 , &CAN1RxMessage, aData);
		//	sprintf(data,"%d %d %d %d %d %d %d %d \r\n",aData[0],aData[1],aData[2],aData[3],aData[4],aData[5],aData[6],aData[7]);

		if(CAN1RxMessage.IDE == CAN_ID_EXT)
		{
			vescmsg.Rxmsg = CAN1RxMessage;
			memcpy(vescmsg.Data, aData,8);
			set_rx_frames(&vescmsg);
			source = VESC_PACKET;
		}else{
			source = RNS_PACKET;
			uint16_t id_mask = 0x1F;   // mask last 7 bits, get only the command
			uint16_t command = CAN1RxMessage.StdId & id_mask;
			uint16_t command_mask = 0x7E0;  // mask first 7 bits, get only the id
			uint16_t id = CAN1RxMessage.StdId&command_mask;
			id=id>>5;

			for( int i=0 ; i < number_of_odrive; i++)
			{
				if(id == P_to_Odrive[i]->Instance&&(command == GET_ENCODER_ESTIMATE || command == HEARTBEAT))
				{
					source = ODRIVE_PACKET;
					break;
				}
			}

			if(source  == ODRIVE_PACKET)
			{
				Odrvmsg.RXmsg = CAN1RxMessage;
				memcpy(Odrvmsg.Data,aData,CAN1RxMessage.DLC);
			}

			else{
				switch(CAN1RxMessage.StdId){
				case RNS_TO_mainboard:
					memcpy(&insData_receive, &aData, CAN1RxMessage.DLC);
					buf2_flag = 0;

					break;
				case RNS_TO_mainboard_buf1:
					memcpy(&buf1_receive, &aData, CAN1RxMessage.DLC);

					break;
				case RNS_TO_mainboard_buf2:
					memcpy(&buf2_receive, &aData, CAN1RxMessage.DLC);
					buf2_flag = 1;
					break;

				default:
					rns_can = 0;
					FaulHaber_Handler(aData);
					break;
				}
			}
		}
		if(rns_can)
			CAN_PROCESS(source);
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	}else{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0 , &CAN2RxMessage, aData);
		//sprintf(data,"%d %d %d %d %d %d %d %d \r\n",aData[0],aData[1],aData[2],aData[3],aData[4],aData[5],aData[6],aData[7]);
		if(CAN2RxMessage.IDE == CAN_ID_EXT)
		{
			vescmsg.Rxmsg = CAN2RxMessage;
			memcpy(vescmsg.Data, aData,8);
			set_rx_frames(&vescmsg);
			source = VESC_PACKET;
		}else{
			source = RNS_PACKET;
			uint16_t id_mask = 0x1F;   // mask last 7 bits, get only the command
			uint16_t command = CAN2RxMessage.StdId & id_mask;
			uint16_t command_mask = 0x7E0;  // mask first 7 bits, get only the id
			uint16_t id = CAN2RxMessage.StdId&command_mask;
			id=id>>5;

			for( int i=0 ; i < number_of_odrive; i++)
			{
				if(id == P_to_Odrive[i]->Instance&&(command == GET_ENCODER_ESTIMATE || command == HEARTBEAT))
				{
					source = ODRIVE_PACKET;
					break;
				}
			}

			if(source  == ODRIVE_PACKET)
			{
				Odrvmsg.RXmsg = CAN2RxMessage;
				memcpy(Odrvmsg.Data,aData,CAN2RxMessage.DLC);
			}

			else{
				switch(CAN2RxMessage.StdId){
				case RNS_TO_mainboard:
					memcpy(&insData_receive, &aData, CAN2RxMessage.DLC);
					buf2_flag = 0;

					break;
				case RNS_TO_mainboard_buf1:
					memcpy(&buf1_receive, &aData, CAN2RxMessage.DLC);

					break;
				case RNS_TO_mainboard_buf2:
					memcpy(&buf2_receive, &aData, CAN2RxMessage.DLC);
					buf2_flag = 1;
					break;

				default:
					rns_can = 0;
					FaulHaber_Handler(aData);
					break;
				}
			}
		}
		if(rns_can)
			CAN_PROCESS(source);
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	PACKET_t source;
	uint8_t aData[8];
	uint8_t rns_can = 1;

	if(hcan == &hcan1){
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1 , &CAN1RxMessage, aData);
		//	sprintf(data,"%d %d %d %d %d %d %d %d \r\n",aData[0],aData[1],aData[2],aData[3],aData[4],aData[5],aData[6],aData[7]);

		if(CAN1RxMessage.IDE == CAN_ID_EXT)
		{
			vescmsg.Rxmsg = CAN1RxMessage;
			memcpy(vescmsg.Data, aData,8);
			set_rx_frames(&vescmsg);
			source = VESC_PACKET;
		}else{
			source = RNS_PACKET;
			uint16_t id_mask = 0x1F;   // mask last 7 bits, get only the command
			uint16_t command = CAN1RxMessage.StdId & id_mask;
			uint16_t command_mask = 0x7E0;  // mask first 7 bits, get only the id
			uint16_t id = CAN1RxMessage.StdId&command_mask;
			id=id>>5;

			for( int i=0 ; i < number_of_odrive; i++)
			{
				if(id == P_to_Odrive[i]->Instance&&(command == GET_ENCODER_ESTIMATE || command == HEARTBEAT))
				{
					source = ODRIVE_PACKET;
					break;
				}
			}

			if(source  == ODRIVE_PACKET)
			{
				Odrvmsg.RXmsg = CAN1RxMessage;
				memcpy(Odrvmsg.Data,aData,CAN1RxMessage.DLC);
			}

			else{
				switch(CAN1RxMessage.StdId){
				case RNS_TO_mainboard:
					memcpy(&insData_receive, &aData, CAN1RxMessage.DLC);
					buf2_flag = 0;

					break;
				case RNS_TO_mainboard_buf1:
					memcpy(&buf1_receive, &aData, CAN1RxMessage.DLC);

					break;
				case RNS_TO_mainboard_buf2:
					memcpy(&buf2_receive, &aData, CAN1RxMessage.DLC);
					buf2_flag = 1;
					break;

				default:
					rns_can = 0;
					FaulHaber_Handler(aData);
					break;
				}
			}
		}
		if(rns_can)
			CAN_PROCESS(source);
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
	}else{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1 , &CAN2RxMessage, aData);
		//sprintf(data,"%d %d %d %d %d %d %d %d \r\n",aData[0],aData[1],aData[2],aData[3],aData[4],aData[5],aData[6],aData[7]);
		if(CAN2RxMessage.IDE == CAN_ID_EXT)
		{
			vescmsg.Rxmsg = CAN2RxMessage;
			memcpy(vescmsg.Data, aData,8);
			set_rx_frames(&vescmsg);
			source = VESC_PACKET;
		}else{
			source = RNS_PACKET;
			uint16_t id_mask = 0x1F;   // mask last 7 bits, get only the command
			uint16_t command = CAN2RxMessage.StdId & id_mask;
			uint16_t command_mask = 0x7E0;  // mask first 7 bits, get only the id
			uint16_t id = CAN2RxMessage.StdId&command_mask;
			id=id>>5;

			for( int i=0 ; i < number_of_odrive; i++)
			{
				if(id == P_to_Odrive[i]->Instance&&(command == GET_ENCODER_ESTIMATE || command == HEARTBEAT))
				{
					source = ODRIVE_PACKET;
					break;
				}
			}

			if(source  == ODRIVE_PACKET)
			{
				Odrvmsg.RXmsg = CAN2RxMessage;
				memcpy(Odrvmsg.Data,aData,CAN2RxMessage.DLC);
			}

			else{
				switch(CAN2RxMessage.StdId){
				case RNS_TO_mainboard:
					memcpy(&insData_receive, &aData, CAN2RxMessage.DLC);
					buf2_flag = 0;

					break;
				case RNS_TO_mainboard_buf1:
					memcpy(&buf1_receive, &aData, CAN2RxMessage.DLC);

					break;
				case RNS_TO_mainboard_buf2:
					memcpy(&buf2_receive, &aData, CAN2RxMessage.DLC);
					buf2_flag = 1;
					break;

				default:
					rns_can = 0;
					FaulHaber_Handler(aData);
					break;
				}
			}
		}
		if(rns_can)
			CAN_PROCESS(source);
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
	}

}
