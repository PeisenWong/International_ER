
/************************************************/
/*		 	 	Include Header	       		  	*/
/************************************************/
#include "can.h"
#include "../common.h"
/************************************************/
/*		 	 	Variables	      	 		  	*/
/************************************************/

/************************************************/
/*		 	 	Functions		       		  	*/
/************************************************/
/* Function Name		: CANx_Init
 * Function Description : This function is called to initialise bxCAN.
 * Function Remarks		:
 * Function Arguments	:
 *
 *						  hcanx                         Pointer to CAN handle
 * 						  CAN_FilterFIFOAssignment		CAN_Filter_FIFO0 or CAN_Filter_FIFO1
 * 						  CAN_FilterId_11bits			Filter identification number(0...0x7FF)
 * 						  CAN_FilterMaskId_11bits		Filter mask number(0...0x7FF)
 * 						  CAN_FilterNumber				Filter number. There have 28 filter banks shared between
 * 						  								CAN1 and CAN2. CAN_FilterNumber for CAN1 must less
 * 						  		z						than CAN2 since initialise CAN2 must call function
 * 						  								CAN_SlaveStartBank which cause the onward filter numbers
 * 						  								cannot be used in CAN1. For example: if 24 is set as
 * 						  								CAN_FilterNumber in initialise CAN2, the filter number which
 * 						  								can only be chosen as CAN_FilterNumber in initialise CAN1
 * 						  								is 0 to 23.
 * 						  CAN_Prescaler					Specifies the length of a time quantum.It ranges from 1 to 1024.
 * 						  								CAN Baudrate = APB1_CLK/
 * 						  											  ((CAN_SJW_tq+CAN_BS1_tq+CAN_BS2_tq)*CAN_Prescaler);
 * 						  								Example: CAN Baudrate = 42000000/((1+12+8)*2);
 * 						  N_PPriority					Preemption priority for the IRQ channel. A lower priority value
 * 						  								indicates a higher priority.
 * 						  N_SPriority					Subpriority level for the IRQ channel. A lower priority value
 * 						  								indicates a higher priority.
 * Function Return		: CAN_InitStatus				Constant indicates initialization succeed which will be
  *        											    CAN_InitStatus_Failed or CAN_InitStatus_Success.
 * Function Example		: CAN1_Init(&hcan1,4,CAN_Filter_FIFO0,0,0,13,1,1);
 */
void CANxInit(CAN_HandleTypeDef* hcanx,uint16_t prescaler,uint32_t FilterFIFOAssignment,uint16_t CAN_FilterId_11bits,
		uint16_t CAN_FilterMaskId_11bits,uint8_t CAN_FilterNumber, uint8_t rate){

	CAN_FilterTypeDef sFilterConfig = {0};

	if(hcanx == &hcan1){
		hcanx->Instance = CAN1;
	}else{
		hcanx->Instance = CAN2;
	}


	hcanx->Init.Prescaler = prescaler;
	hcanx->Init.Mode = CAN_MODE_NORMAL;

	hcanx->Init.SyncJumpWidth = CAN_SJW_1TQ;
	if(rate == CAN_1MHz){
		//sampling point 85.71%
		hcanx->Init.TimeSeg1 = CAN_BS1_11TQ;
		hcanx->Init.TimeSeg2 = CAN_BS2_2TQ;
	}else if(rate == CAN_500KHz){
		//sampling point 71.42%
		hcanx->Init.TimeSeg1 = CAN_BS1_14TQ;
		hcanx->Init.TimeSeg2 = CAN_BS2_6TQ;
	}
	hcanx->Init.TimeTriggeredMode = DISABLE;
	hcanx->Init.AutoBusOff = ENABLE;
	hcanx->Init.AutoWakeUp = DISABLE;
	hcanx->Init.AutoRetransmission = DISABLE;
	hcanx->Init.ReceiveFifoLocked = DISABLE;
	hcanx->Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(hcanx) != HAL_OK)
	{
		Error_Handler();
	}

	sFilterConfig.SlaveStartFilterBank = 14;
	sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterFIFOAssignment=FilterFIFOAssignment; //set fifo assignment
	sFilterConfig.FilterIdHigh= CAN_FilterId_11bits <<5; //the ID that the filter looks for (switch this for the other microcontroller)
	sFilterConfig.FilterIdLow=0;
	sFilterConfig.FilterMaskIdHigh=CAN_FilterMaskId_11bits <<5;
	sFilterConfig.FilterMaskIdLow=0;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
	sFilterConfig.FilterBank = CAN_FilterNumber;

	HAL_CAN_ConfigFilter(hcanx, &sFilterConfig);



	if(hcanx == &hcan1){

		if(FilterFIFOAssignment ==  CAN_FILTER_FIFO0){
			/* CAN1_RX0_IRQn interrupt configuration */
			HAL_NVIC_SetPriority(CAN1_RX0_IRQn, CAN1_FIFO1_IRQ_PRIO, 0);
			HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
			HAL_CAN_ActivateNotification(hcanx, CAN_IT_RX_FIFO0_MSG_PENDING);

		}else{
			/* CAN1_RX1_IRQn interrupt configuration */
			HAL_NVIC_SetPriority(CAN1_RX1_IRQn, CAN1_FIFO1_IRQ_PRIO, 0);
			HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
			HAL_CAN_ActivateNotification(hcanx, CAN_IT_RX_FIFO1_MSG_PENDING);
		}
	}else{

		if(FilterFIFOAssignment ==  CAN_FILTER_FIFO0){
			/* CAN2_RX0_IRQn interrupt configuration */
			HAL_NVIC_SetPriority(CAN2_RX0_IRQn, CAN2_FIFO0_IRQ_PRIO, 0);
			HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
			HAL_CAN_ActivateNotification(hcanx, CAN_IT_RX_FIFO0_MSG_PENDING);
		}else{
			/* CAN2_RX1_IRQn interrupt configuration */
			HAL_NVIC_SetPriority(CAN2_RX1_IRQn, CAN2_FIFO1_IRQ_PRIO, 0);
			HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
			HAL_CAN_ActivateNotification(hcanx, CAN_IT_RX_FIFO1_MSG_PENDING);
		}
	}

	HAL_CAN_Start(hcanx);

}



/* Function Name		: CAN_TxMsg
 * Function Description : This function is called to transmit data with bxCAN.
 * Function Remarks		:
 * Function Arguments	: hcanx				hcanx can be hcan1 or hcan2
 * 						  StdId_11bits		The standard identifier for the CAN frame message. This parameter
 * 						  					can be a value between 0 to 0x7FF.
 * 						  Msg				a pointer used to store data to CAN frame
 * 						  len				length of data
 * Function Return		: canmailbox		The number of the mailbox that is used for transmission or
  *         								CAN_TxStatus_NoMailBox if there is no empty mailbox.
 * Function Example		: CAN_TxMsg(&hcan1,1,&data,1);
 */
uint32_t CAN_TxMsg(CAN_HandleTypeDef* hcanx,uint32_t StdId_11bits,uint8_t *Msg,uint8_t len)
{
//	sys.rns_busy = 1;
	while(HAL_CAN_GetTxMailboxesFreeLevel(hcanx) == 0);
	CAN_TxHeaderTypeDef TxMessage;
	uint32_t canMailbox;
	uint8_t* buf = Msg;
	uint16_t i=0,datalen=len;
	uint8_t Txmsg[8];

	TxMessage.StdId=StdId_11bits;					 // standard identifier=0
	TxMessage.ExtId=0;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;
	TxMessage.TransmitGlobalTime = DISABLE;  // the type of frame for the message that will be transmitted

	while(datalen--)
	{
		Txmsg[i++]= *(uint8_t*)buf++;
//		if(i == 8){
//			TxMessage.DLC=8;
//			if (HAL_CAN_AddTxMessage(hcanx, &TxMessage, Txmsg, &canMailbox) != HAL_OK) {
//				Error_Handler();
//			}
//		}
	}
	if(i>0){
		TxMessage.DLC = i;
		if (HAL_CAN_AddTxMessage(hcanx, &TxMessage, Txmsg, &canMailbox) != HAL_OK) {
			Error_Handler();
		}

	}
//	sys.rns_busy = 0;

	return(canMailbox);
}



/* Function Name		: CAN_TxMsg
 * Function Description : This function is called to transmit data with bxCAN.
 * Function Remarks		:
 * Function Arguments	: hcanx				hcanx can be hcan1 or hcan2
 * 						  StdId_11bits		The standard identifier for the CAN frame message. This parameter
 * 						  					can be a value between 0 to 0x7FF.
 * 						  Msg				a pointer used to store data to CAN frame
 * 						  len				length of data
 * Function Return		: canmailbox		The number of the mailbox that is used for transmission or
  *         								CAN_TxStatus_NoMailBox if there is no empty mailbox.
 * Function Example		: CAN_TxMsg(&hcan1,1,&data,1);
 */
uint32_t CAN_TxMsgEID(CAN_HandleTypeDef* hcanx,uint32_t EID,uint8_t *Msg,uint8_t len)
{


	while(HAL_CAN_GetTxMailboxesFreeLevel(hcanx) == 0);
	CAN_TxHeaderTypeDef TxMessage;
	uint32_t canMailbox;
	uint8_t* buf = Msg;
	uint16_t i=0,datalen=len;
	uint8_t Txmsg[8];

	TxMessage.StdId=0;					 // standard identifier=0
	TxMessage.ExtId=EID;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Extended;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;
	TxMessage.TransmitGlobalTime = DISABLE;  // the type of frame for the message that will be transmitted

	while(datalen--)
	{
		Txmsg[i++]= *(uint8_t*)buf++;
		if(i == 8){
			TxMessage.DLC=8;
			if (HAL_CAN_AddTxMessage(hcanx, &TxMessage, Txmsg, &canMailbox) != HAL_OK) {
				Error_Handler();
			}
		}
	}
	if(i>0){
		TxMessage.DLC = i;
		if (HAL_CAN_AddTxMessage(hcanx, &TxMessage, Txmsg, &canMailbox) != HAL_OK) {
			Error_Handler();
		}

	}

	return(canMailbox);
}

uint32_t CAN_OPEN_TxMsg(CAN_HandleTypeDef* hcanx, uint8_t FunctionCode, uint8_t NodeID, uint8_t *Msg,uint8_t len)
{


	while(HAL_CAN_GetTxMailboxesFreeLevel(hcanx) == 0);
	CAN_TxHeaderTypeDef TxMessage;
	uint32_t canMailbox;
	uint8_t* buf = Msg;
	uint16_t i=0,datalen=len;
	uint8_t Txmsg[8];

	TxMessage.StdId= ( (FunctionCode & 0x0F) << 7 ) | ( NodeID & 0x7F );// standard identifier=0
	TxMessage.ExtId=0;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;
	TxMessage.TransmitGlobalTime = DISABLE;  // the type of frame for the message that will be transmitted

	while(datalen--)
	{
		Txmsg[i++]= *(uint8_t*)buf++;
//		if(i == 8){
//			TxMessage.DLC=8;
//			if (HAL_CAN_AddTxMessage(hcanx, &TxMessage, Txmsg, &canMailbox) != HAL_OK) {
//				Error_Handler();
//			}
//		}
	}
	if(i>0){
		TxMessage.DLC = i;
		if (HAL_CAN_AddTxMessage(hcanx, &TxMessage, Txmsg, &canMailbox) != HAL_OK) {
			Error_Handler();
		}

	}

	return(canMailbox);
}

uint32_t CAN_TxRTR(CAN_HandleTypeDef* hcanx,uint32_t StdId_11bits)
{
	while(HAL_CAN_GetTxMailboxesFreeLevel(hcanx) == 0);
	CAN_TxHeaderTypeDef TxMessage;
	uint32_t canMailbox;

	TxMessage.StdId=StdId_11bits;					 // standard identifier=0
	TxMessage.ExtId=0;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_REMOTE;
	TxMessage.TransmitGlobalTime = DISABLE;  // the type of frame for the message that will be transmitted
	TxMessage.DLC = 0;

	if (HAL_CAN_AddTxMessage(hcanx, &TxMessage, NULL, &canMailbox) != HAL_OK) {
		Error_Handler();
	}


	return(canMailbox);
}

uint32_t CAN_Open_TxSync(CAN_HandleTypeDef* hcanx)
{
	while(HAL_CAN_GetTxMailboxesFreeLevel(hcanx) == 0);
	CAN_TxHeaderTypeDef TxMessage;
	uint32_t canMailbox;

	TxMessage.StdId=0x80;					 // standard identifier=0
	TxMessage.ExtId=0;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;
	TxMessage.TransmitGlobalTime = DISABLE;  // the type of frame for the message that will be transmitted
	TxMessage.DLC = 0;

	if (HAL_CAN_AddTxMessage(hcanx, &TxMessage, NULL, &canMailbox) != HAL_OK) {
		Error_Handler();
	}


	return(canMailbox);
}
