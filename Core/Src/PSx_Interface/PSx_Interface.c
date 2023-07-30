
/************************************************/
/*		 	 	Include Header	       		  	*/
/************************************************/
#include "PSx_Interface.h"

/************************************************/
/*		 	 	Variables	      	 		  	*/
/************************************************/
unsigned char PSxdata;

/************************************************/
/*		 	 	Functions		       		  	*/
/************************************************/

/*
 * Function Name		: PSxBTReceiveData
 * Function Description : This function is called to receive PSx data from arduino through UART.
 * Function Remarks		: Called in respective UART interrupt
 * Function Arguments	: *psxbt	,pointer to structure PSxBT_t
 * 						  huartx	,x can be 1 to 6.
 * Function Return		: None
 * Function Example		: PSxBTReceiveData(&ps3x, huart1);
 */
void PSxBTReceiveData(PSxBT_t *psxbt, PSx psx, UART_HandleTypeDef* huartx)
{
	HAL_UART_Receive(huartx, (uint8_t *)&PSxdata, 1, 1000);;
	psxbt->ReceiveBuffer[psxbt->x++] = PSxdata;
	psxbt->psx1 = psx;
}

/*
 * Function Name		: PSxConnect
 * Function Description : This function is called to receive data from ps4 in I2C callback
 * Function Remarks		: Called in respective I2c interrupt
 * Function Arguments	: *psxbt	,pointer to structure PSxBT_t
 * Function Return		: None
 * Function Example		: PSxConnect(&ps4);
 */
void PSxConnect(PSxBT_t* psxbt){
	HAL_I2C_Master_Receive_IT(psxbt->hi2cps4, 0x44 << 1 ,(unsigned char *)psxbt->ReceiveBuffer, 10);
	PSx_HandlerI2C(psxbt);
}

/*
 * Function Name		: PSxConnectDMA
 * Function Description : This function is called to receive data from ps4 in I2C callback using DMA
 * Function Remarks		: Called in respective I2c interrupt
 * Function Arguments	: *psxbt	,pointer to structure PSxBT_t
 * Function Return		: None
 * Function Example		: PSxConnectDMA(&ps4);
 */
void PSxConnectDMA(PSxBT_t* psxbt){
	HAL_I2C_Master_Receive_DMA(psxbt->hi2cps4, 0x44 << 1 ,(unsigned char *)psxbt->ReceiveBuffer, 11);
	PSx_HandlerI2C(psxbt);
}

/*
 * Function Name		: PSxConnectUART
 * Function Description : This function is called to receive data from ps4 in UART callback
 * Function Remarks		: Called in respective UART interrupt
 * Function Arguments	: *psxbt	,pointer to structure PSxBT_t
 * Function Return		: None
 * Function Example		: PSxConnectUART(&ps4);
 */
void PSxConnectUART(PSxBT_t* psxbt){
	HAL_UART_Receive_IT(psxbt->huartps4 ,(unsigned char *)psxbt->ReceiveBuffer, 13);
	PSx_HandlerUART(psxbt);
}

/*
 * Function Name		: PSxReconnect
 * Function Description : This function is called to reconnect the ps4 if it's disconnected
 * Function Remarks		: Called in respective Interrupt
 * Function Arguments	: *psxbt	,pointer to structure PSxBT_t
 * Function Return		: None
 * Function Example		: PSxReconnect(&ps4);
 */
void PSxReconnect(PSxBT_t* psxbt){
	if(HAL_I2C_GetState(psxbt->hi2cps4) == HAL_I2C_STATE_READY){
		PSxConnect(psxbt);
	}
}

/*
 * Function Name		: PSxInit
 * Function Description : This function is called to initialize PSx.
 * Function Arguments	: *psxbt	, Pointer to structure PSxBT_t
 * 						   hi2cx    , Pointer to I2C handle
 * Function Return		: None
 * Function Example		: PSxInit(&ps4,&hi2c1);
 */
void PSxInit(PSxBT_t *psxbt,I2C_HandleTypeDef* hi2cx){
	psxbt->hi2cps4 = hi2cx;
	HAL_I2C_Master_Receive_IT(psxbt->hi2cps4, 0x44 << 1 ,(unsigned char *)psxbt->ReceiveBuffer, 10);
}

/*
 * Function Name		: PSxInitDMA
 * Function Description : This function is called to initialize PSx in DMA Mode.
 * Function Arguments	: *psxbt	, Pointer to structure PSxBT_t
 * 						   hi2cx    , Pointer to I2C handle
 * Function Return		: None
 * Function Example		: PSxInitDMA(&ps4,&hi2c1);
 */
void PSxInitDMA(PSxBT_t *psxbt,I2C_HandleTypeDef* hi2cx){
	psxbt->hi2cps4 = hi2cx;
//	HAL_I2C_Slave_Receive_DMA(hi2cx, psxbt->ReceiveBuffer, 11);//GG
	HAL_I2C_Master_Receive_DMA(psxbt->hi2cps4, 0x44 << 1 ,(unsigned char *)psxbt->ReceiveBuffer, 11);
	psxbt->initialized = 1;
	psxbt->disconnected = 0;
}

/*
 * Function Name		: PSxInitDMA
 * Function Description : This function is called to initialize PSx in UART MODE.
 * Function Arguments	: *psxbt	, Pointer to structure PSxBT_t
 * 						   hi2cx    , Pointer to I2C handle
 * Function Return		: None
 * Function Example		: PSxInitDMA(&ps4,&hi2c1);
 */
void PSxInitUART(PSxBT_t *psxbt,UART_HandleTypeDef* huartx){
	psxbt->huartps4 = huartx;
	HAL_UART_Receive_IT(psxbt->huartps4, (unsigned char *)psxbt->ReceiveBuffer, 1);
}

/*
 * Function Name		: PSx_HandlerUART
 * Function Description : This function is called to handle the received PSx data from UART.
 * Function Remarks		: Called after PSxBTReceiveData function
 * Function Arguments	: *psxbt	,pointer to structure PSxBT_t
 * Function Return		: None
 * Function Example		: PSx_HandlerUART(&ps3x);
 */
void PSx_HandlerUART_Legacy(PSxBT_t *psxbt){
	if(PSxdata == '\n'){
		for (int i = psxbt->x; i < PSxBTMAXLEN; i++){
			psxbt->ReceiveBuffer[i] = '\0';
		}
		psxbt->x = 0;

		if(psxbt->psx1 == PS4)
			sscanf(psxbt->ReceiveBuffer,"%u %u %u %d %d %d %d %d %d",(unsigned int *)&psxbt->buf1, (unsigned int *)&psxbt->buf2, (unsigned int *)&psxbt->buf3, &psxbt->leftjoy_x, &psxbt->leftjoy_y, &psxbt->rightjoy_x, &psxbt->rightjoy_y, &psxbt->an_L2, &psxbt->an_R2);
		else if(psxbt->psx1 == PS3)
			sscanf(psxbt->ReceiveBuffer,"%u %d %d %d %d %d %d", &psxbt->button, &psxbt->leftjoy_x, &psxbt->leftjoy_y, &psxbt->rightjoy_x, &psxbt->rightjoy_y, &psxbt->an_L2, &psxbt->an_R2);

		PSxBTGetXY(psxbt);
	}
}
/*
 * Function Name		: PSx_HandlerI2C
 * Function Description : This function is called to handle the received PSx data from I2C.
 * Function Remarks		: None
 * Function Arguments	: *psxbt	,pointer to structure PSxBT_t
 * Function Return		: None
 * Function Example		: PSx_HandlerI2C(&ps4x);
 */

void PSx_HandlerI2C(PSxBT_t *psxbt){
	psxbt->buf1 = psxbt->ReceiveBuffer[1];
	psxbt->buf2 = psxbt->ReceiveBuffer[2];
	psxbt->leftjoy_x = psxbt->ReceiveBuffer[3];
	psxbt->leftjoy_y = psxbt->ReceiveBuffer[4];
	psxbt->rightjoy_x = psxbt->ReceiveBuffer[5];
	psxbt->rightjoy_y = psxbt->ReceiveBuffer[6];
	psxbt->an_L2 = psxbt->ReceiveBuffer[7];
	psxbt->an_R2 = psxbt->ReceiveBuffer[8];
	psxbt->buf3 = psxbt->ReceiveBuffer[9];
	psxbt->state = psxbt->ReceiveBuffer[10];
	PSxBTGetXY(psxbt);
}


/*
 * Function Name		: PSx_HandlerUART
 * Function Description : This function is called to handle the received PSx data from UART.
 * Function Remarks		: None
 * Function Arguments	: *psxbt	,pointer to structure PSxBT_t
 * Function Return		: None
 * Function Example		: PSx_HandlerUART(&ps4x);
 */

void PSx_HandlerUART(PSxBT_t *psxbt){
	static uint8_t state = 0;
	switch(state){
	case 0:
		if(psxbt->ReceiveBuffer[0] == 0x01)
			state = 1;
		HAL_UART_Receive_IT(psxbt->huartps4, (unsigned char *)&psxbt->ReceiveBuffer[1], 1);
	break;
	case 1:
		if(psxbt->ReceiveBuffer[1] == 0x02){
			state = 2;
			HAL_UART_Receive_IT(psxbt->huartps4, (unsigned char *)&psxbt->ReceiveBuffer[2], 11);
		}else{
			state = 0;
			HAL_UART_Receive_IT(psxbt->huartps4, (unsigned char *)psxbt->ReceiveBuffer, 1);
		}
	break;
	case 2:
		if(psxbt->ReceiveBuffer[11] == 0x03 && psxbt->ReceiveBuffer[12] == 0x04){
			psxbt->buf1 = psxbt->ReceiveBuffer[2];
			psxbt->buf2 = psxbt->ReceiveBuffer[3];
			psxbt->leftjoy_x = psxbt->ReceiveBuffer[4];
			psxbt->leftjoy_y = psxbt->ReceiveBuffer[5];
			psxbt->rightjoy_x = psxbt->ReceiveBuffer[6];
			psxbt->rightjoy_y = psxbt->ReceiveBuffer[7];
			psxbt->an_L2 = psxbt->ReceiveBuffer[8];
			psxbt->an_R2 = psxbt->ReceiveBuffer[9];
			psxbt->buf3 = psxbt->ReceiveBuffer[10];
			PSxBTGetXY(psxbt);
		}
		state = 0;
		HAL_UART_Receive_IT(psxbt->huartps4, (unsigned char *)psxbt->ReceiveBuffer, 1);
	}
}

/*
 * Function Name		: PSxBTGetXY
 * Function Description : This function is called to normalise raw data.
 * Function Remarks		: None
 * Function Arguments	: *psxbt	,pointer to structure PSxBT_t
 * Function Return		: None
 * Function Example		: PSxBTGetXY(&ps3x);
 */

void PSxBTGetXY(PSxBT_t *psxbt)
{
	if(joyR_up){
		psxbt->joyR_y  = ((psx_low_Ry - ((float)psxbt->rightjoy_y))/100.0)/1.00;
		if(psxbt->joyR_y < 0.0){
			psxbt->joyR_y = 0.0;
		}else if(psxbt->joyR_y > 1.0){
			psxbt->joyR_y = 1.0;
		}
	}else if(joyR_down){
		psxbt->joyR_y = ((psx_high_Ry - ((float)psxbt->rightjoy_y))/100.0)/1.00;
		if(psxbt->joyR_y > 0.0){
			psxbt->joyR_y = 0.0;
		}else if(psxbt->joyR_y < -1.0){
			psxbt->joyR_y = -1.0;
		}
	}else{
		psxbt->joyR_y = 0.0;
	}

	if(joyR_left){
		psxbt->joyR_x = ((psx_low_Rx - ((float)psxbt->rightjoy_x))/100.0)/1.00;
		if(psxbt->joyR_x < 0.0){
			psxbt->joyR_x = 0.0;
		}else if(psxbt->joyR_x > 1.0){
			psxbt->joyR_x = 1.0;
		}
	}else if(joyR_right){
		psxbt->joyR_x = ((psx_high_Rx - ((float)psxbt->rightjoy_x))/100.0)/1.00;
		if(psxbt->joyR_x > 0.0){
			psxbt->joyR_x = 0.0;
		}else if(psxbt->joyR_x < -1.0){
			psxbt->joyR_x = -1.0;
		}
	}else{
		psxbt->joyR_x = 0.0;
	}

	if(joyL_up){
		psxbt->joyL_y = ((psx_low_Ly - ((float)psxbt->leftjoy_y))/100.0)/1.00;
		if(psxbt->joyL_y < 0.0){
			psxbt->joyL_y = 0.0;
		}else if(psxbt->joyL_y > 1.0){
			psxbt->joyL_y = 1.0;
		}
	}else if(joyL_down){
		psxbt->joyL_y = ((psx_high_Ly - ((float)psxbt->leftjoy_y))/100.0)/1.00;
		if(psxbt->joyL_y > 0.0){
			psxbt->joyL_y = 0.0;
		}else if(psxbt->joyL_y < -1.0){
			psxbt->joyL_y = -1.0;
		}
	}else{
		psxbt->joyL_y = 0.0;
	}

	if(joyL_left){
		psxbt->joyL_x = ((psx_low_Lx - ((float)psxbt->leftjoy_x))/100.0)/1.00;
		if(psxbt->joyL_x < 0.0){
			psxbt->joyL_x = 0.0;
		}else if(psxbt->joyL_x > 1.0){
			psxbt->joyL_x = 1.0;
		}
	}else if(joyL_right){
		psxbt->joyL_x = ((psx_high_Lx - ((float)psxbt->leftjoy_x))/100.0)/1.00;
		if(psxbt->joyL_x > 0.0){
			psxbt->joyL_x = 0.0;
		}else if(psxbt->joyL_x < -1.0){
			psxbt->joyL_x = -1.0;
		}
	}else{
		psxbt->joyL_x = 0.0;
	}

	psxbt->joyR_2 = (psxbt->an_R2/255.0);
	psxbt->joyL_2 = (psxbt->an_L2/255.0);
}

