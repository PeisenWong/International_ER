/*
 * TuningInterface.c
 *
 *  Created on: Mar 18, 2022
 *      Author: lokcharming
 */
#include "TuningInterface.h"

static int numInt[TOTAL_PAGE] = {
		NUM_INT_TUNE_LIST0,
		NUM_INT_TUNE_LIST1,
		NUM_INT_TUNE_LIST2,
		NUM_INT_TUNE_LIST3,
		NUM_INT_TUNE_LIST4
};

static int numFloat[TOTAL_PAGE] = {
		NUM_FLOAT_TUNE_LIST0,
		NUM_FLOAT_TUNE_LIST1,
		NUM_FLOAT_TUNE_LIST2,
		NUM_FLOAT_TUNE_LIST3,
		NUM_FLOAT_TUNE_LIST4
};

static Tune_Int_t* intList[TOTAL_PAGE] = {
		TuneIntList0,
		TuneIntList1,
		TuneIntList2,
		TuneIntList3,
		TuneIntList4
};

static Tune_Float_t* floatList[TOTAL_PAGE] = {
		TuneFloatList0,
		TuneFloatList1,
		TuneFloatList2,
		TuneFloatList3,
		TuneFloatList4
};

static uint32_t intAddress[TOTAL_PAGE]={
		SECTOR_6_FOR_PAGE_0,
		SECTOR_6_FOR_PAGE_1,
		SECTOR_6_FOR_PAGE_2,
		SECTOR_6_FOR_PAGE_3,
		SECTOR_6_FOR_PAGE_4
};

static uint32_t floatAddress[TOTAL_PAGE]={
		SECTOR_7_FOR_PAGE_0,
		SECTOR_7_FOR_PAGE_1,
		SECTOR_7_FOR_PAGE_2,
		SECTOR_7_FOR_PAGE_3,
		SECTOR_7_FOR_PAGE_4
};

void tuneRegister(int numPage, UART_HandleTypeDef* huartx){
	tunePage = numPage;
	tuneUart = huartx;

	for(int i=0; i<tunePage; i++){
		tuneWriteSuccess += numInt[i];
		tuneWriteSuccess += numFloat[i];
	}
}

void tunePending(void){
	HAL_UART_Receive_DMA(tuneUart, (uint8_t *)&tuneCommand, 1);
}

void tuneInit(TUNE_PAGE page){
	//[NUM_INT_TUNE][NUM_FLOAT_TUNE]
	//[0x52][0x01][strlen][str...][*curValue...][min ...][max ...][0x31][0x40] format of INT
	//[0x14][0x01][strlen][str...][*curValue...][minF...][maxF...][0x20][0x00] format of FLOAT


	uint8_t numVar[2] = {numInt[page], numFloat[page]};

	HAL_UART_Transmit(tuneUart, numVar, 2, 10);
	uint8_t sendBuf[100];
	for(int i=0; i<numInt[page]; i++){
		sendBuf[0] = 0x52;
		sendBuf[1] = 0x01;
		uint8_t strLen = strlen(intList[page][i].varName);
		sendBuf[2] = strLen;
		uint8_t len = 1+1+1+strLen+4+4+4+1+1;

		sendBuf[len-2] = 0x31; sendBuf[len-1] = 0x40;
		memcpy(&sendBuf[3], intList[page][i].varName, strLen);
		memcpy(&sendBuf[3+strLen], intList[page][i].ptr, 4);
		memcpy(&sendBuf[3+strLen+4], &intList[page][i].min, 4);
		memcpy(&sendBuf[3+strLen+8], &intList[page][i].max, 4);
		HAL_UART_Transmit(tuneUart, sendBuf, len, 1000);
	}

	for(int i=0; i<numFloat[page]; i++){
		sendBuf[0] = 0x14;
		sendBuf[1] = 0x01;
		uint8_t strLen = strlen(floatList[page][i].varName);
		sendBuf[2] = strLen;
		uint8_t len = 3+strLen+4+4+4+1+1;

		sendBuf[len-2] = 0x20; sendBuf[len-1] = 0x00;
		memcpy(&sendBuf[3], floatList[page][i].varName, strLen);
		memcpy(&sendBuf[3+strLen], floatList[page][i].ptr, 4);
		memcpy(&sendBuf[3+strLen+4], &floatList[page][i].min, 4);
		memcpy(&sendBuf[3+strLen+8], &floatList[page][i].max, 4);

		HAL_UART_Transmit(tuneUart, sendBuf, len, 100);
	}
	tunePending();
}

void tuneUpdate(TUNE_PAGE page){
	//[0x52][0x01][index][*curValue...][0x31][0x40] format of INT
	//[0x14][0x01][index][*curValue...][0x20][0x00] format of FLOAT

	for(int i=0; i<numInt[page]; i++){
		int len = 1+1+1+4+1+1;
		uint8_t sendBuf[len];
		sendBuf[0] = 0x52; sendBuf[1] = 0x01; sendBuf[len-2] = 0x31; sendBuf[len-1] = 0x40;
		sendBuf[2] = i;
		memcpy(&sendBuf[3], intList[page][i].ptr, 4);
		HAL_UART_Transmit(tuneUart, sendBuf, len, 100);
	}

	for(int i=0; i<numFloat[page]; i++){
		int len = 1+1+1+4+1+1;
		uint8_t sendBuf[len];
		sendBuf[0] = 0x14; sendBuf[1] = 0x01; sendBuf[len-2] = 0x20; sendBuf[len-1] = 0x00;
		sendBuf[2] = i;
		memcpy(&sendBuf[3], floatList[page][i].ptr, 4);
		HAL_UART_Transmit(tuneUart, sendBuf, len, 100);
	}

	tunePending();
}

void tuneEdit(TUNE_PAGE page){
	//[0x52][0x01][index][editValue...][0x31][0x40] format of INT
	//[0x14][0x01][index][editValue...][0x20][0x00] format of FLOAT
	//[0x88][0x77] terminate, back to tune pending
	static uint8_t state = 0;

	switch(state){
	case 0://When first received tuneCommand == 'e'
		HAL_UART_Receive_DMA(tuneUart, &tuneHeader, 1);
		state = 1;
		break;
	case 1:
		if(tuneHeader == 0x52 || tuneHeader == 0x14 || tuneHeader == 0x88){
			state = 2;
			HAL_UART_Receive_DMA(tuneUart, &tuneHeader1, 1);
		}
		else{
			HAL_UART_Receive_DMA(tuneUart, &tuneHeader, 1);
			state = 1;
		}
		break;
	case 2:
		if(tuneHeader1 == 0x01){
			HAL_UART_Receive_DMA(tuneUart, tuneBuffer, 7);
			state = 3;
		}
		else if(tuneHeader1 == 0x77){
			tune_flag.editTerminated = 1;
			tunePending();
			state = 0;
		}
		else{
			HAL_UART_Receive_DMA(tuneUart, &tuneHeader, 1);
			state = 1;
		}
		break;
	case 3:
		if(tuneBuffer[5] == 0x31 && tuneBuffer[6] == 0x40){
			*((int *)(intList[page][tuneBuffer[0]].ptr)) = *((int *)&tuneBuffer[1]);
		}

		if(tuneBuffer[5] == 0x20 && tuneBuffer[6] == 0x00){
			*(floatList[page][tuneBuffer[0]].ptr) = *((float *)&tuneBuffer[1]);
		}
		tuneBuffer[5] = tuneBuffer[6] = 0;
		HAL_UART_Receive_DMA(tuneUart, &tuneHeader, 1);
		state = 1;
		break;
	}
}

uint32_t tuneWriteInt(void){
	uint32_t success=0;
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError;
	HAL_FLASH_Unlock();

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector        = FLASH_SECTOR_6;
	EraseInitStruct.NbSectors     = 1;
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	{
		return HAL_FLASH_GetError ();
	}
	for(int i=0; i<tunePage; i++){
		uint32_t Address = intAddress[i];
		for(int j=0; j<numInt[i]; j++){
			if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, *((int *)(intList[i][j].ptr)))==HAL_OK){
				success ++;
			}
			Address += 4;
		}
	}

	HAL_FLASH_Lock();
	return success;
}

uint32_t tuneWriteFloat(void){

	uint32_t success=0;
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError;
	HAL_FLASH_Unlock();

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector        = FLASH_SECTOR_7;
	EraseInitStruct.NbSectors     = 1;
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	{
		return HAL_FLASH_GetError ();
	}
	for(int i=0; i<tunePage; i++){
		uint32_t Address = floatAddress[i];
		for(int j=0; j<numFloat[i]; j++){
			uint32_t temp;
			memcpy(&temp, floatList[i][j].ptr, 4);
			if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, temp)==HAL_OK){
				success ++;
			}
			Address += 4;
		}
	}
	HAL_FLASH_Lock();
	return success;
}

void tuneRead(void){
	for(int page=0; page<tunePage; page++){
		uint32_t Address = intAddress[page];
		for(int i=0; i<numInt[page]; i++){
			memcpy(intList[page][i].ptr, (__IO uint32_t *)Address, 4);
			Address += 4;
		}
		Address = floatAddress[page];
		for(int i=0; i<numFloat[page]; i++){
			memcpy(floatList[page][i].ptr, (__IO uint32_t *)Address, 4);
			Address += 4;
		}
	}
}

void tunePID(UART_HandleTypeDef* huartx, PSxBT_t* ps4, RNS_interface_t* rns){
	static float target_speed = 3.0;

	if(ps4->button == R1){
		while(ps4->button == R1)
			target_speed += 1.0;
		if(target_speed > 6.0)
			target_speed = 6.0;
	}else if(ps4->button == L1){
		while(ps4->button == L1)
			target_speed -= 1.0;
		if(target_speed < 1.0)
			target_speed = 1.0;
	}

	if(ps4->button == UP){
		uint32_t tick = HAL_GetTick();
		float a, b, c, d;
		while(ps4->button == UP){
			RNSVelocity(target_speed, target_speed, target_speed, target_speed, rns);
			RNSEnquire(RNS_VEL_BOTH, rns);
			a = rns->enq.enq_buffer[0].data;
			b = rns->enq.enq_buffer[1].data;
			c = rns->enq.enq_buffer[2].data;
			d = rns->enq.enq_buffer[3].data;
			sprintf(data, "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
					target_speed, a, target_speed, b, target_speed, c, target_speed, d);
			if(HAL_GetTick()-tick >= 20){
				tick = HAL_GetTick();
				UARTPrintString(huartx, data);
			}
		}
		RNSStop(rns);
		memset(data, 0, 100);
	}else if(ps4->button == LEFT){
		uint32_t tick = HAL_GetTick();
		float a, b, c, d;
		while(ps4->button == LEFT){
			RNSVelocity(-target_speed, target_speed, target_speed, -target_speed, rns);
			RNSEnquire(RNS_VEL_BOTH, rns);
			a = rns->enq.enq_buffer[0].data;
			b = rns->enq.enq_buffer[1].data;
			c = rns->enq.enq_buffer[2].data;
			d = rns->enq.enq_buffer[3].data;
			sprintf(data, "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
					target_speed, -a, target_speed, b, target_speed, c, target_speed, -d);
			if(HAL_GetTick() - tick >= 20){
				tick = HAL_GetTick();
				UARTPrintString(huartx, data);
			}
		}
		RNSStop(rns);
		memset(data, 0, 100);
	}else if(ps4->button == DOWN){
		uint32_t tick = HAL_GetTick();
		float a, b, c, d;
		while(ps4->button == DOWN){
			RNSVelocity(-target_speed, -target_speed, -target_speed, -target_speed, rns);
			RNSEnquire(RNS_VEL_BOTH, rns);
			a = rns->enq.enq_buffer[0].data;
			b = rns->enq.enq_buffer[1].data;
			c = rns->enq.enq_buffer[2].data;
			d = rns->enq.enq_buffer[3].data;
			sprintf(data, "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
					target_speed, -a, target_speed, -b, target_speed, -c, target_speed, -d);
			if(HAL_GetTick() - tick >= 20){
				tick = HAL_GetTick();
				UARTPrintString(huartx, data);
			}
		}
		RNSStop(rns);
		memset(data, 0, 100);
	}else if(ps4->button == RIGHT){
		uint32_t tick = HAL_GetTick();
		float a, b, c, d;
		while(ps4->button == RIGHT){
			RNSVelocity(target_speed, -target_speed, -target_speed, target_speed, rns);
			RNSEnquire(RNS_VEL_BOTH, rns);
			a = rns->enq.enq_buffer[0].data;
			b = rns->enq.enq_buffer[1].data;
			c = rns->enq.enq_buffer[2].data;
			d = rns->enq.enq_buffer[3].data;
			sprintf(data, "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
					target_speed, a, target_speed, -b, target_speed, -c, target_speed, d);
			if(HAL_GetTick() - tick >= 20){
				tick = HAL_GetTick();
				UARTPrintString(huartx, data);
			}
		}
		RNSStop(rns);
		memset(data, 0, 100);
	}

	realMODN(ps4, rns);
	if(ps4->button == CROSS){
		while(ps4->button == CROSS);
		uint8_t terminatePlot = 0x15;
		HAL_UART_Transmit(huartx, &terminatePlot, 1, 1);
	}

	if(ps4->button == OPTION)
	{
		while(ps4->button == OPTION);
		sys.tunePid = 0;
		led3 = !led3;
	}
}

void tunePIDTri(UART_HandleTypeDef* huartx, PSxBT_t* ps4, RNS_interface_t* rns){
	static float target_speed = 3.0;

	if(ps4->button == R1){
		while(ps4->button == R1)
			target_speed += 1.0;
		if(target_speed > 6.0)
			target_speed = 6.0;
	}else if(ps4->button == L1){
		while(ps4->button == L1)
			target_speed -= 1.0;
		if(target_speed < 1.0)
			target_speed = 1.0;
	}

	if(ps4->button == UP){
		uint32_t tick = HAL_GetTick();
		float a, b, c;
		float ta = target_speed*0.866, tb = target_speed*0.866, tc = 0.0;
		while(ps4->button == UP){
			RNSVelocity(ta ,tb, tc, 0.0, rns);
			RNSEnquire(RNS_VEL_BOTH, rns);
			a = rns->enq.enq_buffer[0].data;
			b = rns->enq.enq_buffer[1].data;
			c = rns->enq.enq_buffer[2].data;
			sprintf(data, "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
					ta, a, tb, b, tc, c);
			if(HAL_GetTick()-tick >= 20){
				tick = HAL_GetTick();
				UARTPrintString(huartx, data);
			}
		}
		RNSStop(rns);
		memset(data, 0, 100);
	}else if(ps4->button == LEFT){
		uint32_t tick = HAL_GetTick();
		float a, b, c;
		float ta = -0.5*target_speed, tb = 0.5*target_speed, tc = target_speed;
		while(ps4->button == LEFT){
			RNSVelocity(ta ,tb, tc, 0.0, rns);
			RNSEnquire(RNS_VEL_BOTH, rns);
			a = rns->enq.enq_buffer[0].data;
			b = rns->enq.enq_buffer[1].data;
			c = rns->enq.enq_buffer[2].data;
			sprintf(data, "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
					-ta, -a, tb, b, tc, c);
			if(HAL_GetTick()-tick >= 20){
				tick = HAL_GetTick();
				UARTPrintString(huartx, data);
			}
		}
		RNSStop(rns);
		memset(data, 0, 100);
	}else if(ps4->button == DOWN){
		uint32_t tick = HAL_GetTick();
		float a, b, c;
		float ta = -target_speed, tb = -target_speed*0.866, tc = 0.0;
		while(ps4->button == DOWN){
			RNSVelocity(ta ,tb, tc, 0.0, rns);
			RNSEnquire(RNS_VEL_BOTH, rns);
			a = rns->enq.enq_buffer[0].data;
			b = rns->enq.enq_buffer[1].data;
			c = rns->enq.enq_buffer[2].data;
			sprintf(data, "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
					-ta, -a, -tb, -b, tc, c);
			if(HAL_GetTick()-tick >= 20){
				tick = HAL_GetTick();
				UARTPrintString(huartx, data);
			}
		}
		RNSStop(rns);
		memset(data, 0, 100);
	}else if(ps4->button == RIGHT){
		uint32_t tick = HAL_GetTick();
		float a, b, c;
		float ta = 0.5*target_speed, tb = -0.5*target_speed, tc = -target_speed;
		while(ps4->button == RIGHT){
			RNSVelocity(ta ,-tb, -tc, 0.0, rns);
			RNSEnquire(RNS_VEL_BOTH, rns);
			a = rns->enq.enq_buffer[0].data;
			b = rns->enq.enq_buffer[1].data;
			c = rns->enq.enq_buffer[2].data;
			sprintf(data, "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
					ta, a, -tb, -b, -tc, -c);
			if(HAL_GetTick()-tick >= 20){
				tick = HAL_GetTick();
				UARTPrintString(huartx, data);
			}
		}
		RNSStop(rns);
		memset(data, 0, 100);
	}

	realMODN(ps4, rns);
	if(ps4->button == CROSS){
		while(ps4->button == CROSS);
		uint8_t terminatePlot = 0x15;
		HAL_UART_Transmit(huartx, &terminatePlot, 1, 1);
	}
}
