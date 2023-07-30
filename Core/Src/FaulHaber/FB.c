/*
 * FB.c
 *
 *  Created on: Oct 7, 2022
 *      Author: Shaon
 */

#include "FB.h"
#include "../adapter.h"
/* don't know why called it FB instead of FH, but anyways*/

/*
 * Function Name		: 	FHInit
 * Function Description : 	Called to init FaulHaber motion controller, will not cause delay
 * Function Remarks		: 	offset is only used to reduce small errors after you reset encoder,
 * 						  	if error is very big, just reset encoder
 * Function Arguments	: 	*fh		, pointer to structure FaulHaber_t
 * 							*hcanx	, pointer to structure CAN_HandleTypeDef
 * 							ID		, node ID of motion controller
 * 							of		, offset of absolute encoder
 * 							vel		, Profile Velocity, refer to documentation
 * 							maxP	, max position of absolute encoder (only when in PP mode)
 * 							minP	, min position of absolute encoder (only when in PP mode)
 * Function Return		: 	None
 * Function Example		: 	FHInit(&fh1, &hcan1, 1, 0, 3000, 2147483647, -2147483647);
 */
void FHInit(FaulHaber_t *fh, CAN_HandleTypeDef* hcanx, uint8_t ID, int of, uint32_t vel, int maxP, int minP){

	static uint8_t mtrCnt=0;
	fh->nodeID=ID;
	fh->hcanx=hcanx;
	fh->mode=CNA;
	fh->offset=of;
	fh->brake=1;
	fh->maxV=vel;
	fh->maxPos=maxP;
	fh->minPos=minP;
	pfh[mtrCnt++]=fh;

}

/*
 * Function Name		: 	FH_WaitInit
 * Function Description : 	Called to init FaulHaber motion controller, will wait for confirmation
 * 							from motion controller
 * Function Remarks		: 	will wait for motion controller to send boot up message first,
 * 							better call it after all initialization
 * Function Arguments	: 	None
 * Function Return		: 	None
 * Function Example		: 	FH_WaitInit();
 */
void FH_WaitInit(){
	while(!PB1);
	uint8_t i=0;
	uint8_t temp[8]={ 0x01, 0, 0x60, 0, 0x06, 0, 0, 0};
	uint32_t timbuf;
//	while(pfh[i]!=0){
//		while(!pfh[i]->init);
//		temp[1]=pfh[i]->nodeID;
//		CAN_TxMsg(pfh[i]->hcanx, 0x00, temp, 2);	//activate PDO
//		i++;
//	}
//	HAL_Delay(1000);
//	temp[1]=pfh[i]->nodeID;
//	CAN_TxMsg(pfh[i]->hcanx, 0x00, temp, 2);	//activate PDO
//	i=1;
	while(pfh[i]!=0){		//have to wait for driver to enable PDO transmission first
		temp[1]=pfh[i]->nodeID;
		pfh[i]->PDOInit=0;
		timbuf=HAL_GetTick();
		CAN_TxMsg(pfh[i]->hcanx, 0x00, temp, 2);
		CAN_TxRTR(pfh[i]->hcanx, (PDO1Tx<<7) + pfh[i]->nodeID);
		while(!pfh[i]->PDOInit){
			if(HAL_GetTick()-timbuf>=5){
				CAN_TxMsg(pfh[i]->hcanx, 0x00, temp, 2);	//activate PDO
				CAN_TxRTR(pfh[i]->hcanx, (PDO1Tx<<7) + pfh[i]->nodeID);
				timbuf=HAL_GetTick();
			}
		}
		i++;
	}

	while(i>0){
		i--;
		FH_Start(pfh[i]);	//operation enable state
		FH_Config(pfh[i]);
		FH_Pvel(pfh[i], pfh[i]->maxV);	//set profile velocity
		pfh[i]->start=0;
		timbuf=HAL_GetTick();
		while(HAL_GetTick()-timbuf<3);
		pfh[i]->waiting=1;
		CAN_TxRTR(pfh[i]->hcanx, (PDO1Tx<<7) + pfh[i]->nodeID);
		while(pfh[i]->waiting);
		timbuf=HAL_GetTick();
		while(!pfh[i]->start){
			FH_Start(pfh[i]);
			while(HAL_GetTick()-timbuf<3);
			led2 = !led2;
			pfh[i]->waiting=1;
			CAN_TxRTR(pfh[i]->hcanx, (PDO1Tx<<7) + pfh[i]->nodeID);
			while(pfh[i]->waiting);
			timbuf=HAL_GetTick();
		}

	}


}

/*
 * Function Name		: 	FH_Config
 * Function Description : 	Called in FH_WaitInit() function
 * Function Remarks		: 	-
 * Function Arguments	: 	*fh	, pointer to structure FaulHaber_t
 * Function Return		: 	None
 * Function Example		: 	FH_Config(&fh1);
 */
void FH_Config(FaulHaber_t *fh){
	uint8_t temp[8]={0x2b, 0x5a, 0x60, 0x00, 0x06, 0, 0, 0};
	int num=2147483647;
	FH_SDO(fh, temp);	//quickStop uses Quicktop ramp
	temp[1]=0x3f;
	temp[2]=0x23;
	temp[4]=0x31;
	FH_SDO(fh, temp);	//PV mode not limited by position limit
	temp[0]=0x23;
	temp[1]=0x7b;
	temp[2]=0x60;
	temp[3]=0x02;
	memcpy(&temp[4], &num, 4);
	FH_SDO(fh, temp);	//max position limit
	temp[3]=0x01;
	num=-num;
	memcpy(&temp[4], &num, 4);
	FH_SDO(fh, temp);	//min position limit
	FH_Pos_Limit(fh, fh->maxPos, fh->minPos);
	temp[0]=0x23;
	temp[1]=0x65;
	temp[3]=0x00;
	temp[4]=temp[5]=temp[6]=temp[7]=0xff;
	FH_SDO(fh, temp);	//max following error

}

/*
 * Function Name		: 	FH_Start
 * Function Description : 	Called to set motion controller into Operation Enable State
 * Function Remarks		: 	When the motion controller senses an error, it will go into Fault State,
 * 							call this function to set it back into Operational Enable State
 * Function Arguments	: 	*fh	, pointer to structure FaulHaber_t
 * Function Return		: 	None
 * Function Example		: 	FH_Start(&fh1);
 */
void FH_Start(FaulHaber_t *fh){
	uint8_t temp[2]={0x06, 0x00};
	CAN_OPEN_TxMsg(fh->hcanx, PDO1Rx, fh->nodeID, temp, 2);		//shut down
	temp[0]=0x07;
	CAN_OPEN_TxMsg(fh->hcanx, PDO1Rx, fh->nodeID, temp, 2);		//switch on
	temp[0]=0x0f;
	CAN_OPEN_TxMsg(fh->hcanx, PDO1Rx, fh->nodeID, temp, 2);		//enable operation
}

/*
 * Function Name		: 	FHEnq
 * Function Description : 	Called to enquire PDOs
 * Function Remarks		: 	Not suitable to be used with RNS_Enquire. Refer to documentation for solution
 * Function Arguments	: 	*fh	, pointer to structure FaulHaber_t
 * 							par	, PDO to be enquired
 * Function Return		: 	None
 * Function Example		: 	FHEnq(&fh1, PDO2Tx);
 */
void FHEnq(FaulHaber_t *fh, FH_Fcode par){
	fh->waiting=1;
	CAN_TxRTR(fh->hcanx, (par<<7)+fh->nodeID);
	while(fh->waiting);
}

/*
 * Function Name		: 	FH_PosAbs
 * Function Description : 	Called to set absolute target position
 * Function Remarks		: 	unlike relative position, it overides the target position completely
 * Function Arguments	: 	*fh	, pointer to structure FaulHaber_t
 * 							pos	, value of absolute target position
 * Function Return		: 	None
 * Function Example		: 	FH_PosAbs(&fh1, 500000);
 */
void FH_PosAbs(FaulHaber_t *fh, int pos){
	FHmode(fh, PP);
	uint8_t temp[6]={0x0f, 0x00, 0, 0, 0, 0};
	CAN_OPEN_TxMsg(fh->hcanx, PDO1Rx, fh->nodeID, temp, 2);		//set bit 4 to LOW
	int posi=pos+fh->offset;
	temp[0]=0x3f;
	memcpy(&temp[2], &posi, 4);
	CAN_OPEN_TxMsg(fh->hcanx, PDO2Rx, fh->nodeID, temp, 6);
}

/*
 * Function Name		: 	FH_PosRel
 * Function Description : 	Called to set relative absolute position
 * Function Remarks		: 	This function adds/substracts the value of the target position,
 * 							Eg. If initial position = 0;
 * 							FH_PosRel(1000); FH_PosRel(-800)	=> target position=200
 * 							FH_PosAbs(1000); FH_PosRel(200)		=> target position=1200
 * 							FH_PosRel(200); FH_PosAbs(1000)		=> target position=1000
 * 							*The motion of motor is only based on the target position
 * Function Arguments	: 	*fh	, pointer to structure FaulHaber_t
 * 							pos	, value of relative target position
 * Function Return		: 	None
 * Function Example		: 	FH_PosRel(&fh1, 20000);
 */
void FH_PosRel(FaulHaber_t *fh, int pos){
	fh1.start_pos = 1;
	FHmode(fh, PP);
	uint8_t temp[6]={0x0f, 0x00, 0, 0, 0, 0};
	CAN_OPEN_TxMsg(fh->hcanx, PDO1Rx, fh->nodeID, temp, 2);		//set bit 4 to LOW
	temp[0]=0x7f;
	memcpy(&temp[2], &pos, 4);
	CAN_OPEN_TxMsg(fh->hcanx, PDO2Rx, fh->nodeID, temp, 6);
}

/*
 * Function Name		: 	FH_Vel
 * Function Description : 	Called to set target velocity
 * Function Remarks		: 	-
 * Function Arguments	: 	*fh	, pointer to structure FaulHaber_t
 * 							vel	, value of target velocity
 * Function Return		: 	None
 * Function Example		: 	FH_Vel(&fh1, 2000);
 */
void FH_Vel(FaulHaber_t *fh, int vel){
	FHmode(fh, PV);
	uint8_t temp[8]={0x0f, 0x00, 0, 0, 0, 0};
	memcpy(&temp[2], &vel, 4);
	CAN_OPEN_TxMsg(fh->hcanx, PDO3Rx, fh->nodeID, temp, 6);
}

/*
 * Function Name		: 	FH_StopM
 * Function Description : 	Stops the motor with specified deceleration (manual)
 * Function Remarks		: 	Sets motion controller into Quick Stop Active State
 * Function Arguments	: 	*fh	, pointer to structure FaulHaber_t
 * 							dec	, value of deceleration
 * Function Return		: 	None
 * Function Example		: 	FH_StopM(&fh1, 25);
 */
void FH_StopM(FaulHaber_t *fh, uint32_t dec){

	uint8_t temp[8]={ 0x2b, 0x5a, 0x60, 0, 0x06, 0, 0, 0 };
	if(!fh->brake)
		FH_SDO(fh, temp);	//set to user-set brake deceleration
	fh->brake=1;
	temp[0]=0x23;
	temp[1]=0x85;
	memcpy(&temp[4], &dec, 4);
	FH_SDO(fh, temp);		//set brake deceleration
	temp[0]=0x0b;
	temp[1]=0;
	CAN_OPEN_TxMsg(fh->hcanx, PDO1Rx, fh->nodeID, temp, 2);		//quick stop active state

}

/*
 * Function Name		: 	FH_StopA
 * Function Description : 	Stops the motor naturally (Voltage = 0)  (auto)
 * Function Remarks		: 	-
 * Function Arguments	: 	*fh	, pointer to structure FaulHaber_t
 * Function Return		: 	None
 * Function Example		: 	FH_StopM(&fh1);
 */
void FH_StopA(FaulHaber_t *fh){
	//this function was never tested. Please delete this comment if tested
	uint8_t temp[8]={ 0x2b, 0x5a, 0x60, 0, 0x08, 0, 0, 0 };
	if(fh->brake)
		FH_SDO(fh, temp);	//set to natural brake deceleration
	fh->brake=0;
	temp[0]=0x0b;
	temp[1]=0;
	CAN_OPEN_TxMsg(fh->hcanx, PDO1Rx, fh->nodeID, temp, 2);		//quick stop active state
}
/*
 * Function Name		: 	FH_target
 * Function Description : 	Called to check if target position is reached in PP mode
 * Function Remarks		: 	If you call FH_target() immediately after FH_PosRel/Abs(),
 * 							it will return 1 (True), have to wait for a little only
 * 							will it return 0 (False)
 * Function Arguments	: 	*fh	, pointer to structure FaulHaber_t
 * Function Return		: 	1(True) or 0(False)
 * Function Example		: 	FH_target(&fh1);
 */
uint8_t FH_target(FaulHaber_t *fh){
	FHEnq(fh, PDO1Tx);
	if(fh->target)
		return 1;
	else
		return 0;
}

/*
 * Function Name		: 	FH_Pvel
 * Function Description : 	Called to set Profile Velocity.
 * 							PP mode: runs motor at speed = Profile Velocity
 * 							PV mode: max motor speed = Profile Velocity
 * Function Remarks		: 	The value has to be positive value, or the command will be ignored
 * Function Arguments	: 	*fh	, pointer to structure FaulHaber_t
 * 							vel	, value of Profile Velocity
 * Function Return		: 	None
 * Function Example		: 	FH_Pvel(&fh1, 3000);
 */
void FH_Pvel(FaulHaber_t *fh, uint32_t vel){
	fh->maxV=vel;
	uint8_t temp[8]={ 0x23, 0x81, 0x60, 0x00, 0, 0, 0, 0 };
	memcpy(&temp[4], &vel, 4);
	FH_SDO(fh, temp);
}

void FH_acc(FaulHaber_t *fh, uint32_t acc){

	uint8_t temp[8]={ 0x23, 0x83, 0x60, 0, 0, 0, 0, 0 };
	memcpy(&temp[4], &acc, 4);
	FH_SDO(fh, temp);
}

//*******************************************************************************************************************************************************\\

/*
 * Function Name		: 	FH_Pos_Limit
 * Function Description : 	Sets min and max Software Position Limit
 * Function Remarks		: 	-
 * Function Arguments	: 	*fh	, pointer to structure FaulHaber_t
 * 							max	, value of maximum Software Position Limit
 * 							min	, value of minimum Software Position Limit
 * Function Return		: 	None
 * Function Example		: 	FH_Pos_Limit(&fh1, 200000000, -200000000);
 */
void FH_Pos_Limit(FaulHaber_t *fh, int max, int min){
	fh->maxPos=max;
	fh->minPos=min;
	uint8_t temp[8]={0x23, 0x7d, 0x60, 0x02, 0, 0, 0, 0};
	int set = max + fh->offset;
	memcpy(&temp[4], &set, 4);
	FH_SDO(fh, temp);
	temp[3]=0x01;
	set = min + fh->offset;
	memcpy(&temp[4], &set, 4);
	FH_SDO(fh, temp);
}

/*
 * Function Name		: 	FHmode
 * Function Description : 	Called to set motion controller into specific mode
 * Function Remarks		: 	-
 * Function Arguments	: 	*fh	, pointer to structure FaulHaber_t
 * 							mode, mode of motion controller
 * Function Return		: 	None
 * Function Example		: 	FHmode(&fh1, PV);
 */
void FHmode(FaulHaber_t *fh, FH_OPmode mode){
	if(fh->mode != mode){
		uint8_t temp[8]={ 0x2f, 0x60, 0x60, 0x00, mode, 0x00, 0x00, 0x00 };
		FH_SDO(fh, temp);
	}

}

/*
 * Function Name		: 	FH_SDO
 * Function Description : 	Called to write SDO request
 * Function Remarks		: 	-
 * Function Arguments	: 	*fh		, pointer to structure FaulHaber_t
 * 							buff	, data to be written
 * Function Return		: 	None
 * Function Example		: 	FH_SDO(&fh1, temp);
 */
void FH_SDO(FaulHaber_t *fh, uint8_t buff[8]){
	fh->waiting=1;
	memcpy(fh->conf, &buff[1], 3);
	CAN_OPEN_TxMsg(fh->hcanx, SDORx, fh->nodeID, buff, 8);
	uint32_t ti=HAL_GetTick();
	while(fh->waiting){
		if(HAL_GetTick()-ti >= 25){
			CAN_OPEN_TxMsg(fh->hcanx, SDORx, fh->nodeID, buff, 8);
			ti=HAL_GetTick();
		}
	}
	fh->conf[0]=fh->conf[1]=fh->conf[2]=0;
}

/*
 * Function Name		: 	FH_CheckCAN
 * Function Description : 	Check if CAN Id received is one of the FaulHaber instructions
 * Function Remarks		: 	-
 * Function Arguments	: 	*fh	, pointer to structure FaulHaber_t
 * 							dat	, data received by CAN
 * Function Return		: 	0(FaulHaber ID found) or 1(not from FaulHaber)
 * Function Example		: 	FH_CheckCAN(&fh1, temp);
 */
uint8_t FH_CheckCAN(FaulHaber_t *fh, uint8_t dat[8]){
	uint32_t id;
	if(fh->hcanx==&hcan1){
		id=CAN1RxMessage.StdId;
	}else{
		id=CAN2RxMessage.StdId;
	}

	if((id - fh->nodeID) & 0b00001111111)
		return 1;
	switch((id)>>7){

		case 0b1110:		//boot up messege
			return 0;
			break;

		case 0x80:		//error messege
			return 0;
			break;

		case PDO1Tx:		//PDO1/statusword
			fh->target=(dat[1]>>2) & 0b01;		//target reached bit in PP mode
			if(dat[0] & 0b01000000)				//switch on disabled bit
				fh->PDOInit=1;
			else if(dat[0] & 0b00000100)
				fh->start=1;
			fh->waiting=0;
			return 0;
			break;

		case PDO2Tx:		//PDO2/position
			memcpy(fh->rx_buff, &dat[2], 4);
			fh->rx-=fh->offset;
			fh->waiting=0;
			return 0;
			break;

		case PDO3Tx:		//PDO3/velocity
			memcpy(fh->rx_buff, &dat[2], 4);
			fh->waiting=0;
			return 0;
			break;

		case PDO4Tx:		//PDO4/torque
			memcpy(fh->rx_buff, &dat[2], 4);//not sure if need to convert to int16, because datasheet says type is S16, not S32
			fh->waiting=0;
			return 0;
			break;

		case SDOTx:		//SDO
			if((dat[0]=0x60)&&(dat[1]==fh->conf[0])&&(dat[2]==fh->conf[1])&&(dat[3]==fh->conf[2]))
				fh->waiting=0;
			return 0;
			break;

		default :
			return 1;
			break;
	}
}

/*
 * Function Name		: 	FaulHaber_Handler
 * Function Description : 	Called to handle the received CAN data from FaulHaber.
 * Function Remarks		: 	Can be configured to return boolean value to indicate whether
 * 							the CAN data is from FaulHaber or not
 * Function Arguments	: 	dat	, data received by CAN
 * Function Return		: 	None
 * Function Example		: 	FaulHaber_Handler(aData);
 */
void FaulHaber_Handler(uint8_t dat[8]){
	led2 = 1;
	uint8_t bflag=1, ad=0;
	while(bflag && pfh[ad]!=0)
		bflag=FH_CheckCAN(pfh[ad++], dat);

}
