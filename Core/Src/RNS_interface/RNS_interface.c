/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "RNS_interface.h"
#include "../common.h"
#include "../adapter.h"
//#include "include.h"

/*********************************************/
/*          Variable                         */
/*********************************************/

uint8_t insData_send[2];

/*********************************************/
/*           Subroutine Function             */
/*********************************************/

/*
 * Function Name		: RNSInit
 * Function Description : This function is called to initialize the Robot Navigation System Module.
 * Function Remarks		: NONE
 * Function Arguments	: -if user define USED_CAN
 * 						   		CANx 		Select CAN peripheral (CAN1 or CAN2)
 * 						 		rns 		pointer to a RNS data structure with RNS_interface _t type
 * 						  -if user define USED_I2C
 * 						  		id			I2C address of RNS
 * 						  		I2Cx		Select I2C peripheral (I2C1, I2C2 or I2C3)
 * 						  		rns 		pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSInit(CAN1, &RNS);
 * 						  RNSInit(Robot_navi_system, I2C1,  &RNS);
 */

void RNSInit(CAN_HandleTypeDef* hcanx, RNS_interface_t* rns)
{
	rns->rns_hcanx = hcanx;

	insData_send[0] = 1;
	insData_send[1] = RNS_PENDING;

	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS,insData_send,2);
	rns->RNS_data.common_instruction = RNS_PENDING;

	insData_send[0] = 1;
	insData_send[1] = RNS_RESET_POS;
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS,insData_send,2);

	rns->RNS_data.common_instruction = RNS_WAITING;
	int wait=0;
	while(rns->RNS_data.common_instruction == RNS_WAITING){
		if(wait >= 2000000){
			insData_send[0] = 1;
			insData_send[1] = RNS_RESET_POS;
			CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS,insData_send,2);
			GPIOC_OUT->bit15 = !GPIOC_OUT->bit15;
			wait = 0;
		}else{
			wait ++;
		}
	}
}





/*
 * Function Name		: RNSStop
 * Function Description : Command the RNS board to stop and reset the position count.
 * Function Remarks		: NONE
 * Function Arguments	: rns 		pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSStop(&RNS);
 */

void RNSStop(RNS_interface_t* rns)
{


	rns->ins.instruction = RNS_STOP;
	rns->ins.ins_buffer[0].data = 0.0;
	rns->ins.ins_buffer[1].data = 0.0;
	rns->ins.ins_buffer[2].data = 0.0;
	rns->ins.ins_buffer[3].data = 0.0;

	//RNSSendIns(rns);
	rns->RNS_data.common_instruction = RNS_WAITING;

	insData_send[0] = 17;
	insData_send[1] = rns->ins.instruction;
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS,insData_send,2);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf1,&(rns->ins.ins_buffer[0]),8);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf2,&(rns->ins.ins_buffer[2]),8);

	int wait = 0;
	while(rns->RNS_data.common_instruction == RNS_WAITING)
	{
		if(wait >= 2000000)
		{
			insData_send[0] = 17;
			insData_send[1] = rns->ins.instruction;
			CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS,insData_send,2);
			CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf1,&(rns->ins.ins_buffer[0]),8);
			CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf2,&(rns->ins.ins_buffer[2]),8);

			GPIOC_OUT->bit15 = !GPIOC_OUT->bit15; // Toggle LED3
			wait = 0;
		}
		else
			wait++;
	}


}



/*
 * Function Name		: RNSVelocity
 * Function Description : Command the RNS to move with specified velocity without any position control.
 * Function Remarks		: NONE
 * Function Arguments	: f_left_velocity	speed of front left motor in meter per second
 * 						  f_right_velocity	speed of front right motor in meter per second
 * 						  b_left_velocity 	speed of back left motor in meter per second
 * 						  b_left_velocity	speed of back right motor in meter per second
 * 						  rns 				pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSVelocity(1.0, 1.5 , 1.5 , 1.0, &RNS);
 */

void RNSVelocity(float fFLeftVelR, float fFRightVelR, float fBLeftVelR, float fBRightVelR, RNS_interface_t* rns)
{

	rns->ins.instruction = RNS_VELOCITY;
	rns->ins.ins_buffer[0].data = fFLeftVelR;
	rns->ins.ins_buffer[1].data = fFRightVelR;
	rns->ins.ins_buffer[2].data = fBLeftVelR;
	rns->ins.ins_buffer[3].data = fBRightVelR;

	RNSSendIns(rns);

}



/*
 * Function Name		: RNSPDC
 * Function Description : Command the RNS to move with give pulse width modulation duty cycle.
 * Function Remarks		: NONE
 * Function Arguments	: f_left_pdc	speed of front left motor in meter per second
 * 						  f_right_pdc	speed of front right motor in meter per second
 * 						  b_left_pdc 	speed of back left motor in meter per second
 * 						  b_left_pdc	speed of back right motor in meter per second
 * 						  rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSVelocity(1.0, 1.5 , 1.5 , 1.0, &RNS);
 */
void RNSPDC(float fFLeftPDC, float fFRightPDC, float fBLeftPDC, float fBRightPDC, RNS_interface_t* rns)
{
	rns->ins.instruction = RNS_PDC;
	rns->ins.ins_buffer[0].data = fFLeftPDC;
	rns->ins.ins_buffer[1].data = fFRightPDC;
	rns->ins.ins_buffer[2].data = fBLeftPDC;
	rns->ins.ins_buffer[3].data = fBRightPDC;

	RNSSendIns(rns);

}

/*
 * Function Name		: RNSLFDist
 * Function Description : Command the RNS to line follow with given distance and direction.
 * Function Remarks		: NONE
 * Function Arguments	: MoveDir	    enum type with the members of x_ax, y_ax
 * 						  Dir			enum type with the members of
 * 						     			DIR_FRONT, DIR_BACK,DIR_LEFT,DIR_RIGHT
 * 						  LF_dist 		Distance for line follow
 * 						  rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSLFDist(x_ax, DIR_LEFT, 100.0 ,&RNS);
 */
void RNSLFDist( dir_t Dir, float LF_vel, float LF_dist,RNS_interface_t* rns){

	rns->ins.instruction = RNS_LF_DIST;
	rns->ins.ins_buffer[0].data = (float)Dir;
	rns->ins.ins_buffer[1].data = LF_vel;
	rns->ins.ins_buffer[2].data = LF_dist;
	rns->ins.ins_buffer[3].data = 0;

	RNSSendIns(rns);
}

/*
 * Function Name		: RNSLFJunc
 * Function Description : Command the RNS to line follow with given direction and junction.
 * Function Remarks		: NONE
 * Function Arguments	: Dir			enum type with the members of
 * 						     			DIR_FRONT, DIR_BACK,DIR_LEFT,DIR_RIGHT
 * 						  LF_junc 		number of junction to be pass through with line follow
 * 						  rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSLFJunc(DIR_LEFT,1.5,2.5, 2.0 ,&RNS);
 */
void RNSLFJunc(dir_t Dir, float LF_vel, float LF_dist, float LF_junc,RNS_interface_t* rns){

	rns->ins.instruction = RNS_LF_JUNC;
	rns->ins.ins_buffer[0].data = (float)Dir;
	rns->ins.ins_buffer[1].data = LF_vel;
	rns->ins.ins_buffer[2].data = LF_dist;
	rns->ins.ins_buffer[3].data = LF_junc;

	RNSSendIns(rns);
}
/*
 * Function Name		: RNSIMURotate
 * Function Description : Command the RNS to rotate with given angle of max:+-180 degree.
 * Function Remarks		: NONE
 * Function Arguments	: AngleDeg		Rotate angle. Max:+-180degree, +:clkwise, -:anticlkwise
 * 						  rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSIMURotate(70,&RNS);
 */
void RNSIMURotate(int AngleDeg,RNS_interface_t* rns){
	rns->ins.instruction = RNS_ROTATE;
	rns->ins.ins_buffer[0].data = (float)AngleDeg;
	rns->ins.ins_buffer[1].data = 0;
	rns->ins.ins_buffer[2].data = 0;
	rns->ins.ins_buffer[3].data = 0;

	RNSSendIns(rns);
}

/*
 * Function Name		: RNSOdnGoto
 * Function Description : This function is called to send all the points  for the path to rns for calculation
 * Function Remarks		: This function can only be called after RNSOdnStart is called
 * Function Arguments	: allpoints[][5]	array for all the points in the path
 * 							[][0]= minimum speed
 * 							[][1]= x-coordinate
 * 							[][2]= y-coordinate
 * 							[][3]= z-coordinate
 * 							[][4]= xy pid output
 * 							[][5]= Point Lock
 * 							[][6]= Curve Control Radius
 * 						  no_point			Number of points to be sent
 * 						  rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: float point[1][7] = {{3.0, 0.001, 1.0, -180.0, 0.0, 0.0, 1.5}};					//for coordinates use this type of naming
 * 								RNSPPstart(point,1,&rns);
 */

void RNSPPstart(float allpoints[][7],int no_point,RNS_interface_t* rns){
	RNSSet(rns, RNS_PPSend_num_Point,(float)no_point);
	int k;
	for(k=0;k<no_point;k++)
		RNSSet(rns, RNS_PPSendPoint, allpoints[k][0],allpoints[k][1],allpoints[k][2],allpoints[k][3],allpoints[k][4],allpoints[k][5],allpoints[k][6]);
	rns->ins.instruction = RNS_PPStart;
	rns->ins.ins_buffer[0].data = 0;
	rns->ins.ins_buffer[1].data = 0;
	rns->ins.ins_buffer[2].data = 0;
	rns->ins.ins_buffer[3].data = 0;

	RNSSendIns(rns);
}

/*
 * Function Name		: RNSSendVelIns
 * Function Description : Checks the status of the RNS and sends the instruction to RNS after the previous instruction is sent.
 * Function Remarks		: Not intended to be used by user
 * Function Arguments	: rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: None
 */

void RNSSendIns(RNS_interface_t* rns)
{
	while(rns->RNS_data.common_instruction == RNS_BUSY);
	rns->RNS_data.common_instruction = RNS_WAITING;

	insData_send[0] = 17;
	insData_send[1] = rns->ins.instruction;
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS, insData_send,2);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf1,&(rns->ins.ins_buffer[0].data),8);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf2,&(rns->ins.ins_buffer[2].data),8);

	int wait = 0;
	while(rns->RNS_data.common_instruction == RNS_WAITING)
	{
		if(wait >= 2000000)
		{
			insData_send[0] = 17;
			insData_send[1] = rns->ins.instruction;
			CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS, insData_send,2);
			CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf1,&(rns->ins.ins_buffer[0].data),8);
			CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf2,&(rns->ins.ins_buffer[2].data),8);

			GPIOC_OUT->bit15 = !GPIOC_OUT->bit15; // Toggle LED3
			wait = 0;
		}
		else
		{
			wait++;
		}
	}

}

/*
 * Function Name		: RNSSet
 * Function Description : Configure the RNS in terms of system function and gains.
 * Function Remarks		: Can be called during an instruction is running or during pending. Every value must cast to float type.
 * Function Arguments	: rns 			pointer to a RNS data structure with RNS_interface _t type
 * 						  parameter 	Enumeration of parameters
 * 						  ...			Values to set for the RNS, depending on parameters
 * Function Return		: None
 * Function Example		: RNSSet(&RNS, RNS_F_KCD_PTD, 0.9956, 0.01/2000);
 */

void RNSSet(RNS_interface_t* rns, unsigned char parameter, ...)
{

	va_list value;
	rns->param.parameter = parameter;
	va_start(value, parameter);
	while(rns->RNS_data.common_instruction == RNS_WAITING);
	if (parameter > RNS_PARAM_1){
		rns->param.param_buffer[0].data = va_arg(value, double);
		rns->param.param_buffer[1].data = 0;
		rns->param.param_buffer[2].data = 0;
		rns->param.param_buffer[3].data = 0;
		rns->param.param_buffer[4].data = 0;
		rns->param.param_buffer[5].data = 0;
		rns->param.param_buffer[6].data = 0;
		rns->param.param_buffer[7].data = 0;
	}
	if (parameter > RNS_PARAM_2){
		rns->param.param_buffer[1].data = va_arg(value, double);
		rns->param.param_buffer[2].data = 0;
		rns->param.param_buffer[3].data = 0;
		rns->param.param_buffer[4].data = 0;
		rns->param.param_buffer[5].data = 0;
		rns->param.param_buffer[6].data = 0;
		rns->param.param_buffer[7].data = 0;
	}
	if (parameter > RNS_PARAM_3){
		rns->param.param_buffer[2].data = va_arg(value, double);
		rns->param.param_buffer[3].data = 0;
		rns->param.param_buffer[4].data = 0;
		rns->param.param_buffer[5].data = 0;
		rns->param.param_buffer[6].data = 0;
		rns->param.param_buffer[7].data = 0;
	}
	if (parameter > RNS_PARAM_4){
		rns->param.param_buffer[3].data = va_arg(value, double);
		rns->param.param_buffer[4].data = 0;
		rns->param.param_buffer[5].data = 0;
		rns->param.param_buffer[6].data = 0;
		rns->param.param_buffer[7].data = 0;
	}
	if (parameter > RNS_PARAM_5){
		rns->param.param_buffer[4].data = va_arg(value,double);
		rns->param.param_buffer[5].data = 0;
		rns->param.param_buffer[6].data = 0;
		rns->param.param_buffer[7].data = 0;
	}
	if (parameter > RNS_PARAM_6){
		rns->param.param_buffer[5].data = va_arg(value, double);
		rns->param.param_buffer[6].data = 0;
		rns->param.param_buffer[7].data = 0;
	}
	if (parameter > RNS_PARAM_7){
		rns->param.param_buffer[6].data = va_arg(value, double);
		rns->param.param_buffer[7].data = 0;
	}
	if(parameter > RNS_PARAM_8){
		rns->param.param_buffer[7].data = va_arg(value, double);
	}

	va_end(value);

	insData_send[0] = 17;
	insData_send[1] = rns->param.parameter;

	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS,insData_send,2);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf1,&(rns->param.param_buffer[0]),8);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf2,&(rns->param.param_buffer[2]),8);
	if(parameter > RNS_PARAM_5)
		CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf3,&(rns->param.param_buffer[4]),8);
	if(parameter > RNS_PARAM_7)
		CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf4,&(rns->param.param_buffer[6]),8);

	rns->RNS_data.common_instruction = RNS_WAITING;
	int wait = 0;

	while(rns->RNS_data.common_instruction == RNS_WAITING)
	{
		if(wait >= 2000000)
		{
			insData_send[0] = 17;
			insData_send[1] = rns->param.parameter;

			CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS,insData_send,2);
			CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf1,&(rns->param.param_buffer[0]),8);
			CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf2,&(rns->param.param_buffer[2]),8);
			if(parameter > RNS_PARAM_5)
				CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf3,&(rns->param.param_buffer[4]),8);
			if(parameter > RNS_PARAM_7)
				CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf4,&(rns->param.param_buffer[6]),8);

			GPIOC_OUT->bit15 = !GPIOC_OUT->bit15; // Toggle LED3
			wait = 0;
		}
		else
			wait++;
	}
	//UARTPrintString(&huart5,"1\r\n");

}

/*
 * Function Name		: RNSEnquire
 * Function Description : This function is classified under enquiry type instruction.
 * Function Remarks		: Should only be called during an instruction is running (the RNS is in motion).
 * 						  Failing to do so will return gibberish data or previously returned value.
 * Function Arguments	: parameter 	Enumeration to parameter to enquire
 * 						  rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSSet(&RNS, RNS_F_KCD_PTD, 0.9956, 0.01/2000);
 */

uint8_t RNSEnquire(unsigned char parameter, RNS_interface_t* rns)
{

	rns->ins.instruction = parameter;
	rns->ins.ins_buffer[0].data = 0.0;
	rns->ins.ins_buffer[1].data = 0.0;
	rns->ins.ins_buffer[2].data = 0.0;
	rns->ins.ins_buffer[3].data = 0.0;

//	RNSSendIns(rns);

	rns->RNS_data.common_instruction = RNS_WAITING;

	insData_send[0] = 17;
	insData_send[1] = rns->ins.instruction;
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS,insData_send,2);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf1,&(rns->ins.ins_buffer[0].data),8);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf2,&(rns->ins.ins_buffer[2].data),8);

	int wait = 0;
	while(rns->RNS_data.common_instruction == RNS_WAITING)
	{
		if(wait >= 2000000)
		{
			insData_send[0] = 17;
			insData_send[1] = rns->ins.instruction;
			CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS,insData_send,2);
			CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf1,&(rns->ins.ins_buffer[0].data),8);
			CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf2,&(rns->ins.ins_buffer[2].data),8);

			GPIOC_OUT->bit15 = !GPIOC_OUT->bit15; // Toggle LED3
			wait = 0;
		}
		else wait++;

	}

	rns->enq.enquiry = parameter;
	rns->enq.enq_buffer[0].data = rns->RNS_data.common_buffer[0].data;
	rns->enq.enq_buffer[1].data = rns->RNS_data.common_buffer[1].data;
	rns->enq.enq_buffer[2].data = rns->RNS_data.common_buffer[2].data;
	rns->enq.enq_buffer[3].data = rns->RNS_data.common_buffer[3].data;


	return 1;
}

void RNSSync (RNS_interface_t* rns)
{
	if(rns->busy){
		GPIOC_OUT->bit15 = !GPIOC_OUT->bit15;
		insData_send[0] = 5;
		insData_send[1] = 0;
		CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS, insData_send,2);
	}
}
/*********************************************/



