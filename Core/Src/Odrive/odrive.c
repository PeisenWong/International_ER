/*
 * odriver.c
 *
 *  Created on: Oct 6, 2022
 *      Author: heheibhoi
 */

#include "odrive.h"
#include "../adapter.h"
/*Initialise variables*/
uint8_t Odrv_Commands[8]={GET_ENCODER_ESTIMATE,GET_IQ,HEARTBEAT,GET_MOTOR_ERROR,GET_ENCODER_ERROR,GET_SENSORLESS_ERROR,GET_SHADOW_COUNT,GET_BUS_VOLTAGE};
int number_of_odrive = 0;

// Code from Anas

void Odrive_Init(Odrive_t *Odrive, UART_HandleTypeDef *huartx){
	Odrive->huartx = huartx;
}

void Odrive_Pos(Odrive_t *Odrive, int motor_number, float position, float torque_limit, float vel_limit){
	sprintf(Odrive->buffer,"q %u %.3f %.3f %.3f\n",motor_number, position, torque_limit, vel_limit);
	HAL_UART_Transmit(Odrive->huartx, (uint8_t *)Odrive->buffer, strlen(Odrive->buffer),10);
}

void Odrive_Vel(Odrive_t *Odrive, int motor_number, float velocity, float torque_ff){
	sprintf(Odrive->buffer,"v %u %.3f %.3f\n",motor_number, velocity, torque_ff);
	HAL_UART_Transmit(Odrive->huartx, (uint8_t *)Odrive->buffer, strlen(Odrive->buffer),10);
}

void Odrive_Curr(Odrive_t *Odrive, int motor_number, float current){
	sprintf(Odrive->buffer,"c %d %.3f\n",motor_number, current);
	HAL_UART_Transmit(Odrive->huartx, (uint8_t *)Odrive->buffer, strlen(Odrive->buffer),10);
}

void Odrive_Traj(Odrive_t *Odrive, int motor_number, float position){
	sprintf(Odrive->buffer,"t %u %.3f\n",motor_number, position);
	HAL_UART_Transmit(Odrive->huartx,(uint8_t *)Odrive->buffer, strlen(Odrive->buffer),10);
}

void Odrive_Idle(Odrive_t *Odrive, int motor_number){
	sprintf(Odrive->buffer,"w axis%u.requested_state 1\n",motor_number);
	HAL_UART_Transmit(Odrive->huartx,(uint8_t *)Odrive->buffer, strlen(Odrive->buffer),10);
}

void Odrive_ClosedLoop(Odrive_t *Odrive, int motor_number){
	sprintf(Odrive->buffer,"w axis%u.requested_state 8\n",motor_number);
	HAL_UART_Transmit(Odrive->huartx,(uint8_t *)Odrive->buffer, strlen(Odrive->buffer),10);
}

void Odrive_Vel_Limit(Odrive_t *Odrive, int motor_number, float vel_limit){
	sprintf(Odrive->buffer,"w axis%u.controller.config.vel_limit %.3f\n",motor_number,vel_limit);
	HAL_UART_Transmit(Odrive->huartx,(uint8_t *)Odrive->buffer, strlen(Odrive->buffer),10);
}

void Odrive_ClearErrors(Odrive_t *Odrive){
	sprintf(Odrive->buffer,"sc\n");
	HAL_UART_Transmit(Odrive->huartx,(uint8_t *)Odrive->buffer, strlen(Odrive->buffer),10);
}

// Not Tested
float Odrive_Read(Odrive_t *Odrive){
	memset(Odrive->buffer,'\0',sizeof(Odrive->buffer));
	uint8_t cdata = '\0';
	uint8_t i = 0;
	uint32_t timeout = HAL_GetTick();

	while(cdata != '\n'){
		HAL_UART_Receive(Odrive->huartx, &cdata, 1, 10);
		Odrive->buffer[i] = cdata;
		i+=1;
		if(HAL_GetTick() - timeout > 500){
			break;
		}
	}
	return(atof(Odrive->buffer));
}
// Not Tested
float Odrive_GetVelocity(Odrive_t *Odrive, int motor_number) {
	sprintf(Odrive->buffer,"r axis%u.encoder.vel_estimate\n",motor_number);
	HAL_UART_Transmit(Odrive->huartx,(uint8_t *)Odrive->buffer, strlen(Odrive->buffer),10);
	return(Odrive_Read(Odrive));
}
// Not Tested
float Odrive_GetPosition(Odrive_t *Odrive, int motor_number) {
	sprintf(Odrive->buffer,"r axis%u.encoder.pos_estimate\n",motor_number);
	HAL_UART_Transmit(Odrive->huartx,(uint8_t *)Odrive->buffer, strlen(Odrive->buffer),3);
	return(Odrive_Read(Odrive));
}

// End code from Anas


/*
 * Function Name			: OdriveInit
 * Function Description  : This function is called to init odrive structure
 * Function Arguments	: odrive ,structure pointer to Odrv_t
 * 						  hcanx, can used for communication with odrive
 * 						  axis_id, can id, restricted to 6 bits,max 0x3F
 * Function Return		: None
 * Function Example		: 	OdriveInit(&Odrv1,&hcan1,ODRIVE1,15.0,10.0,0.0,VEL_RAMP,VELOCITY_CONTROL);
 */
void OdriveInit(Odrv_t* odrive, CAN_HandleTypeDef* hcanx, Odrive_Type type, uint16_t axis_id,ControlMode control_mode,InputMode input_mode)
{

	if(number_of_odrive==0)
	{
		Odrv_ID = (uint8_t*)malloc(sizeof(uint8_t));
		P_to_Odrive =  (Odrv_t **)malloc(sizeof(Odrv_t*));
	}

	else
	{
		Odrv_ID = realloc(Odrv_ID,(number_of_odrive+1)*sizeof(uint8_t));
		P_to_Odrive = realloc(P_to_Odrive,(number_of_odrive+1) * sizeof(Odrv_t*));
	}

	Odrv_ID[number_of_odrive] = axis_id;

	P_to_Odrive[number_of_odrive] = odrive;

	odrive->Instance = axis_id;

	odrive->hcanx=hcanx;

	odrive->type=type;

	OdriveSetControlInputMode(odrive,control_mode,input_mode);

	odrive->stop=true;


	number_of_odrive++;


}

/*
 * Function Name			: OdriveSetControlMode
 * Function Description  : This function is called to change control mode of odrive
 * Function Arguments	: odrive ,structure pointer to Odrv_t
 * 						  requested_control_mode
 * Function Return		: None
 * Function Example		:	OdriveSetControlMode(&odrive,POSITION_CONTROL);
 */
void OdriveSetControlMode(Odrv_t* odrive,ControlMode requested_control_mode)
{
	uint8_t TxBuffer[8]={0};

	if(odrive->control_mode==requested_control_mode)
		return;

	TxBuffer[0] = requested_control_mode;

	odrive->control_mode=requested_control_mode;

	OdriveSendCAN(odrive,SET_CONTROLLER_MODES,TxBuffer);
}

void OdriveSetWheelDiameter(Odrv_t* odrive,float wheel_diameter)
{
	if(wheel_diameter<=0)
		return;
	odrive->wheel_diameter=wheel_diameter;

}
/*
 * Function Name			: OdriveSetInputMode
 * Function Description  : This function is called to change input mode of odrive
 * Function Arguments	: odrive ,structure pointer to Odrv_t
 * 						  requested_input_mode
 * Function Return		: None
 * Function Example		:	OdriveSetInputMode(&odrive,PASSTHROUGH);
 */
void OdriveSetInputMode(Odrv_t* odrive,InputMode requested_input_mode)
{
	uint8_t TxBuffer[8]={0};

	if(odrive->input_mode==requested_input_mode)
		return;

	TxBuffer[4]  = requested_input_mode;

	odrive->input_mode=requested_input_mode;

	OdriveSendCAN(odrive,SET_CONTROLLER_MODES,TxBuffer);
}

/*
 * Function Name			: OdriveSetControlInputMode
 * Function Description  : This function is called to change input mode and control of odrive
 * Function Arguments	: odrive ,structure pointer to Odrv_t
 *                         requested_control_mode
 * 						  requested_input_mode
 * Function Return		: None
 * Function Example		:	OdriveSetControlInputMode(&odrive,VELOCITY_CONTROL,PASSTHROUGH);
 */

void OdriveSetControlInputMode(Odrv_t* odrive,ControlMode requested_control_mode,InputMode requested_input_mode)
{

	uint8_t TxBuffer[8]={0};

	if(odrive->control_mode==requested_control_mode&&odrive->input_mode==requested_input_mode)
		return;

	TxBuffer[0]  = requested_control_mode;

	TxBuffer[4]  = requested_input_mode;

	odrive->control_mode=requested_control_mode;

	odrive->input_mode=requested_input_mode;

	OdriveSendCAN(odrive,SET_CONTROLLER_MODES,TxBuffer);

}

/*
 * Function Name			: OdriveTurn
 * Function Description  : This function is called to turn odrive motor n number of turns from current position, float number is also acceptable
 * Function Arguments	: odrive ,structure pointer to Odrv_t
 *                         count_num,number of turns to count
 * 						  input_mode, requested_input_mode
 * Function Return		: None
 * Function Example		:OdriveTurn(&odrive, 1.3,TRAPEZOIDAL_TRAJECTORY);
 */

void OdriveTurn(Odrv_t* odrive, float count_num,InputMode input_mode)//turn number of turns
{
	odrive->stop=odrive->hand_brake=0;

	uint8_t TxBuffer[8]={0};

	float target;

	if(input_mode!=PASSTHROUGH&&input_mode!=POS_FILTER&&input_mode!=TRAPEZOIDAL_TRAJECTORY)
		input_mode=TRAPEZOIDAL_TRAJECTORY;

	if(odrive->type==DENG_FOC){

		OdriveEnquire(odrive,TURN_COUNT_AND_VELOCITY);
		while(odrive->busy[TURN_COUNT_AND_VELOCITY]);

	}

	target = odrive->feedback.encoder + (float)count_num;
	memcpy(&TxBuffer[0],&target,4);
	OdriveSetControlInputMode(odrive,POSITION_CONTROL,input_mode);
	OdriveSendCAN(odrive,SET_INPUT_POS,TxBuffer);
}


/*
 * Function Name			: OdriveUpdatePos
 * Function Description  : This function is called to avoid odrive overspeed error that occurs when user change from velocity control mode to position control mode
 * Function Arguments	: odrive ,structure pointer to Odrv_t
 * Function Return		: None
 * Function Example		:OdriveUpdatePos(&odrive);
 */
void OdriveUpdatePos(Odrv_t* odrive)
{
	uint8_t TxBuffer[8]={0};
	if(odrive->type==DENG_FOC)
	{
		OdriveEnquire(odrive,TURN_COUNT_AND_VELOCITY);
		while(odrive->busy[TURN_COUNT_AND_VELOCITY]);
	}
	memcpy(&TxBuffer[0],&odrive->feedback.encoder,4);
	OdriveSendCAN(odrive,SET_INPUT_POS,TxBuffer);
}


/*
 * Function Name			: OdriveTurnCountInertia
 * Function Description  : This function is called to turn with high initial load
 * Function Arguments	: odrive ,structure pointer to Odrv_t
 *                         count_num
 *                         inertia
 *
 * Function Return		: None
 * Function Example		:OdriveTurnCountInertia(&odrive,1.1,0.5);
 */
void OdriveTurnCountInertia(Odrv_t* odrive, float count_num,float inertia)//turn number of turns
{
	odrive->stop=0;
	uint8_t TxBuffer[8]={0};
	OdriveSetInertia(odrive, inertia);
	memcpy(&TxBuffer[0],&count_num,4);
	OdriveSendCAN(odrive,SET_INPUT_POS,TxBuffer);
	OdriveSetControlInputMode(odrive,POSITION_CONTROL,TRAPEZOIDAL_TRAJECTORY);
}

void OdriveHandBrake(Odrv_t* odrive)
{
	if(HAL_GetTick() - odrive->hand_brake_start>=BRAKE_PERIOD && odrive->hand_brake)
		OdriveStop(odrive);

	if(odrive->hand_brake==true)
		return;

	uint8_t TxBuffer[8]={0};

	if(odrive->type==DENG_FOC){
		OdriveEnquire(odrive,TURN_COUNT_AND_VELOCITY);
		while(odrive->busy);
	}

	memcpy(&TxBuffer[0],&odrive->feedback.encoder,4);
	OdriveSetControlInputMode(odrive, POSITION_CONTROL, PASSTHROUGH);//only passthrough works, trap traj and pos filter will induce overspeed
	OdriveSendCAN(odrive,SET_INPUT_POS,TxBuffer);
	odrive->hand_brake=1;
	odrive->hand_brake_start=HAL_GetTick();

}

/*
 * Function Name			: OdriveVelocity
 * Function Description  : This function is called to turn odrive motor in RPS
 * Function Arguments	: odrive ,structure pointer to Odrv_t
 *                         target_velocity, target rps
 * 						  input_mode, requested_input_mode
 * Function Return		: None
 * Function Example		:OdriveVelocity(&odrive, 1.3,VEL_RAMP);
 */
void OdriveVelocity(Odrv_t* odrive, float target_velocity,InputMode input_mode)//turn odrive velocity
{

	uint8_t TxBuffer[8]={0};

	if(input_mode!=PASSTHROUGH&&input_mode!=VEL_RAMP)
		input_mode=PASSTHROUGH;

	memcpy(&TxBuffer[0],&target_velocity,4);

	OdriveSetControlInputMode(odrive,VELOCITY_CONTROL,input_mode);
	OdriveSendCAN(odrive,SET_INPUT_VEL,TxBuffer);


	odrive->stop=odrive->hand_brake=0;

}
/*
 * Function Name			: OdriveStop
 * Function Description  : This function is called to stop odrive with handbrake or without
 * Function Arguments	: odrive ,structure pointer to Odrv_t
 * Function Return		: None
 * Function Example		:OdriveStop(&odrive);
 */
void OdriveStop(Odrv_t* odrive)
{
	if(odrive->stop==true)
		return;
	OdriveVelocity(odrive, 0.0, VEL_RAMP);
	odrive->stop=1;
}

void OdriveBlockingRelease(Odrv_t* odrive)//sometimes orive will miss out the command
{
	if(odrive->feedback.state==AXIS_CLOSED_LOOP_CONTROL){

		int start=HAL_GetTick();
		OdriveSetAxisRequestedState(odrive,IDLE_STATE);
		while(odrive->feedback.state==AXIS_CLOSED_LOOP_CONTROL)
		{
			if(HAL_GetTick()-start>O_DRV_TIMEOUT){
				start=HAL_GetTick();
				OdriveSetAxisRequestedState(odrive,IDLE_STATE);
			}
		}
	}
}

void OdriveBlockingArm(Odrv_t* odrive)
{
	if(odrive->feedback.state==AXIS_IDLE_STATE){
		int start=HAL_GetTick();
		OdriveSetAxisRequestedState(odrive,AXIS_CLOSED_LOOP_CONTROL);
		while(odrive->feedback.state==AXIS_IDLE_STATE)
		{
			if(HAL_GetTick()-start>O_DRV_TIMEOUT){
				start=HAL_GetTick();
				OdriveSetAxisRequestedState(odrive,AXIS_CLOSED_LOOP_CONTROL);
			}
		}
	}
}
void OdriveRelease(Odrv_t* odrive)
{
	if(odrive->feedback.state==AXIS_CLOSED_LOOP_CONTROL)
		OdriveSetAxisRequestedState(odrive,IDLE_STATE);
}

void OdriveArm(Odrv_t* odrive)
{
	if(odrive->feedback.state==AXIS_IDLE_STATE)
		OdriveSetAxisRequestedState(odrive,CLOSED_LOOP_CONTROL);
}








/*
 * Function Name			: OdriveTorque
 * Function Description  : This function is called to turn odrive motor with torque control
 * Function Arguments	: odrive ,structure pointer to Odrv_t
 *                         target_torque
 *                         input mode
 * Function Return		: None
 * Function Example		:OdriveTorque(&odrive, 1.3,TORQUE_RAMP);
 */
void OdriveTorque(Odrv_t* odrive, float target_torque, InputMode input_mode)//torque control
{
	odrive->stop=odrive->hand_brake=0;

	uint8_t TxBuffer[8]={0};

	if(input_mode!=PASSTHROUGH&&input_mode!=TORQUE_RAMP)
		input_mode=TORQUE_RAMP;

	memcpy(&TxBuffer[0],&target_torque,sizeof(float));

	OdriveSetControlInputMode(odrive,TORQUE_CONTROL,input_mode);

	OdriveSendCAN(odrive,SET_INPUT_TORQUE,TxBuffer);
}

/*
 * Function Name			: OdriveRestart
 * Function Description  : This function is called to restart odrive
 * Function Arguments	: odrive ,structure pointer to Odrv_t
 * Function Return		: None
 * Function Example		:OdriveRestart(&odrive);
 */
void OdriveRestart(Odrv_t* odrive)
{
	uint8_t TxBuffer[8]={0};
	OdriveSendCAN(odrive,REBOOT_ODRIVE,TxBuffer);
}

/*
 * Function Name			: OdriveClearError
 * Function Description  : This function is called to clear errors of odrive
 * Function Arguments	: odrive ,structure pointer to Odrv_t
 * Function Return		: None
 * Function Example		:OdriveClearError(&odrive);
 */
void OdriveClearError(Odrv_t* odrive)
{

	uint8_t TxBuffer[8]={0};
	OdriveSendCAN(odrive,CLEAR_ERROR,TxBuffer);
}
/*
 * Function Name			: OdriveSetAbsolutePosition
 * Function Description  : This function is called to set absolute position of odrive(not useful)
 * Function Arguments	: odrive ,structure pointer to Odrv_t
 * Function Return		: None
 * Function Example		:OdriveSetAbsolutePosition(&odrive);
 */
void OdriveSetAbsolutePosition(Odrv_t* odrive)
{
	uint8_t TxBuffer[8]={0};
	memcpy(&TxBuffer[0],&odrive->feedback.encoder,4);
	OdriveSendCAN(odrive,SET_LINEAR_COUNTS,TxBuffer);
}


/*
 * Function Name			: OdriveEnquire
 * Function Description  : This function is called to get odrive feedback
 * Function Arguments	: odrive ,structure pointer to Odrv_t
 *                        feedback, the feedback required
 * Function Return		: None
 * Function Example		:OdriveEnquire(&odrive,TURN_COUNT_AND_VELOCITY);
 */
void OdriveEnquire(Odrv_t* odrive,Feedback feedback)
{

	if(feedback==BUS_VOLTAGE)
	{
		OdriveGetBusVoltage(odrive);
		odrive->busy[BUS_VOLTAGE]=1;

	}

	else if(feedback==TURN_COUNT_AND_VELOCITY)
	{
		OdriveGetEncoderFeedback(odrive);
		odrive->busy[TURN_COUNT_AND_VELOCITY]=1;

	}

	else if(feedback==SENSORLESS_ESTIMATE)
	{
		OdriveGetSensorlessEstimates(odrive);
		odrive->busy[SENSORLESS_ESTIMATE]=1;
	}

	else if(feedback==IQ_VALUE)
	{
		OdriveGetIQValue(odrive);
		odrive->busy[IQ_VALUE]=1;
	}

	else if(feedback==SHADOW_COUNT)
	{
		OdriveGetShadowCount(odrive);
		odrive->busy[SHADOW_COUNT]=1;
	}

}


/*
 * Function Name			: decode_Odrive
 * Function Description  : This function is called to decode odrive message sent back from o drive
 * Function Arguments	: odrive ,structure pointer to Odrv_t
 * Function Return		: None
 * Function Example		:decode_Odrive(&odrive);
 */

/*In CAN FIFO_Callback
 * uint16_t id_mask = 0x1F;   // mask last 7 bits, get only the command

			uint16_t command=CAN1RxMessage.StdId&id_mask;

			if(command==GET_ENCODER_ESTIMATE||command==GET_TEMPERATURE)
			{

				Odrvmsg.RXmsg = CAN1RxMessage;

				source = ODRIVE_PACKET;

				memcpy(Odrvmsg.Data,aData,CAN1RxMessage.DLC);

			}
 *
 * In CAN_PROCESS, call this
 *
 * uint16_t command_mask = 0x7E0;  // mask first 7 bits, get only the id

	uint16_t id = CAN1RxMessage.StdId&command_mask;

	id=id>>5;

	switch(packet_src){
	case ODRIVE_PACKET:


		if(id==Odrv1.Instance)
			{
			decode_Odrive(&Odrv1);
			}
		break;
	}

 * */
void decode_Odrive(Odrv_t* odrive)
{
	float data=0;
	uint16_t id = Odrvmsg.RXmsg.StdId;
	uint16_t mask = 0x01F;
	uint16_t command=id&mask;
	if(command==GET_ENCODER_ESTIMATE){

		if(odrive->type==DENG_FOC)
			odrive->busy[TURN_COUNT_AND_VELOCITY]=0;

		memcpy(&data,&Odrvmsg.Data[0],sizeof(float));
		odrive->feedback.encoder=data;
		memcpy(&data,&Odrvmsg.Data[4],sizeof(float));
		odrive->feedback.round_per_second=data;
		odrive->feedback.velocity=odrive->feedback.round_per_second*M_PI*odrive->wheel_diameter;

	}
	else if(command==HEARTBEAT){

		memcpy(&data,&Odrvmsg.Data[0],sizeof(float));
		odrive->feedback.error=data;

		odrive->feedback.state=Odrvmsg.Data[4];

		odrive->feedback.traj_state = Odrvmsg.Data[7];


		if(odrive->feedback.traj_state == TRAJECTORY_DONE && odrive->turning == 1)//TRAJECTORY
			odrive->turning = 0; // Done turning

	}

	else if(command==GET_BUS_VOLTAGE){
		memcpy(&data,&Odrvmsg.Data[0],sizeof(float));
		odrive->feedback.bus_voltage=data;

		odrive->busy[BUS_VOLTAGE]=0;
	}
	else if(command==GET_SENSORLESS_ESTIMATES){
		memcpy(&data,&Odrvmsg.Data[0],sizeof(float));
		odrive->feedback.sensorless_estimates.position=data;
		memcpy(&data,&Odrvmsg.Data[4],sizeof(float));
		odrive->feedback.sensorless_estimates.velocity=data;
		odrive->busy[SENSORLESS_ESTIMATE]=0;
	}
	else if(command==GET_IQ){
		memcpy(&data,&Odrvmsg.Data[0],sizeof(float));
		odrive->feedback.Iq.iq_setpoint=data;
		memcpy(&data,&Odrvmsg.Data[4],sizeof(float));
		odrive->feedback.Iq.iq_measured=data;
		odrive->busy[IQ_VALUE]=0;
	}
	else if(command==GET_MOTOR_ERROR){
		memcpy(&data,&Odrvmsg.Data[0],sizeof(float));
		odrive->feedback.error_code=data;
		strcat(odrive->feedback.error_msg, Odrive_Error_To_String(odrive->feedback.error_code));
		strcat(odrive->feedback.error_msg, "\n");
		odrive->busy[ODRIVE_ERROR_CODE]=0;
	}
	else if(command==GET_SHADOW_COUNT){
		memcpy(&data,&Odrvmsg.Data[0],sizeof(float));
		odrive->feedback.shadow_count=data;
		memcpy(&data,&Odrvmsg.Data[4],sizeof(float));
		odrive->feedback.cpr=data;
		odrive->busy[SHADOW_COUNT]=0;
	}
}

void OdriveBlockingTurn(Odrv_t* odrive,float turn_count)
{

	uint8_t TxBuffer[8]={0};
	if(odrive->type==DENG_FOC){
		OdriveEnquire(odrive,TURN_COUNT_AND_VELOCITY);
		int start_2 = HAL_GetTick();
		while(odrive->busy[TURN_COUNT_AND_VELOCITY])
		{
			if(HAL_GetTick()-start_2>500)
				break;
		}
	}

	float target = odrive->feedback.encoder + turn_count;

	memcpy(&TxBuffer[0],&target,4);
	OdriveSetControlInputMode(odrive,POSITION_CONTROL,TRAPEZOIDAL_TRAJECTORY);
	OdriveSendCAN(odrive,SET_INPUT_POS,TxBuffer);

	if(odrive->feedback.traj_state==TRAJECTORY)
	{
		while(odrive->feedback.traj_state==TRAJECTORY)
		{
			OdriveSetControlInputMode(odrive,POSITION_CONTROL,TRAPEZOIDAL_TRAJECTORY);
			OdriveSendCAN(odrive,SET_INPUT_POS,TxBuffer);
		}
	}
	else
	{
		while(odrive->feedback.traj_state==TRAJECTORY_DONE)
		{
			OdriveSetControlInputMode(odrive,POSITION_CONTROL,TRAPEZOIDAL_TRAJECTORY);
			OdriveSendCAN(odrive,SET_INPUT_POS,TxBuffer);
		}
		while(odrive->feedback.traj_state==TRAJECTORY)
		{
			OdriveSetControlInputMode(odrive,POSITION_CONTROL,TRAPEZOIDAL_TRAJECTORY);
			OdriveSendCAN(odrive,SET_INPUT_POS,TxBuffer);
		}
	}


	//	while(odrive->feedback.traj_state==TRAJECTORY_DONE);
	//	odrive->turning=1;
	//	int start = HAL_GetTick();
	//	while(odrive->turning==1)
	//	{
	//		sprintf((char*) debug, "Stuck second loop\n");
	//		HAL_UART_Transmit(&huart4, debug, strlen((char*) debug), HAL_MAX_DELAY);
	//		if(HAL_GetTick()-start>500)
	//			break;
	//	}
}


//void OdriveBlockingTurn(Odrv_t* odrive,float turn_count)
//{
//
//	uint8_t TxBuffer[8]={0};
//	if(odrive->type==DENG_FOC){
//		OdriveEnquire(odrive,TURN_COUNT_AND_VELOCITY);
//		while(odrive->busy[TURN_COUNT_AND_VELOCITY]);
//	}
//	float target = odrive->feedback.encoder+turn_count;
//	memcpy(&TxBuffer[0],&target,4);
//	OdriveSetControlInputMode(odrive,POSITION_CONTROL,TRAPEZOIDAL_TRAJECTORY);
//	OdriveSendCAN(odrive,SET_INPUT_POS,TxBuffer);
//	while(odrive->turning==1);
//	//		odrive->turning=1;
//	//	int start = HAL_GetTick();
//	//	while(odrive->turning)
//	//	{
//	//		if(HAL_GetTick()-start>500)
//	//			return;
//	//	}
//
//}


void OdriveSetAxisRequestedState(Odrv_t* odrive,AxisRequestedState AxisRequested_state)
{
	//odrive->current_state=AxisRequested_state;
	uint8_t TxBuffer[8]={0};
	memcpy(&TxBuffer[0],&AxisRequested_state,1);
	OdriveSendCAN(odrive,SET_AXIS_STATE,TxBuffer);
}

/*
 * Function Name			: OdriveSetInertia
 * Function Description  : This function is called to set inertia to odrive if a heavy load is attached
 * Function Arguments	: odrive ,structure pointer to Odrv_t
 *                         inertia value, initial torque value in Nm needed to move motor with a load attached
 * Function Return		: None
 * Function Example		:OdrveSetInertia(&odrive,1.1);
 */
void OdriveSetInertia(Odrv_t* odrive,float inertia)
{
	uint8_t TxBuffer[8]={0};

	memcpy(&TxBuffer[0],&inertia,4);

	OdriveSendCAN(odrive,SET_TRAP_TRAJ_INERTIA,TxBuffer);
	odrive->trap_traj_param.inertia=inertia;
}

void OdriveSetTrapTrajMaxVel(Odrv_t* odrive,float trap_traj_max_velocity)
{
	uint8_t TxBuffer[8]={0};

	memcpy(&TxBuffer[0],&trap_traj_max_velocity,4);

	OdriveSendCAN(odrive,SET_TRAP_TRAJ_VEL_LIMIT,TxBuffer);
	odrive->trap_traj_param.velocity=trap_traj_max_velocity;
}
/*
 * Function Name			: OdriveAbsoluteTurn
 * Function Description  : This function is called to turn odrive motor n number of turns, float number is also acceptable
 * Function Arguments	: odrive ,structure pointer to Odrv_t
 *                         count_num,number of turns to count
 * 						  input_mode, requested_input_mode
 * Function Return		: None
 * Function Example		:OdriveAbsoluteTurn(&odrive, 1.3,TRAPEZOIDAL_TRAJECTORY);
 */

void OdriveAbsoluteTurn(Odrv_t* odrive, float count_num,InputMode input_mode)//turn number of turns
{
	odrive->stop=odrive->hand_brake=0;
	uint8_t TxBuffer[8]={0};
	if(input_mode!=PASSTHROUGH&&input_mode!=POS_FILTER&&input_mode!=TRAPEZOIDAL_TRAJECTORY)
		input_mode=TRAPEZOIDAL_TRAJECTORY;
	memcpy(&TxBuffer[0],&count_num,4);
	OdriveSetControlInputMode(odrive,POSITION_CONTROL,input_mode);//make sure control mode is position control
	OdriveSendCAN(odrive,SET_INPUT_POS,TxBuffer);

	if(input_mode!=TRAPEZOIDAL_TRAJECTORY)
		return;

	while(odrive->feedback.traj_state==TRAJECTORY_DONE);//if using small movements that require less than 100ms, remember to adjust heartbeat rate

	odrive->turning = 1;

}


void OdriveSetRPSandCurrentMax(Odrv_t* odrive,float rps_lim,float current_lim)
{
	uint8_t TxBuffer[8]={0};
	memcpy(&TxBuffer[0],&rps_lim,4);
	memcpy(&TxBuffer[4],&current_lim,4);
	OdriveSendCAN(odrive,SET_LIMITS,TxBuffer);
}

void OdriveSetTrapTrajAccelDecelMax(Odrv_t* odrive,float accel_limit,float decel_limit)
{
	uint8_t TxBuffer[8]={0};
	memcpy(&TxBuffer[0],&accel_limit,4);
	memcpy(&TxBuffer[4],&decel_limit,4);
	OdriveSendCAN(odrive,SET_TRAP_TRAJ_ACCEL_LIMIT,TxBuffer);
	odrive->trap_traj_param.accel=accel_limit;
	odrive->trap_traj_param.decel=decel_limit;

}

void OdriveSendCAN(Odrv_t* odrive,uint8_t command,uint8_t* buffer){

	CAN_TxMsg(odrive->hcanx,odrive->Instance<<5 | ((uint16_t)command),buffer,8);

}

void OdriveSendRTRCAN(Odrv_t* odrive,uint8_t command){
	CAN_TxRTR(odrive->hcanx, odrive->Instance<<5 |((uint16_t)command));
}

void OdriveGetEncoderFeedback(Odrv_t* odrive)
{
	OdriveSendRTRCAN(odrive,GET_ENCODER_ESTIMATE);
}

void OdriveGetSensorlessEstimates(Odrv_t* odrive)
{
	OdriveSendRTRCAN(odrive,GET_SENSORLESS_ESTIMATES);
}

void OdriveGetShadowCount(Odrv_t* odrive)
{
	OdriveSendRTRCAN(odrive,GET_SHADOW_COUNT);
}

void OdriveGetIQValue(Odrv_t* odrive)
{
	OdriveSendRTRCAN(odrive,GET_IQ);
}

void OdriveGetBusVoltage(Odrv_t* odrive)
{
	OdriveSendRTRCAN(odrive,GET_BUS_VOLTAGE);
}

void OdriveRPStoVelocity(Odrv_t* odrive)
{
	float wheel_radius=odrive->wheel_diameter/2;
	odrive->feedback.velocity=wheel_radius*PI*odrive->feedback.encoder;
}


void OdriveSetPosGain(Odrv_t* odrive,float gains)
{
	uint8_t TxBuffer[8]={0};
	memcpy(&TxBuffer[0],&gains,4);
	OdriveSendCAN(odrive,SET_POS_GAIN,TxBuffer);
}

void OdriveSetVelGain(Odrv_t* odrive,float vel_gains,float vel_integrator_gains)
{
	uint8_t TxBuffer[8]={0};
	memcpy(&TxBuffer[0],&vel_gains,4);
	memcpy(&TxBuffer[4],&vel_integrator_gains,4);
	OdriveSendCAN(odrive,SET_VEL_GAIN,TxBuffer);
}

const char* Odrive_Error_To_String(error_code fault) {
	switch (fault) {
	case NO_ERROR: return "NO_ERROR";
	case INITIALIZING: return "INITIALIZING";
	case SYSTEM_LEVEL: return "SYSTEM_LEVEL";	//firmware bug / system error: memory corruption, stack overflow, frozen thread
	case TIMING_ERROR: return "TIMING_ERROR";
	case MISSING_ESTIMATE: return "MISSING_ESTIMATE";
	case BAD_CONFIG: return "BAD_CONFIG";
	case DRV_FAULT: return "DRV_FAULT";
	case MISSING_INPUT: return "MISSING_INPUT";
	case DC_BUS_OVER_VOLTAGE: return "DC_BUS_OVER_VOLTAGE";
	case DC_BUS_UNDER_VOLTAGE: return "DC_BUS_UNDER_VOLTAGE";
	case DC_BUS_OVER_CURRENT: return "DC_BUS_OVER_CURRENT";
	case DC_BUS_OVER_REGEN_CURRENT: return "DC_BUS_OVER_REGEN_CURRENT";
	case CURRENT_LIMIT_VIOLATION: return "CURRENT_LIMIT_VIOLATION";
	case MOTOR_OVER_TEMP: return "MOTOR_OVER_TEMP";
	case INVERTER_OVER_TEMP: return "INVERTER_OVER_TEMP";
	case VELOCITY_LIMIT_VIOLATION: return "VELOCITY_LIMIT_VIOLATION";
	case POSITION_LIMIT_VIOLATION: return "POSITION_LIMIT_VIOLATION";
	case WATCHDOG_TIMER_EXPIRED: return "WATCHDOG_TIMER_EXPIRED";
	case ESTOP_REQUESTED: return "ESTOP_REQUESTED";
	case SPINOUT_DETECTED: return "SPINOUT_DETECTED";
	case OTHER_DEVICE_FAILED: return "OTHER_DEVICE_FAILED";
	case CALIBRATION_ERROR: return "CALIBRATION_ERROR";
	}

	return "Unknown fault";
}

void OdriveAngle(Odrv_t* odrive, float angle){
	double tmp,frac=modf((double)angle,&tmp);//points stored in frac
	int temp=(int)tmp;
	temp=temp%360;
	float pos=temp+frac;
	OdriveAbsoluteTurn(odrive, pos, TRAPEZOIDAL_TRAJECTORY);
}

void OdriveBlockingTurnAbs(Odrv_t* odrive,float turn_count)
{
	uint8_t TxBuffer[8]={0};
	if(odrive->type==DENG_FOC){
		OdriveEnquire(odrive,TURN_COUNT_AND_VELOCITY);
		while(odrive->busy[TURN_COUNT_AND_VELOCITY]);
	}
	memcpy(&TxBuffer[0],&turn_count,4);
	OdriveSetControlInputMode(odrive,POSITION_CONTROL,TRAPEZOIDAL_TRAJECTORY);
	OdriveSendCAN(odrive,SET_INPUT_POS,TxBuffer);
	odrive->turning=1;

	while(odrive->turning);

}



void OdriveCAN_Handler()
{
	static int c[2];
	uint16_t command_mask = 0x7E0;  // mask first 7 bits, get only the id
	uint16_t id = CAN1RxMessage.StdId&command_mask;
	id=id>>5;
	int i=0;
	for( i=0 ; i < number_of_odrive; i++)
	{
		if(id == P_to_Odrive[i]->Instance)
		{
			decode_Odrive(P_to_Odrive[i]);
			c[i]++;
			break;
		}
		//		if(id == Odrv1.Instance)
		//		{
		//			decode_Odrive(&Odrv1);
		//			break;
		//		}
		//		if(id == Odrv2.Instance)
		//		{
		//			decode_Odrive(&Odrv2);
		//			break;
		//		}
	}

	if(c[0]==100)
	{
		c[0]=0;
		GPIOC_OUT->bit14=!GPIOC_OUT->bit14;
	}

	if(c[1]==100)
	{
		c[1]=0;
		GPIOC_OUT->bit15=!GPIOC_OUT->bit15;
	}

}

