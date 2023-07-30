/*
 * odriver.h
 *
 *  Created on: Oct 6, 2022
 *      Author: heheibhoi
 */
#ifndef SRC_ODRIVE_ODRIVER_H_
#define SRC_ODRIVE_ODRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif


// Code from Anas
#include "../adapter.h"

typedef struct{
	UART_HandleTypeDef *huartx;
	char buffer[100];
} Odrive_t;

void Odrive_Init(Odrive_t *Odrive, UART_HandleTypeDef *huartx);
void Odrive_Pos(Odrive_t *Odrive, int motor_number, float position, float torque_limit, float vel_limit);
void Odrive_Vel(Odrive_t *Odrive, int motor_number, float velocity, float torque);
void Odrive_Curr(Odrive_t *Odrive, int motor_number, float current);
void Odrive_Traj(Odrive_t *Odrive, int motor_number, float position);
float Odrive_GetVelocity(Odrive_t *Odrive, int motor_number);
float Odrive_GetPosition(Odrive_t *Odrive, int motor_number);
float Odrive_Read(Odrive_t *Odrive);
void Odrive_Idle(Odrive_t *Odrive, int motor_number);
void Odrive_ClearErrors(Odrive_t *Odrive);
void Odrive_ClosedLoop(Odrive_t *Odrive, int motor_number);
void Odrive_Vel_Limit(Odrive_t *Odrive, int motor_number, float vel_limit);

// End code from Anas
#include "../CAN/can.h"
#include <stdbool.h>
#include <string.h>
#include <math.h>
#define PI 3.142857
#define BRAKE_PERIOD 500
#define TRAJECTORY_DONE 128
#define TRAJECTORY 0


#define O_DRV_TIMEOUT 200

char odrivefaultcode[128];

typedef enum
{
	BUS_VOLTAGE,
	TURN_COUNT_AND_VELOCITY,
	MOTOR_AND_INVERTER_TEMPERATURE,
	SHADOW_COUNT,
	IQ_VALUE,
	ODRIVE_ERROR_CODE,
	SENSORLESS_ESTIMATE,
}Feedback;

typedef enum
{
	ENCODER_MODE_INCREMENTAL,
	ENCODER_MODE_SPI_ABS_AMS,
	ENCODER_MODE_HALL,
}Encoder_mode;

typedef enum
{
	DENG_FOC,
	ODRV3
}Odrive_Type;

enum Commands{
	HEARTBEAT=1,
	E_STOP,
	GET_MOTOR_ERROR,
	GET_ENCODER_ERROR,
	GET_SENSORLESS_ERROR,
	SET_AXIS_NODE_ID=6,
	SET_AXIS_STATE=7,
	GET_ENCODER_ESTIMATE=9,
	GET_SHADOW_COUNT,
	SET_CONTROLLER_MODES,
	SET_INPUT_POS,
	SET_INPUT_VEL,
	SET_INPUT_TORQUE,
	SET_LIMITS,//set current limit and rps limit, rps byte 0-3, current 4-7
	SET_TRAP_TRAJ_VEL_LIMIT=17,
	SET_TRAP_TRAJ_ACCEL_LIMIT,//accel limit at lower 4 byte
	SET_TRAP_TRAJ_INERTIA,
	GET_IQ,
	GET_SENSORLESS_ESTIMATES,
	REBOOT_ODRIVE,
	GET_BUS_VOLTAGE,
	CLEAR_ERROR,
	SET_LINEAR_COUNTS,
	SET_POS_GAIN,
	SET_VEL_GAIN,
};//basic commands

typedef enum {
	VOLTAGE_CONTROL=0,
	TORQUE_CONTROL=1,
	VELOCITY_CONTROL,
	POSITION_CONTROL
}ControlMode;//control mode state

typedef enum {
	IDLE_STATE=1,
	STARTUP_SEQUENCE,
	FULL_CALIBRATION_SEQUENCE,
	MOTOR_CALIBRATION,
	ENCODER_INDEX_SEARCH=6,
	ENCODER_OFFSET_CALIBRATION,
	CLOSED_LOOP_CONTROL,
	LOCKIN_SPIN,
	ENCODER_DIR_FIND,
	HOMING,
	ENCODER_HALL_POLARITY_CALIBRATION,
	ENCODER_HALL_PHASE_CALIBRATION,
}AxisRequestedState;//axis requested state

typedef enum {
	AXIS_IDLE_STATE=1,
	AXIS_CLOSED_LOOP_CONTROL=8,
}AxisCurrentState;//axis  state

typedef enum {
	PASSTHROUGH=1,
	VEL_RAMP,
	POS_FILTER,
	TRAPEZOIDAL_TRAJECTORY=5,
	TORQUE_RAMP,
	MIRROR,
}InputMode;//input mode

typedef enum{
	NO_ERROR					= 0x00,
	INITIALIZING 				= 0x01,
	SYSTEM_LEVEL 				= 0x02,	//firmware bug / system error: memory corruption, stack overflow, frozen thread
	TIMING_ERROR 				= 0x04,
	MISSING_ESTIMATE			= 0x08,	//enc not calibrated / Abs pos ctrl used b4 axis homed / enc misbehaving or dc
	BAD_CONFIG 					= 0x10,
	DRV_FAULT					= 0x20,
	MISSING_INPUT				= 0x40,
	DC_BUS_OVER_VOLTAGE			= 0x100,
	DC_BUS_UNDER_VOLTAGE		= 0x200,
	DC_BUS_OVER_CURRENT			= 0x400,
	DC_BUS_OVER_REGEN_CURRENT	= 0x800,
	CURRENT_LIMIT_VIOLATION		= 0x1000,
	MOTOR_OVER_TEMP				= 0x2000,
	INVERTER_OVER_TEMP			= 0x4000,
	VELOCITY_LIMIT_VIOLATION	= 0x8000,
	POSITION_LIMIT_VIOLATION	= 0x10000,
	WATCHDOG_TIMER_EXPIRED		= 0x1000000,
	ESTOP_REQUESTED				= 0x2000000,
	SPINOUT_DETECTED			= 0x4000000,
	OTHER_DEVICE_FAILED			= 0x8000000,
	CALIBRATION_ERROR			= 0x40000000,
}error_code;



typedef struct {
	float decel;
	float accel;
	float inertia;
	float velocity;
}Odrv_trap_traj;

typedef struct {
	float position;
	float velocity;
}Odrv_sensorless;

typedef struct {
	float iq_setpoint;
	float iq_measured;
}Odrv_Iq;



typedef struct {
	CAN_RxHeaderTypeDef RXmsg;
	uint8_t Data[8];
}odrvmsg;

typedef struct {
	float velocity;
	float encoder;
	float round_per_second;
	float inverter_temperature;
	float motor_temperature;
	float bus_voltage;
	float shadow_count;
	float cpr;

	error_code error;
	char error_msg[20];

	Odrv_sensorless sensorless_estimates;
	Odrv_Iq Iq;
	AxisCurrentState state;
	error_code error_code;
	uint8_t traj_state;
}Odrv_feedback;

typedef struct{
	CAN_HandleTypeDef* hcanx;
	float wheel_diameter;
	float target;
	float trap_traj;
	uint8_t turning;
	uint8_t busy[5];
	uint8_t stop;
	uint8_t hand_brake;
	uint16_t Instance;
	uint32_t hand_brake_start;
	InputMode input_mode;
	ControlMode control_mode;
	AxisRequestedState current_state;
	Odrv_feedback feedback;
	Odrv_trap_traj trap_traj_param;
	Encoder_mode encoder_mode;
	Odrive_Type type;
	uint8_t waiting_for_heartbeat;
}Odrv_t;

Odrv_t** P_to_Odrive;
extern uint8_t Odrv_Commands[8];
uint8_t* Odrv_ID;
extern int number_of_odrive;
Odrv_t Odrv1,Odrv2;
int odrive_reached;

odrvmsg Odrvmsg;
void OdriveInit(Odrv_t* odrive, CAN_HandleTypeDef* hcanx, Odrive_Type type, uint16_t axis_id,ControlMode control_mode,InputMode input_mode);
void OdriveTurn( Odrv_t* odrive , float count_num , InputMode input_mode);
void OdriveHandBrake( Odrv_t* odrive);
void OdriveAbsoluteTurn( Odrv_t* odrive , float count_num , InputMode input_mode);
void OdriveVelocity(Odrv_t* odrive, float target_velocity,uint8_t input_mode);
void OdriveTorque(Odrv_t* odrive, float target_torque,uint8_t input_mode);
void decode_Odrive(Odrv_t* odrive);
void OdriveSetControlMode(Odrv_t* odrive,uint8_t control_mode);
void OdriveSetInputMode(Odrv_t* odrive,uint8_t input_mode);
void OdriveSetAxisRequestedState(Odrv_t* odrive,uint8_t AxisRequested_state);
void OdriveSetControlInputMode(Odrv_t* odrive,ControlMode requested_control_mode,InputMode requested_input_mode);
void OdriveClearError(Odrv_t* odrive);
void OdriveSetRPSandCurrentMax(Odrv_t* odrive,float rps_lim,float current_lim);
void OdriveRelease(Odrv_t* odrive);
void OdriveArm(Odrv_t* odrive);
void OdriveStop(Odrv_t* odrive);
void OdriveSetAbsolutePosition(Odrv_t* odrive);
void OdriveGetTemperatureFeedback(Odrv_t* odrive);
void OdriveGetEncoderFeedback(Odrv_t* odrive);
void OdriveGetShadowCountCPR(Odrv_t* odrive);
void OdriveUpdatePos(Odrv_t* odrive);
void OdriveSetInertia(Odrv_t* odrive,float inertia);
void OdriveTurnCountInertia(Odrv_t* odrive, float count_num,float inertia);
void OdriveEnquire(Odrv_t* odrive,uint8_t feedback);
void OdriveFeedBackCorrection(Odrv_t* odrive);
void OdriveSendCAN(Odrv_t* odrive,uint8_t command,uint8_t* buffer);
void OdriveSendRTRCAN(Odrv_t* odrive,uint8_t command);
void OdriveGetBusVoltageAndCurrent(Odrv_t* odrive);
void OdriveRPStoVelocity(Odrv_t* odrive);

void OdriveSetTrapTrajAccelDecelMax(Odrv_t* odrive,float accel_limit,float decel_limit);
void OdriveSetTrapTrajMaxVel(Odrv_t* odrive,float trap_traj_max_velocity);
void OdriveSetPosGain(Odrv_t* odrive,float gains);
void OdriveSetVelGain(Odrv_t* odrive,float vel_gains,float vel_integrator_gains);

void OdriveSetWheelDiameter(Odrv_t* odrive,float wheel_diameter);
const char* Odrive_Error_To_String(error_code fault);
void OdriveGetSensorlessEstimates(Odrv_t* odrive);
void OdriveGetShadowCount(Odrv_t* odrive);
void OdriveGetIQValue(Odrv_t* odrive);
void OdriveGetBusVoltage(Odrv_t* odrive);
void OdriveAngle(Odrv_t* odrive, float angle);
void OdriveBlockingTurnAbs(Odrv_t* odrive,float turn_count);
void OdriveBlockingTurn(Odrv_t* odrive,float turn_count);
void OdriveCAN_Handler();
void OdriveBlockingArm(Odrv_t* odrive);
void OdriveBlockingRelease(Odrv_t* odrive);
void OdriveRestart(Odrv_t* odrive);

#ifdef __cplusplus
}
#endif

#endif /* SRC_ODRIVE_ODRIVER_H_ */
