/*******************************************************************************
 * Title   : common.h
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 12/10
 *******************************************************************************
 * Description: Sensor and function definitions
 *
 * Version History:
 *  1.0 - converted to hal library
 *
 * Bugs:
 *
 ******************************************************************************/

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "adapter.h"
#include "cmsis_os.h"

// Blinking hz for modes
#define NormalMode 500
#define AutoMode 125
#define PoleSpeed 8.5
//#define Offset 0.55
//#define Offset_2 0.8
#define Offset 0.5
#define Offset_2 0.75
#define Offset_3 0.8
#define Offset_4 0.73
//#define PoleSpeed 2.5
//#define BluePickMore 0
//#define RedPickMore 0

// Speed
#define type1 3.7
#define BlueType2 8.15
#define RedType2 8.3
#define BlueType3 13.1
#define RedType3 13.25
#define BlueOppoType2 12.95
#define RedOppoType2 13.3

// Duty
#define type1Duty 890
#define BlueType2Duty 1525
#define RedType2Duty 1550
#define BlueType3Duty 1980
#define RedType3Duty  2015
#define BlueOppoType2Duty  2080
#define RedOppoType2Duty   2105
#define BlueType2RotateDuty 1745  //-66.67 deg,
#define RedType2RotateDuty 1760
#define BlueType3RotateDuty 2170
#define RedType3RotateDuty  2190
#define BlueOppoType2RotateDuty 2305
#define RedOppoType2RotateDuty 2330

#define SAMPLE_TIME 0.005f

#define IP1  		HAL_GPIO_ReadPin(IP1_PIN)
#define IP2  		HAL_GPIO_ReadPin(IP2_PIN)
#define IP3  		HAL_GPIO_ReadPin(IP3_PIN)
#define IP4			HAL_GPIO_ReadPin(IP4_PIN)
#define IP5 		HAL_GPIO_ReadPin(IP5_PIN)
#define IP6 		HAL_GPIO_ReadPin(IP6_PIN)
#define IP7		    HAL_GPIO_ReadPin(IP7_PIN)
#define IP8	 		HAL_GPIO_ReadPin(IP8_PIN)
#define IP9  		HAL_GPIO_ReadPin(IP9_PIN)
#define IP10    	HAL_GPIO_ReadPin(IP10_PIN)
#define IP11  		HAL_GPIO_ReadPin(IP11_PIN)
#define IP12 		HAL_GPIO_ReadPin(IP12_PIN)
#define IP13  		HAL_GPIO_ReadPin(IP13_PIN)
#define IP14 		HAL_GPIO_ReadPin(IP14_PIN)
#define IP15		HAL_GPIO_ReadPin(IP15_PIN)

//ANALOG PIN//
#define IP16	HAL_GPIO_ReadPin(IP16_Analog1_PIN)
#define IP17	HAL_GPIO_ReadPin(IP17_Analog2_PIN)
#define	IP18   HAL_GPIO_ReadPin(IP18_Analog3_PIN)
#define IP19	HAL_GPIO_ReadPin(IP19_Analog4_PIN)
#define IP20	HAL_GPIO_ReadPin(IP20_Analog5_PIN)
#define IP21	HAL_GPIO_ReadPin(IP21_Analog6_PIN)

// Sensors
#define In_Pick			IP1 == 1
#define In_Load			IP2 == 1
#define In_ShotReady 	IP3 == 0
#define In_ShotDone		IP4 == 0
#define In_Pitch45		IP5 == 0
#define In_Pitch70		IP6 == 0
#define In_LS_Left_1	IP7 == 0
#define In_LS_Left_2	IP8 == 0
#define In_LS_Right_1	IP9 == 0
#define In_LS_Right_2	IP10 == 0
#define In_LS_Shot_1	IP11 == 0
#define In_LS_Shot_2	IP12 == 0
#define In_Pick_0		IP13 == 0

// Actuator
#define cylinder_load		SR.cast[1].bit6 = 1;  // BDC1
#define cylinder_retract	SR.cast[1].bit6 = 0;
#define pitch_up			WriteBDC(&BDC3, -19999);
#define pitch_down			WriteBDC(&BDC3, 19999);
#define pitch_stop			WriteBDC(&BDC3, 0);
#define push_shoot			WriteBDC(&BDC4, 19999);
#define push_return			WriteBDC(&BDC4, -19999);
#define push_stop			WriteBDC(&BDC4, 0);

// BDC7 (old) + BDC8 (new)
#define open_servo			ServoSetPulse(&servo_ring, 2220); ServoSetPulse(&servo_ring_new, 800);
#define close_servo			ServoSetPulse(&servo_ring, 700); ServoSetPulse(&servo_ring_new, 2250);
#define load_adjust_servo	ServoSetPulse(&servo_ring, 940); ServoSetPulse(&servo_ring_new, 2000);
#define adjust_servo		ServoSetPulse(&servo_ring, 800); ServoSetPulse(&servo_ring_new, 2150);

#define pick_up				WriteBDC(&BDC2, -19999);
#define pick_down			WriteBDC(&BDC2, 19999);
#define pick_stop			WriteBDC(&BDC2, 0);
#define pick_manual(x)		WriteBDC(&BDC2, -x);

#define Mux1		 MUX.mux_data.bit0
#define Mux2		 MUX.mux_data.bit1
#define Mux3		 MUX.mux_data.bit2
#define Mux4		 MUX.mux_data.bit3
#define Mux5		 MUX.mux_data.bit4
#define Mux6		 MUX.mux_data.bit5
#define Mux7		 MUX.mux_data.bit6
#define Mux8		 MUX.mux_data.bit7

osThreadId_t MainTaskHandle;
osThreadId_t NaviTaskHandle;
osThreadId_t PitchTaskHandle;
osThreadId_t CheckingTaskHandle;
osThreadId_t LaserNavigateTaskHandle;
osThreadId_t EmergencyTaskHandle;
osThreadId_t SecondaryTaskHandle;
osThreadId_t TuneTaskHandle;
osThreadId_t FlywheelPitchPIDTaskHandle;
osThreadId_t FlywheelYawPIDTaskHandle;
osThreadId_t TestTaskHandle;

osSemaphoreId_t TuneSemaphore;

typedef union{
	uint16_t flags;
	struct{
		//Least significant 16 bits can be cleared all at once by
		//sys.flags = 0 for example during emergency
		unsigned manual       :1;
		unsigned tunePid      :1;
		unsigned pp_ready	  :1;
		unsigned pp_start     :1;
		unsigned rns_busy     :1;
		unsigned ros_test_start  :1;
		unsigned ros_path_start  :1;
		unsigned ros_stop		  :1;
		unsigned vel_ready        :1;
		unsigned stop        :1;
		unsigned pitch_cali	      :1;
		unsigned set_pitch		  :1;
		unsigned load_start		  :1;
		unsigned pick_start       :1;
		unsigned pick_adjust       :1;
		unsigned flag15       :1;

		//Most significant 16 bits are not clear
		unsigned flag16       :1;
		unsigned flag17       :1;
		unsigned flag18	      :1;
		unsigned flag19       :1;
		unsigned flag20       :1;
		unsigned flag21       :1;
		unsigned flag22       :1;
		unsigned flag23		  :1;
		unsigned flag24       :1;
		unsigned flag25       :1;
		unsigned flag26	      :1;
		unsigned flag27		  :1;
		unsigned flag28		  :1;
		unsigned flag29       :1;
		unsigned flag30       :1;
		unsigned flag31       :1;

		//A flag can use more than 1 bit
		//Example : Combine flag30 and flag31
		//unsigned flag29     :1;
		//unsigned flag30     :2;
		//the value of sys.flag30 range from 0 to 3 then overflow to 0 again and vice versa
		//the value of flag29 is not affected when flag30 overflow
	};
}sys_t;

sys_t sys;

enum
{
	NORMAL,
	AUTO,
	TUNE
};

typedef enum
{
	PITCH45,
	PITCH70
}PITCH_t;

PITCH_t Robot_Pitch;

void RNS_config(CAN_HandleTypeDef* hcanx);
void set(void);
void manual_mode(void);
void enq(void);
void setTargetPitch(float target);
void NormalControl();
void RobotStart();
void Checking();
void LoadRing(void);
void AutoLoadRing(void);
void ResetCoordinate(void);
void ResetPickEnc(void);
void StopAutoPP(void);
void Auto();
void setPick(int32_t target);
void CheckPickEnc(void);
void CheckLoad();
void CheckShoot();
void CheckPick0();
void flywheelStop();
void flywheelAct();
void AdjustRings(void);
void flywheelPID(float speed);
void Shot();
void CheckPitch();
void tune(void);
void setPitch(PITCH_t target_pitch);
void Await(uint32_t ticks);

#define NUM_INT_UPDATE		1
#define NUM_FLOAT_UPDATE	1

void ILI9341_Init_List(void);
void ILI9341_Update_List(void);
void calcSpeed(uint32_t);
void registerPath(float path[][7], int numPoint, ...);

struct{
	UART_HandleTypeDef* huart;
	uint8_t buffer[10];
	char lagoriColor;
	uint16_t lagoriHeight;
	uint16_t lagoriWidth;
	uint32_t lagoriArea;
}esp32;
#endif /* INC_COMMON_H_ */
