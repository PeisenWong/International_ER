/*******************************************************************************
 * Title   : adapter.h
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 12/10
 *******************************************************************************
 * Description: includes all the important includes and pin definitions
 *
 * Version History:
 *  1.0 - converted to HAL library
 *
 * Bugs:
 *
 ******************************************************************************/

#ifndef SRC_ADAPTER_H_
#define SRC_ADAPTER_H_


/* Private variables ---------------------------------------------------------*/


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "BIOS/bios.h"
#include <math.h>
#include "CAN/can.h"
#include "MODN/MODN.h"
#include "PID/PID.h"
#include "ABT/ABT.h"
#include "I2C/i2c.h"
#include "SPI/SPI.h"
#include "PSx_Interface/PSx_Interface.h"
#include "RNS_interface/RNS_interface.h"
#include "ADC/adc.h"
#include "SERVO/servo.h"
#include "KF/KF.h"
#include "LASER/laser.h"
#include "STEPPER/stepper.h"
#include "SERVO_DRIVER/servo_driver.h"
#include "Moving_Average/mov_ave.h"
#include "VESC_CAN/vesc_interface.h"
#include "Eeprom/eeprom.h"
#include "ILI9341/ILI9341_Driver.h"
#include "Dongbei/dongbei.h"
#include "RGB/rgb.h"
#include "TuningInterface/TuningInterface.h"
#include "Tune.h"
#include "ROS_Interface/ROS_Interface.h"
#include "ROS_navi/ROS_navi.h"
#include "Odrive/odrive.h"
#include "FaulHaber/FB.h"
#include "R6091U/r6091u.h"
#include "PP/PP.h"
#include "obstacle_detect/obstacle_detect.h"
/* Exported functions prototypes ---------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define LED1_PIN			GPIOC, GPIO_PIN_13
#define LED2_PIN			GPIOC, GPIO_PIN_14
#define LED3_PIN			GPIOC, GPIO_PIN_15

#define PB1_PIN				GPIOB, GPIO_PIN_7
#define PB2_PIN				GPIOE, GPIO_PIN_0

#define IP1_PIN				GPIOE, GPIO_PIN_12              /* TIM1_CH3N                                              */
#define IP2_PIN				GPIOE, GPIO_PIN_13              /* TIM1_CH3                                               */
#define IP3_PIN				GPIOE, GPIO_PIN_14              /* TIM1_CH4                                               */
#define IP4_PIN				GPIOE, GPIO_PIN_15              /* TIM1_BKN                                               */
#define IP5_PIN				GPIOB, GPIO_PIN_14              /* TIM12_CH1                                              */
#define IP6_PIN				GPIOB, GPIO_PIN_15              /* TIM12_CH2                                              */
#define IP7_PIN				GPIOD, GPIO_PIN_10              /*                                                        */
#define IP8_PIN				GPIOD, GPIO_PIN_11              /*                                                        */
#define IP9_PIN				GPIOC, GPIO_PIN_8               /* TIM8_CH3                                               */
#define IP10_PIN			GPIOA, GPIO_PIN_10              /* TIM1_CH3                                               */
#define IP11_PIN			GPIOD, GPIO_PIN_3               /*                                                        */
#define IP12_PIN			GPIOD, GPIO_PIN_4               /*                                                        */
#define IP13_PIN			GPIOD, GPIO_PIN_7               /*                                                        */
#define IP14_PIN			GPIOB, GPIO_PIN_5               /*                                                        */
#define IP15_PIN			GPIOB, GPIO_PIN_6               /* TIM4_CH1                                               */

#define IP16_Analog1_PIN	GPIOC, GPIO_PIN_0
#define IP17_Analog2_PIN	GPIOC, GPIO_PIN_1
#define IP18_Analog3_PIN	GPIOC, GPIO_PIN_2
#define IP19_Analog4_PIN	GPIOC, GPIO_PIN_3
#define IP20_Analog5_PIN	GPIOC, GPIO_PIN_4
#define IP21_Analog6_PIN	GPIOC, GPIO_PIN_5

#define QEI1_PLUSEA_PIN		GPIOE , GPIO_PIN_9
#define QEI1_PLUSEB_PIN		GPIOE , GPIO_PIN_11

#define QEI4_PLUSEA_PIN		GPIOD , GPIO_PIN_12
#define QEI4_PLUSEB_PIN		GPIOD , GPIO_PIN_13

#define QEI6_PLUSEA_PIN		GPIOC , GPIO_PIN_6
#define QEI6_PLUSEB_PIN		GPIOC , GPIO_PIN_7

#define TIM5_CHANNEL1_PIN	GPIOA, GPIO_PIN_0
#define TIM5_CHANNEL2_PIN	GPIOA, GPIO_PIN_1
#define TIM5_CHANNEL3_PIN	GPIOA, GPIO_PIN_2
#define TIM5_CHANNEL4_PIN	GPIOA, GPIO_PIN_3

#define TIM9_CHANNEL1_PIN	GPIOE, GPIO_PIN_5
#define TIM9_CHANNEL2_PIN	GPIOE, GPIO_PIN_6

#define TIM3_ChANNEL1_PIN   GPIOA, GPIO_PIN_6
#define TIM3_CHANNEL3_PIN	GPIOB, GPIO_PIN_0
#define TIM3_CHANNEL4_PIN	GPIOB, GPIO_PIN_1

#define MUX1_INPUT_PIN 		GPIOE , GPIO_PIN_1
#define MUX1_S0_PIN 		GPIOE , GPIO_PIN_2
#define MUX1_S1_PIN 		GPIOE , GPIO_PIN_3
#define MUX1_S2_PIN 		GPIOE , GPIO_PIN_4

#define SR_SCK_PIN			GPIOE , GPIO_PIN_7
#define SR_RCK_PIN			GPIOE , GPIO_PIN_8
#define SR_SI_PIN			GPIOE , GPIO_PIN_10

#define SPI1_NSS_PIN		GPIOA, GPIO_PIN_4
#define SPI1_SCK_PIN		GPIOA, GPIO_PIN_5
#define SPI1_MISO_PIN		GPIOA, GPIO_PIN_6
#define SPI1_MOSI_PIN		GPIOA, GPIO_PIN_7

#define UART2_Tx			GPIOD , GPIO_PIN_5
#define UART2_Rx			GPIOD , GPIO_PIN_6

#define UART3_Tx			GPIOD , GPIO_PIN_9
#define UART3_Rx			GPIOD , GPIO_PIN_8

#define UART4_Tx			GPIOC , GPIO_PIN_10
#define UART4_Rx			GPIOC , GPIO_PIN_11

#define UART5_Tx			GPIOC , GPIO_PIN_12
#define UART5_Rx			GPIOD , GPIO_PIN_2

#define CAN1_Tx				GPIOD , GPIO_PIN_1
#define CAN1_Rx				GPIOD , GPIO_PIN_0

#define CAN2_Tx				GPIOB , GPIO_PIN_13
#define CAN2_Rx				GPIOB , GPIO_PIN_12


MUX_t MUX;
shiftreg_t SR;
RNS_interface_t rns;
BDC_t BDC1, BDC2, BDC3, BDC4, BDC5, BDC6, BDC7, BDC8;
uint8_t insData_receive[2];
PSxBT_t ps4;
ABT_t filter;
ADC_t adc;
//LASER_t r_laser,l_laser;
KALMANFILTER_t kf_adc_r,kf_adc_l,kf_pres;
PID_t pid_laser_R,pid_laser_L,pid_pres,pid_z;
Srv_Drv_t srv_drv;
Mov_Ave_t mov_l_r,mov_l_l;
SERVO_t vesc_ppm[3], servo, servo1, regulator, servo_ring, servo_ring_new;
R6091U_t IMU, PITCH_IMU;
ABT_t x_data, y_data;
PathPlan_t pp;
PID_t pick_PID, left_PID, right_PID;


#define PB1 		GPIOB_IN->bit7
#define PB2 		GPIOE_IN->bit0

#define led1		GPIOC_OUT->bit13
#define led2		GPIOC_OUT->bit14
#define led3		GPIOC_OUT->bit15
#define led4_on 	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 19999); // BDC 5
#define led4_off 	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
#define led6_on 	SR.cast[0].bit6 = 1;
#define led6_off 	SR.cast[0].bit6 = 0;
#define led5_on 	SR.cast[0].bit7 = 1;
#define led5_off 	SR.cast[0].bit7 = 0;
#define led7_on 	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 19999); // BDC 6
#define led7_off 	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
#define led9		SR.cast[0].bit4
#define led8		SR.cast[0].bit5

//Global Declarations
float v1, v2, v3, v4, wa, xr, yr;                 //MODN variables
//float xpos, ypos, z;                              //Encoder Values
float a, b, c, d, pa, pb, pc, pd;  				  //Enquiry Values
volatile uint16_t  adc1_buf[7];
volatile uint16_t  adc2_buf[7];
volatile uint16_t  adc3_buf[7];

int counter, run, shooting;                                      //global timer variable
float speed;                                      //for manual control

uint32_t servo_pulse;
float vesc_duty;
uint8_t position[12], debug[300], ack;
int acks, vesc_start, vesc_stop;

float fx_accel, fy_accel, fz_accel, roll, pitch, yaw, vesc_extra;
float moveSpeed, vesc_duty;
int mode, before;
int shoot_start, shoot_done, load_start, pick_start, set_pitch, pick_left, pick_right, set_pick_enc, vel_adjust, reload, load_stop_once, wait_load;
int picked_left, picked_right, loaded, servo_close_once, stick_fence;
int tune_p, tune_i, tune_d, wheel;
int led_enb, before_shot, shot_prd;
int pick_0, picked_manual, pick_left_manual, load_adjust, adjust_count, up_done, type_3_done, blue, go_type_3, shooted, pick_right_enb, go_A;
int start_flywheel, cylinder_load_once, must_load, servo_enb, stop_adjust, vesc_start, angle_shoot, shot_count, manual_adjust, extra_rpm;


float fXEncData, fYEncData;
float fXPos, fYPos;			/* Position, ABT output, pos PID feedback */
float fXVel, fYVel;			/* Velocity, ABT output */
float fXAcc, fYAcc;			/* Acceleration, ABT output*/
float fyaw;
//float lsrL, lsrR;
float xpos, ypos, z;
float pickErr, pickU, pickR;
float leftErr, leftU;
float rightErr, rightU;
float vesc_speed;
extern float pickVG[7], flywheelVG[7];
int32_t pick_enc, pick_tol, pick_target_enc, pick_enc_buf, RedPickLess, BluePickLess, RedPickMore, BluePickMore;



union{
	float data;
	struct{
		char byte1;
		char byte2;
		char byte3;
		char byte4;
	};
}buf1_receive[2];
union{
	float data;
	struct{
		char byte1;
		char byte2;
		char byte3;
		char byte4;
	};
}buf2_receive[2];

typedef enum{
	RNS_PACKET,
	VESC_PACKET,
	ODRIVE_PACKET
}PACKET_t;

uint32_t faul_counter;
struct{
	uint16_t statusword;
	int pos_act;
	int vel_act;
	int16_t tor_act;
	uint16_t ODindex;
	uint8_t ODsubindex;
	uint8_t num_valid;
	uint8_t buffer[4];
	int HomeOffset;
	uint32_t velKp;
	int8_t modeOperation;
	union{
		uint16_t flags;
		struct{
			unsigned can    	    :1;
			unsigned flag1          :1;
			unsigned flag2    		:1;
			unsigned flag3          :1;
			unsigned flag4          :1;
			unsigned flag5          :1;
			unsigned flag6          :1;
			unsigned flag7          :1;
			unsigned flag8          :1;
			unsigned flag9          :1;
			unsigned flag10         :1;
			unsigned flag11         :1;
			unsigned flag12         :1;
			unsigned flag13 	    :1;
			unsigned flag14         :1;
			unsigned flag15        	:1;
		};
	};
}Faul_t;
void CAN_PROCESS(PACKET_t packet_src);
void Initialize (void);

#ifdef __cplusplus
}
#endif
#endif /* SRC_ADAPTER_H_ */
