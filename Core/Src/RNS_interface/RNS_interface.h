
/*******************************************************************************
 * Title   : ROBOT NAVIGATION SYSTEM INTERFACE
 * Author  : PROGRAMMING DEPARTMENT 2018 & 2020
 * Version : 1.00
 * Date    : 23 JANUARY 2018
 *******************************************************************************
 * Description: Interface with RNS
 *
 * Version History:
 *  1.0 - converted to HAL library
 *
 *
 * Bugs:
 *
 ******************************************************************************/

#ifndef RNS_INTERFACE_RNS_INTERFACE_H_
#define RNS_INTERFACE_RNS_INTERFACE_H_

#include "instruction.h"
#include "../BIOS/bios.h"
//#include "I2C/i2c.h"
#include "../CAN/can.h"

/* To change the communication method user need to change the define below
 * #define USED_CAN for communication using CAN
 * #define USED_I2C for communication using I2C
 */



#define USED_CAN

/*********************************************/
/*          Variable                         */
/*********************************************/
typedef struct{

#if defined USED_I2C
	char rns_address;
	I2C_TypeDef *rns_I2Cx;
#elif defined USED_CAN
	CAN_HandleTypeDef* rns_hcanx;
#endif

	struct{
		unsigned char instruction;
		union{
			float data;
			struct{
				char byte1;
				char byte2;
				char byte3;
				char byte4;
			};
		}ins_buffer[4];
	}ins;

	struct{
		unsigned char parameter;
		union{
			float data;
			struct{
				char byte1;
				char byte2;
				char byte3;
				char byte4;
			};
		}param_buffer[8];
	}param;

	struct{
		unsigned char enquiry;
		union{
			float data;
			struct{
				char byte1;
				char byte2;
				char byte3;
				char byte4;
			};
		}enq_buffer[4];
	}enq;


	struct{
		unsigned char common_instruction;
		union{
			float data;
			struct{
				char byte1;
				char byte2;
				char byte3;
				char byte4;
			};
		}common_buffer[6];
	}RNS_data;

	struct{
		uint8_t buf1;
		uint8_t buf2;
		uint8_t buf3;
		uint8_t buf4;
	}cont;

	struct{
		uint8_t Feedback;
		union{
			uint8_t all;
			struct{
				uint8_t x : 1;
				uint8_t y : 1;
				uint8_t z : 1;
			};
		}flag;
	}RNS_feedback;
	uint8_t busy;
}RNS_interface_t;

#if defined USED_CAN

typedef struct{
	unsigned char common_instruction;
	union{
		float data;
		struct{
			char byte1;
			char byte2;
			char byte3;
			char byte4;
		};
	}common_buffer[4];
}comm_t;

comm_t canRx_buf;

//Enumeration for can StdId
enum {

	RNS_TO_mainboard = 20,
	RNS_TO_mainboard_buf1,
	RNS_TO_mainboard_buf2,
	mainboard_TO_RNS,
	mainboard_TO_RNS_buf1,
	mainboard_TO_RNS_buf2,
	mainboard_TO_RNS_buf3,
	mainboard_TO_RNS_buf4,
	XY_feedback_state,
	CONT_data

};

#endif

/*********************************************/


/*********************************************/
/*          Define                           */
/*********************************************/
#define NONE	1000.0
/*********************************************/


/*********************************************/
/*          Enumarator                       */
/*********************************************/
enum {FORWARD = 0, REVERSE};
enum {START_END = 0, START, CONT, END};
enum {failed=0 , success};
enum {ONAV_X_GAIN, ONAV_Y_GAIN, ONAV_Z_GAIN, ONAV_LF_GAIN};
/*********************************************/
extern uint8_t insData_send[2];

/*********************************************/
/*           Function Prototype              */
/*********************************************/

#if defined USED_CAN
	void RNSInit(CAN_HandleTypeDef* hcanx, RNS_interface_t* rns);
#elif defined USED_I2C
	void RNSInit(char id, I2C_TypeDef *I2Cx, RNS_interface_t* rns);
#endif

void RNSStop(RNS_interface_t* rns);
void RNSVelocity(float f_left_velocity, float f_right_velocity, float b_left_velocity, float b_right_velocity,RNS_interface_t* rns);
void RNSVelocity1(float fFLeftVelR, RNS_interface_t* rns);
void RNSVelocity2(float fFRightVelR, RNS_interface_t* rns);
void RNSVelocity3(float fBLeftVelR, RNS_interface_t* rns);
void RNSVelocity4(float fBRightVelR, RNS_interface_t* rns);
void RNSPDC(float f_left_pdc, float f_right_pdc, float b_left_pdc, float b_right_pdc, RNS_interface_t* rns);
void RNSLFDist( dir_t Dir, float LF_vel, float LF_dist,RNS_interface_t* rns);
void RNSLFJunc(dir_t Dir, float LF_vel, float LF_dist, float LF_junc,RNS_interface_t* rns);
void RNSIMURotate(int AngleDeg,RNS_interface_t* rns);
void RNSODNStart(float velocity1, float x1, float y1, float z1, float velocity2, float xf, float yf, float zf, int no_point, RNS_interface_t* rns);
void RNSSendIns(RNS_interface_t* mcm);
void RNSODNGoto(float allpoints[][4],int no_point,RNS_interface_t* rns);
void RNSSet(RNS_interface_t* rns, unsigned char parameter, ...);
uint8_t RNSEnquire(unsigned char parameter, RNS_interface_t* rns);
void RNSODNKSet(unsigned char path_ctrl, unsigned char pos_ctrl, unsigned char a, float value,RNS_interface_t* rns);

void RNS_CONT(RNS_interface_t* rns, uint8_t psxJoy_x, uint8_t psxJoy_y, uint8_t psxJoyL_2,uint8_t psxJoyR_2);
void RNSPPstart(float allpoints[][7],int no_point,RNS_interface_t* rns);
void RNSSync (RNS_interface_t* rns);
/*********************************************/

#endif /* RNS_INTERFACE_RNS_INTERFACE_H_ */
