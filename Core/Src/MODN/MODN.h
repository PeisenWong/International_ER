#ifndef MODN_H
#define MODN_H



/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "../PID/PID.h"
#include "../PSx_Interface/PSx_Interface.h"
#include "../RNS_interface/RNS_interface.h"
/*********************************************/




/*********************************************/
/*          Define                           */
/*********************************************/

/*********************************************/




/*********************************************/
/*          Enumarator                       */
/*********************************************/
typedef enum{
	MODN_FWD_OMNI,
	MODN_TRI_OMNI,
	MODN_MECANUM
}RobotBaseType_t;

typedef enum{
	OPERATOR_TURNED_0_DEGREE,
	OPERATOR_TURNED_90_DEGREES_CLOCKWISE,
	OPERATOR_TURNED_180_DEGREES,
	OPERATOR_TURNED_90_DEGREES_ANTICLOCKWISE
}OrientationMODN_t;
/*********************************************/




/*********************************************/
/*          Variable                         */
/*********************************************/
struct {
	float x_vel;
	float y_vel;
	float w_vel;
	float vel1;
	float vel2;
	float vel3;
	float vel4;
	float speed;
	float d;
	float e;
	RobotBaseType_t base;
	uint8_t turnState;
	float radTol;
	float imuGain;
	float imuFeedback;
	float radTarget;
	PSxBT_t* psx;
	int orientation;
	float *real_x_vel;
	float *real_y_vel;
}MODN;

enum{
	NO_TURN,
	START_TURN
}TurnState;
/*********************************************/


/*********************************************/
/*           Function Prototype              */
/*********************************************/
void MODNInit(RobotBaseType_t base, float speed, float turnSpeed, float angleTol, float imuGain);
void LegacyMODN(PSxBT_t *psx, RNS_interface_t* rns);
void realMODN(PSxBT_t *psx, RNS_interface_t* rns);
void imuRealMODN(PSxBT_t *psx, RNS_interface_t* rns);
void movableRealMODN(PSxBT_t *psx, RNS_interface_t* rns);
void setOrientationMODN(OrientationMODN_t orientation);
void setSpeedMODN(float speed);
void setImuGainMODN(float imuGain);
/*********************************************/
#endif
