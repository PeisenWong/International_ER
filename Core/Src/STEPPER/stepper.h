/************************************************
 * Title   : Stepper
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 4/21/2021
 * **********************************************
 * Descriptions: Control Stepper Motor
 *
 *
 * Version History:
 * 1.0 -
 *
 * Bugs:
 *
 ************************************************/
#ifndef STP_H_
#define STP_H_

#include "../BIOS/bdc.h"

typedef struct{

	int total_steps;
	float step_count;
	float step_target;
	float step_1_angle;
	int step_dir;
	int isturning;
	int period;

	BDC_t *Step_BDC;

}STP_t;


void StepperInit(STP_t *stpx,BDC_t *BDC,TIM_HandleTypeDef* htimx, uint32_t Channel, int Period, int Rev_steps, BCDDIRPINType PinType, ...);
void StepperMoveSteps(STP_t *stpx,int Steps);
void StepperMovetoStep(STP_t *stpx,int Step);
void StepperMovetoAngle(STP_t *stpx,float Angle);
void StepperUpdate(STP_t *stpx);
void StepperResetCount(STP_t *stpx);
void StepperStop(STP_t *stpx);
#endif
