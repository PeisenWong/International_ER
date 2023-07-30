/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "stepper.h"


/************************************************/
/*		 	 	Functions		       		  	*/
/************************************************/


/*
 * Function Name		: StepperInit
 * Function Description : This function is called to initialize Stepper Motors with BDC.
 * Function Remarks		:
 *
 *
 * Function Arguments	: stpx   				Pointer to stepper handle
 * 						  BDC    				Pointer to BDC handle
 * 						  htimx         		Pointer to timer handle
 * 						  Channel       		Timer Channel
 * 						  Rev_steps     		Total steps per Revolution
 * 						  Dirpintype 			type of direction pin (GPIO or SHIFTREG)
 * 						  ...					direction pin
 * Function Return		: None
 * Function Example		: StepperInit(&stp1,&BDC1,&htim3,TIM_CHANNEL_1,400,SHIFTREG, &(SR.cast[1]), Bit6, Bit7);
 */
void StepperInit(STP_t *stpx,BDC_t *BDC,TIM_HandleTypeDef* htimx, uint32_t Channel, int Period, int Rev_steps, BCDDIRPINType PinType, ...)
{

	stpx->total_steps = Rev_steps;
	stpx->step_count = 0.0;
	stpx->step_target = 0.0;
	stpx->step_1_angle = (Rev_steps/360.0);
	stpx->period = Period;

	stpx->Step_BDC = BDC;

	stpx->Step_BDC->htim = htimx;
	stpx->Step_BDC->Channel = Channel;
	stpx->Step_BDC->Dirpintype = PinType;

	va_list pinconfig;
	va_start(pinconfig, PinType);

	if( stpx->Step_BDC->Dirpintype == GPIO){

		stpx->Step_BDC->GPIOx_DIR1 = va_arg(pinconfig, GPIO_TypeDef*);
		stpx->Step_BDC->GPIO_Pin_DIR1	= va_arg(pinconfig, int);
		stpx->Step_BDC->GPIOx_DIR2 = va_arg(pinconfig, GPIO_TypeDef*);
		stpx->Step_BDC->GPIO_Pin_DIR2	= va_arg(pinconfig, int);



		GPIOPinsInit(stpx->Step_BDC->GPIOx_DIR1, stpx->Step_BDC->GPIO_Pin_DIR1,GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
		GPIOPinsInit(stpx->Step_BDC->GPIOx_DIR2, stpx->Step_BDC->GPIO_Pin_DIR2,GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);

	}else if(stpx->Step_BDC->Dirpintype == SHIFTREG){

		stpx->Step_BDC->shiftreg = va_arg(pinconfig, byte_t *);
		stpx->Step_BDC->dir1 = va_arg(pinconfig, int);
		stpx->Step_BDC->dir2 = va_arg(pinconfig, int);
	}

	va_end(pinconfig);

	__HAL_TIM_SET_COMPARE(stpx->Step_BDC->htim, (stpx->Step_BDC->Channel), 0);
}


/*
 * Function Name		: StepperMovetoAngle
 * Function Description : This function is called to move to an absolute angle relative to the step count 0.
 * Function Remarks		: Depending on the total step count and pwm period it might not be accurate to use this function.
 *						  Especially if the total steps are less than 360. Instead it's better to use StepperMovetoStep.
 *
 * Function Arguments	: stpx   				Pointer to stepper handle
 * 						  Angle    				Angle to go to can be either positive or negative
 * Function Return		: None
 * Function Example		: StepperMovetoAngle(&stp1,90);
 */
void StepperMovetoAngle(STP_t *stpx,float Angle){

	StepperMoveSteps(stpx,(stpx->step_1_angle * Angle) - stpx->step_count);
}

/*
 * Function Name		: StepperMovetoStep
 * Function Description : This function is called to move to a certain step relative to step count 0.
 * Function Remarks		:
 *
 *
 * Function Arguments	: stpx   				Pointer to stepper handle
 * 						  Step    				Step to move to can be positive or negative
 * Function Return		: None
 * Function Example		: StepperMovetoStep(&stp1,200);
 */
void StepperMovetoStep(STP_t *stpx,int Step){
	StepperMoveSteps(stpx,Step - stpx->step_count);
}

/*
 * Function Name		: StepperMoveSteps
 * Function Description : This function is called to move a certain number of steps.
 * Function Remarks		:
 *
 *
 * Function Arguments	: stpx   				Pointer to stepper handle
 * 						  Steps    				steps to move can be either positive or negative
 * Function Return		: None
 * Function Example		: StepperMoveSteps(&stp1,50);
 */
void StepperMoveSteps(STP_t *stpx,int Steps){

	stpx->step_target = stpx->step_count + Steps;

	if(Steps>0){
		stpx->step_dir=0;
		WriteBDC(stpx->Step_BDC,stpx->period/2);
		stpx->isturning=1;
	}else{
		stpx->step_dir=1;
		WriteBDC(stpx->Step_BDC,-(stpx->period/2));
		stpx->isturning=1;
	}
}

/*
 * Function Name		: StepperResetCount
 * Function Description : This function is called to reset the step count to 0.
 * Function Remarks		:
 *
 *
 * Function Arguments	: stpx   				Pointer to stepper handle
 * Function Return		: None
 * Function Example		: StepperResetCount(&stp1);
 */
void StepperResetCount(STP_t *stpx){
	stpx->step_count = 0;
}

/*
 * Function Name		: StepperUpdate
 * Function Description : This function is called to check whether the number of steps is reached
 * Function Remarks		: Call in systick if not using Freertos and call in timer base if using freertos
 *
 *
 * Function Arguments	: stpx   				Pointer to stepper handle
 * Function Return		: None
 * Function Example		: StepperUpdate(&stp1);
 */
void StepperUpdate(STP_t *stpx){

	if(stpx->isturning){
		if(stpx->step_dir){

			stpx->step_count =  stpx->step_count - 1.0;

			if(stpx->step_count <= stpx->step_target){
				WriteBDC(stpx->Step_BDC, 0);
				stpx->isturning=0;
			}

		}else{

			stpx->step_count = stpx->step_count + 1.0;

			if(stpx->step_count >= stpx->step_target){
				WriteBDC(stpx->Step_BDC, 0);
				stpx->isturning=0;
			}

		}
	}

}


/*
 * Function Name		: StepperStop
 * Function Description : This function is called to stop the stepper
 * Function Remarks		:
 *
 *
 * Function Arguments	: stpx   				Pointer to stepper handle
 * Function Return		: None
 * F
 */
void StepperStop(STP_t *stpx){
	WriteBDC(stpx->Step_BDC,0);
	stpx->isturning=0;
}
