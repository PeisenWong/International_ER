
/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "PID.h"
/*********************************************/




/*********************************************/
/*          Variable                         */
/*********************************************/

/*********************************************/




/*********************************************/
/*           Function Prototype              */
/*********************************************/

/*********************************************/




/*********************************************/
/*           Subroutine Function             */
/*********************************************/
void PIDSourceInit (float *in, float *out, PID_t *pid)
{
	pid->error = in;
	pid->out_put = out;
}

void PIDGainInit(float ts, float sat, float ke, float ku, float kp, float ki,
									float kd, float kn, PID_t *pid)
{
	pid->K[TS] = ts;
	pid->K[SAT] = sat;
	pid->K[KE] = ke;
	pid->K[KU] = ku;
	pid->K[KP] = kp;
	pid->K[KI] = ki;
	pid->K[KD] = kd;
	pid->K[KN] = kn;
	PIDCoeffCalc(pid);

//	pid->i_flag = pid->pd_flag = pid->s_flag = 0;

//	if (pid->K[KI] != 0){
//		pid->i_flag = 1;
//		pid->K[B_0] = (pid->K[TS] * pid->K[KI]) * 0.5;
//		pid->K[B_1] = pid->K[B_0];
//	} else {
//		pid->i_flag = 0;
//		pid->K[B_0] = pid->K[B_1] = 0;
//	}
//
//	if ((pid->K[KD] != 0) || (pid->K[KP] != 0)){
//	pid->pd_flag = 1;
//	pid->K[C_1] = -((pid->K[TS] * pid->K[KN] - 2.0) / (pid->K[TS] * pid->K[KN] + 2.0));
//	pid->K[D_0] = (pid->K[KP] + ((2.0 * pid->K[KD] * pid->K[KN]) / (pid->K[KN] * pid->K[TS] + 2.0)));
//	pid->K[D_1] = (((pid->K[KN] * pid->K[TS] - 2.0) * pid->K[KP]) - (2.0 * pid->K[KD] * pid->K[KN])) / (pid->K[KN] * pid->K[TS] + 2);
//	} else {
//		pid->pd_flag = 0;
//		pid->K[C_1] = pid->K[D_0] = pid->K[D_1] = 0;
//	}
}

void PIDGainSet (unsigned char a, float value, PID_t *pid)
{
	pid->K[a] = value;
	PIDCoeffCalc(pid);
}

void PIDCoeffCalc (PID_t *pid)
{
	if (pid->K[KI] != 0){
		pid->K[B_0] = (pid->K[TS] * pid->K[KI]) * 0.5;
		pid->K[B_1] = pid->K[B_0];
	} else {
		pid->K[B_0] = pid->K[B_1] = 0;
	}

	if ((pid->K[KP] != 0) || (pid->K[KD] != 0)){
	pid->K[C_1] = -((pid->K[TS] * pid->K[KN] - 2.0) / (pid->K[TS] * pid->K[KN] + 2.0));
	pid->K[D_0] = (pid->K[KP] + ((2.0 * pid->K[KD] * pid->K[KN]) / (pid->K[KN] * pid->K[TS] + 2.0)));
	pid->K[D_1] = ((((pid->K[KN] * pid->K[TS] - 2.0) * pid->K[KP]) - (2.0 * pid->K[KD] * pid->K[KN])) /
					(pid->K[KN] * pid->K[TS] + 2.0));
	} else {
		pid->K[C_1] = pid->K[D_0] = pid->K[D_1] = 0.0;
	}
}

void PIDDelayInit (PID_t *pid)
{
//	unsigned char i;
//	for(i = 0; i < 2; i++){
//		pid->i_delay[i] = 0;
//		pid->d_delay[i] = 0;
//	}
	pid->i_delay[0] = 0.0;
	pid->i_delay[1] = 0.0;
	pid->d_delay[0] = 0.0;
	pid->d_delay[1] = 0.0;
	pid->s_delay = 0.0;
}

char PIDIsSaturared (PID_t *pid)
{
	return !pid->s_flag;	/* 1 is not saturated, 0 is saturated */
}

void PID (PID_t *pid)
{
	/* Error ******************************************************************/
	float input_buffer, error;

	input_buffer = *pid->error;
	error = input_buffer * pid->K[KE];

	/* Reset previous sum *****************************************************/
	pid->s_delay = 0;

	/* I algorithm ************************************************************/

	if (pid->K[KI] != 0){
		if (pid->s_flag){     //flag only, so 0 or 1...
			pid->i_delay[1] = error + pid->i_delay[0];      //it is not saturated, so it will continue add error
		} else {
			pid->i_delay[1] = pid->i_delay[0];				//it is saturated already, so all pid->i_delay will be constant
		}
		pid->s_delay = pid->s_delay + (pid->K[B_0] * pid->i_delay[1]) //after saturated, s_delay will be continously added
							+ (pid->K[B_1] * pid->i_delay[0]);
		pid->i_delay[0] = pid->i_delay[1];
	}

// since final output is addition of 2 outputs, so pid->s_delay will combine with the equation in document..

	/* PD algorithm ***********************************************************/

	if ((pid->K[KP] != 0) || (pid->K[KD] != 0)){       				// i has change behind KP to KD..
		pid->d_delay[1] = error + pid->K[C_1] * pid->d_delay[0];  		//pid->d_delay[0] is old value,pid->d_delay[1]is new value,so both are different values
		pid->s_delay = pid->s_delay + pid->K[D_0] * pid->d_delay[1]
							+ pid->K[D_1] * pid->d_delay[0];
		pid->d_delay[0] = pid->d_delay[1];
	}

	/* Saturation element *****************************************************/
	float output_buffer, output;

	output_buffer = pid->s_delay;
	if (output_buffer > pid->K[SAT]){
		output = (pid->K[SAT] * pid->K[KU]);
		pid->s_flag = 0;							/* Saturated */
	}else if (output_buffer < -(pid->K[SAT])){
		output = -(pid->K[SAT] * pid->K[KU]);
		pid->s_flag = 0;							/* Saturated */
	}else {
		output = (output_buffer * pid->K[KU]);
		pid->s_flag = 1;							/* Not saturated */
	}

	/* Output *****************************************************************/
	*pid->out_put = output;
}

/*Remarks :
-- pid->s_delay is the output coeeficient, i think will be decimal that means smaller than modulus of 1
-- positive value maybe move one direction while negative value move another direction
-- pid->K[SAT] always set as 1, so if more than one no matter which direction, 1 will be coeeficient
-- pid->K[KU] is output gain that means we need put the maximum output limit
-- if it is saturated, the output will be that max limit output
-- if it is not saturated, output coeficient will mutiply with max limit that means will get a value that is smaller value than max limit..
*/

