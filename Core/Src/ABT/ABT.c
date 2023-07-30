
/*******************************************************************************
 * Title   : Alpha Beta Tracker
 * Creator : KHNg
 * Version : 1.00
 * Date    : August 2010
 *******************************************************************************
 * Description:
 * - Filter the input and given the output in filtered form, first derivative
 * 		and second derivative.
 *
 * Version History:
 *
 * Bugs:
 *
 ******************************************************************************/


/*********************************************/
/*          Include Header                   */
/*********************************************/

#include "ABT.h"
/*********************************************/




/*********************************************/
/*          Variable                         */
/*********************************************/

/*********************************************/




/*********************************************/
/*           Subroutine Function             */
/*********************************************/
void ABTInit(float samp_time, float a, float b, float g, float *ip, float *pos, float *vel, float *acc, ABT_t *filt)
{
	filt->sample_time = samp_time;
	filt->alpha = a;
	filt->beta = b;
	filt->gamma = g;
	filt->input = ip;
	filt->pos_output = pos;
	filt->vel_output = vel;
	filt->acc_output = acc;
}

void ABTEstimateInit(ABT_t *filt)
{
	*(filt->pos_output) = 0;
	*(filt->vel_output) = 0;
	*(filt->acc_output) = 0;
}

void ABT(ABT_t *filt)
{
	float pos_residual;
	float pos_predict;
	float vel_predict;

	/* Predict position and velocity using previous sample values */
	pos_predict = *(filt->pos_output) + filt->sample_time *
								(*(filt->vel_output)) + filt->sample_time *
								filt->sample_time * 0.5 * (*(filt->acc_output));
	vel_predict = *(filt->vel_output) + filt->sample_time * (*(filt->acc_output));

	/* Calculate residue */
	pos_residual = *filt->input - pos_predict;

	/* Update estimate with weighted input (a - b - g) */
	*(filt->pos_output) = pos_predict + filt->alpha * pos_residual;
	*(filt->vel_output) = vel_predict + (filt->beta / filt->sample_time)
																* pos_residual;
	*(filt->acc_output) = *(filt->acc_output) + (filt->gamma * 0.5 /
						(filt->sample_time * filt->sample_time)) * pos_residual;
}
/*********************************************/
