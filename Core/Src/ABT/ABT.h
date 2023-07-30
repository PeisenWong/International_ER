
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

#ifndef ABT_H
#define ABT_H


/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "../BIOS/bios.h"

/*********************************************/




/*********************************************/
/*          Define                           */
/*********************************************/

/*********************************************/




/*********************************************/
/*          Enumarator                       */
/*********************************************/

/*********************************************/




/*********************************************/
/*          Variable                         */
/*********************************************/
typedef struct{
	float sample_time;
	float alpha;
	float beta;
	float gamma;
	float *input;
	float *pos_output;
	float *vel_output;
	float *acc_output;
} ABT_t;
/*********************************************/




/*********************************************/
/*           Function Prototype              */
/*********************************************/
void ABTInit(float samp_time, float a, float b, float g, float *ip, float *pos,
										float *vel, float *acc, ABT_t *filt);
void ABTEstimateInit(ABT_t *filt);
void ABT(ABT_t *filt);
/*********************************************/

#endif
