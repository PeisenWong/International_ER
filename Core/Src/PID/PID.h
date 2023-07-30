
/*******************************************************************************
 * Title   : Proportional, Integral, and Derivative Controller
 * Author  : Ng Khin Hooi
 * Version : 2.10
 * Date    : December 2011
 *******************************************************************************
 * Description:
 * - Combined proportional and derivative gain algorithm.
 * - All data types are in float.
 * -
 * Version History:
 * 2.01 by Ng Khin Hooi
 * - Added derivative filter gain.
 * 2.00
 * - Practical version of PID
 * - Increased of flexibility using pointers in structures
 * - Increased accuracy in calculation by using floating point
 *
 * 1.00 by Lim Kian Sheng
 * - Basic function of ideal PID
 *
 * Bugs:
 *
 ******************************************************************************/

#ifndef PID_H
#define PID_H


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
/*          Enumarator                      _ */
/*********************************************/
enum {TS = 0, SAT, KE, KU, KP, KI, KD, KN, B_0, B_1, C_1, D_0, D_1};
/*********************************************/




/*********************************************/
/*          Variable                         */
/*********************************************/
typedef struct{
	struct{
		unsigned s_flag	: 1;	/* Saturation 		*/
//		unsigned i_flag : 1;	/* Source Init Flag */
//		unsigned pd_flag: 1;
	};
	float *error;
	float *out_put;
	float K[13];
	float i_delay[2];
	float d_delay[2];
	float s_delay;
}PID_t;
/*********************************************/




/*********************************************/
/*           Function Prototype              */
/*********************************************/
void PIDSourceInit (float *in, float *out, PID_t *pid);
void PIDGainInit (float ts, float sat, float ke, float ku, float kp, float ki,
						float kd, float kn, PID_t *pid);
void PIDGainSet (unsigned char a, float value, PID_t *pid);
void PIDCoeffCalc (PID_t *pid);
void PIDDelayInit (PID_t *pid);
char PIDsSaturared (PID_t *pid);
void PID (PID_t *pid);
/*********************************************/

#endif
