/*
 * Tune.h
 *
 *  Created on: Mar 18, 2022
 *      Author: lokcharming
 */

#ifndef SRC_TUNE_H_
#define SRC_TUNE_H_

//my laptop screen(15 inch) can have 4 trackbars per workspace(screen), 18 variables per trackbar
#define MAX_VARIABLE_PER_WORKSPACE	72		/*Including Int and Float*/
//#define MAX_VARIABLE_PER_WORKSPACE	26

#define NUM_INT_TUNE_LIST0			0
#define NUM_FLOAT_TUNE_LIST0		22

#define NUM_INT_TUNE_LIST1			0
#define NUM_FLOAT_TUNE_LIST1		5

#define NUM_INT_TUNE_LIST2			1
#define NUM_FLOAT_TUNE_LIST2		1

#define NUM_INT_TUNE_LIST3			1
#define NUM_FLOAT_TUNE_LIST3		1

#define NUM_INT_TUNE_LIST4			1
#define NUM_FLOAT_TUNE_LIST4		1

float AP, AI, AD, BP, BI, BD, CP, CI, CD, DP, DI, DD;

float point1[7];
float point2[7];
float point3[7];
float point4[7];
float point5[7];
float point6[7];

int test[13];
float tol_xy, f_tol_xy, tol_z, f_tol_z;
float kp[2], ki[2], kd[2];
#endif /* SRC_TUNE_H_ */
