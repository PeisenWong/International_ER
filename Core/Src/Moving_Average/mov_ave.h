/************************************************
 * Title   : Moving Average Filter
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 27/9/2021
 * **********************************************
 * Descriptions: Use moving Average filter to stabilize data
 *
 *
 * Version History:
 * 1.0 - Implemented
 *
 *
 * Bugs:
 *
 ************************************************/

#ifndef SRC_MOVING_AVERAGE_MOV_AVE_H_
#define SRC_MOVING_AVERAGE_MOV_AVE_H_

#include "../BIOS/bios.h"

#define WindowLength 30

/* TypeDefs ------------------------------------------------------------------*/
typedef struct{
	float History[WindowLength]; /*Array to store values of filter window*/
	float Sum;	/* Sum of filter window's elements*/
	uint32_t WindowPointer; /* Pointer to the first element of window*/
	float *input;
	float *output;
}Mov_Ave_t;

/* Function prototypes -------------------------------------------------------*/


void Moving_Average_Init(Mov_Ave_t *Mov_Ave, float *Input, float *Output);
void Moving_Average_Filter(Mov_Ave_t *Mov_Ave);

#endif /* SRC_MOVING_AVERAGE_MOV_AVE_H_ */
