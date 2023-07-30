
/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "mov_ave.h"

/************************************************/
/*		 	 	Functions		       		  	*/
/************************************************/
/* Function Name		: Moving_Average_Init
 * Function Description : This function is called to initialise The Moving Average Filter.
 * Function Remarks		:
 * Function Arguments	:
 *
 *						  Mov_Ave                       Pointer to Mov_Ave_t handle
 * 						  Input						    Pointer to Input of the filter
 * 						  Output					    Pointer to Output of the filter
 * Function Return		: NONE
 * Function Example		: Moving_Average_Init (&move_ave, &Laser_dist, &Laser_ave);
 */
void Moving_Average_Init(Mov_Ave_t *Mov_Ave, float *Input, float *Output){

	Mov_Ave->input = Input;
	Mov_Ave->Sum = 0;
	Mov_Ave->WindowPointer = 0;
	Mov_Ave->output = Output;

	for(uint32_t i=0; i<WindowLength; i++)
	{
		Mov_Ave->History[i] = 0;
	}

}

/************************************************/
/*		 	 	Functions		       		  	*/
/************************************************/
/* Function Name		: Moving_Average_Filter
 * Function Description : This function is called to Use The Moving Average Filter.
 * Function Remarks		:
 * Function Arguments	:
 *
 *						  Mov_Ave                       Pointer to Mov_Ave_t handle
 * Function Return		: NONE
 * Function Example		: Moving_Average_Filter (&move_ave);
 */
void Moving_Average_Filter(Mov_Ave_t *Mov_Ave){


	Mov_Ave->Sum += *(Mov_Ave->input);
	Mov_Ave->Sum -= Mov_Ave->History[Mov_Ave->WindowPointer];
	Mov_Ave->History[Mov_Ave->WindowPointer] = *(Mov_Ave->input);
	if(Mov_Ave->WindowPointer < WindowLength - 1)
	{
		Mov_Ave->WindowPointer += 1;
	}
	else
	{
		Mov_Ave->WindowPointer = 0;
	}
	*(Mov_Ave->output) = (Mov_Ave->Sum/WindowLength);

}
