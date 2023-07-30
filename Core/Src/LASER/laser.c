/*
 * laser.c
 *
 *  Created on: Aug 18, 2022
 *      Author: Shaon
 */

#include "laser.h"

/*
 * Function Name		: LaserInit
 * Function Description : init Laser's parameters.
 * Function Remarks		: NONE
 * Function Arguments	: *laser		, pointer to structure Laser_t
 * 						  max_dist		, maximum distance for laser
 * 						  min_dist		, minimum distance for laser
 * 						  kalman_noise	, kalman noise parameter
 * 						  min_error		, error at minimum distance
 * Function Return		: None
 * Function Example		: LaserInit(&lsr1, 3.0, 0.8, 100.0, 0.092);
 */
void LaserInit(Laser_t *laser, float max_dist, float min_dist, float kalman_noise, float min_error){
	laser->min_dist=min_dist;
	laser->min_err=min_error;
	laser->ratio=(max_dist-min_dist)/4095.0;
	KalmanFilterInit(0, &(laser->rawDist), &(laser->dist), 0.05, 6.0, kalman_noise, &laser->kf);
}

/*
 * Function Name		: Laser
 * Function Description : updates laser value.
 * Function Remarks		: NONE
 * Function Arguments	: *laser		, pointer to structure Laser_t
 * Function Return		: None
 * Function Example		: Laser(&lsr1);
 */
void Laser(Laser_t *laser){
	if(laser->rawCu!=0 && laser->cnt!=0){
		laser->raw=((float)laser->rawCu/(float)laser->cnt);
		laser->rawCu=0;
		laser->cnt=0;
		laser->rawDist=((laser->raw)*laser->ratio+laser->min_dist-laser->min_err)*100.0;
		KalmanFilter(&laser->kf);
		laser->dist/=100.0;
	}
}

//*************************************************************************************************************************************\\

/*
 * Function Name		: LaserUpdate
 * Function Description : updates Laser value in ADC_DMA.
 * Function Remarks		: NONE
 * Function Arguments	: *laser		, pointer to structure Laser_t
 * 						  channel_no	, ADC channel
 * Function Return		: None
 * Function Example		: LaserUpdate(&lsr1, 0);
 */
void LaserUpdate(Laser_t *laser, uint8_t channel_no){
	laser->rawCu+=adc.ADC_value[channel_no];
	laser->cnt++;
}



void DMA2_Stream2_IRQHandler(void)
{
	LaserUpdate(&lsrx, 0);
	LaserUpdate(&lsry, 1);

	HAL_DMA_IRQHandler(&hdma_adc1);

}

