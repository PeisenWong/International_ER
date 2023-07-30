/*
 * laser.h
 *
 *  Created on: Aug 18, 2022
 *      Author: Shaon
 */

#ifndef SRC_LASER_LASER_H_
#define SRC_LASER_LASER_H_

#include "../BIOS/bios.h"
//#include "../include.h"
#include "../ADC/adc.h"
#include "../KF/KF.h"

typedef struct{
	uint64_t rawCu;
	float raw;
	uint32_t cnt;
	float min_dist;
	float ratio;
	float rawDist;
	float dist;
	float min_err;
	uint8_t swap;
	KALMANFILTER_t kf;
}Laser_t;


Laser_t lsrx, lsry, lsrL, lsrR;

void LaserInit(Laser_t *laser, float max_dist, float min_dist, float kalman_noise, float min_error);
void Laser(Laser_t *laser);
void LaserUpdate(Laser_t *laser, uint8_t channel_no);



#endif /* SRC_LASER_LASER_H_ */
