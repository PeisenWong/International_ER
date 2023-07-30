/* 
 * SimpleKalmanFilter - a Kalman Filter implementation for single variable models.
 * Created by Denys Sene, January, 1, 2017.
 * Released under MIT License - see LICENSE file for details.
 */
//#include <KF/KF.h>
#include <math.h>
#include "KF.h"
/*
 *
	KALMANFILTER_t kf_adc1,kf_adc3; //adapter.c
 *
 *
	KalmanFilterInit(1, adc1.ADC_1, &(adc1.ADC_1),5.0,2.0,90.0,&kf_adc1 );
	KalmanFilterInit(1, adc1.ADC_3, &(laser_y),5.0,2.0,90.0,&kf_adc3 );
 *
 * ***************************** interrupt ******************************
 *
		KalmanFilter(&kf_adc1);
		KalmanFilter(&kf_adc3);
 *
 *
 * */

void KalmanFilterInit(uint8_t  kalman_loop,  float* kalman_input,float* kalman_output,
						float mea_e, float est_e, float q, KALMANFILTER_t *kalmanfilter){

	kalmanfilter->kalman_input = kalman_input;
	kalmanfilter->kalman_output= kalman_output;
	kalmanfilter->kalman_loop  = kalman_loop;

	kalmanfilter->_err_measure = mea_e;
	kalmanfilter->_err_estimate = est_e;
	kalmanfilter->_q = q;
}

float updateEstimate(float mea, KALMANFILTER_t *kalmanfilter)
{
	kalmanfilter->_kalman_gain = kalmanfilter->_err_estimate / (kalmanfilter->_err_estimate + kalmanfilter->_err_measure);
	kalmanfilter->_current_estimate = kalmanfilter->_last_estimate + kalmanfilter->_kalman_gain * (mea - kalmanfilter->_last_estimate);
	kalmanfilter->_err_estimate = (1.0 - kalmanfilter->_kalman_gain) * kalmanfilter->_err_estimate + fabs(kalmanfilter->_last_estimate - kalmanfilter->_current_estimate) * kalmanfilter->_q;
	kalmanfilter->_last_estimate = kalmanfilter->_current_estimate;

	return kalmanfilter->_current_estimate;
}

void setMeasurementError(float mea_e, KALMANFILTER_t *kalmanfilter)
{
	kalmanfilter->_err_measure = mea_e;
}

void setEstimateError(float est_e, KALMANFILTER_t *kalmanfilter)
{
	kalmanfilter->_err_estimate = est_e;
}

void setProcessNoise(float q, KALMANFILTER_t *kalmanfilter)
{
	kalmanfilter->_q = q;
}

float getKalmanGain(KALMANFILTER_t *kalmanfilter) {
	return kalmanfilter->_kalman_gain;
}

void KalmanFilter (KALMANFILTER_t *kalmanfilter){
		kalmanfilter->kalman_output_tmp  = ((float)*(kalmanfilter->kalman_input))/10000.0;
		kalmanfilter->kalman_output_tmp  = updateEstimate(kalmanfilter->kalman_output_tmp, kalmanfilter);
		kalmanfilter->kalman_output_tmp  = (kalmanfilter->kalman_output_tmp)*10000.0;

		*(kalmanfilter->kalman_output )  = kalmanfilter->kalman_output_tmp ;
}

