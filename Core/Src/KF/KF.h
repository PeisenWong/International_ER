/* 
 * SimpleKalmanFilter - a Kalman Filter implementation for single variable models.
 * Created by Denys Sene, January, 1, 2017.
 * Released under MIT License - see LICENSE file for details.
 */

#ifndef SimpleKalmanFilter_h
#define SimpleKalmanFilter_h

#include "../BIOS/bios.h"
  
typedef struct{
  float _err_measure;
  float _err_estimate;
  float _q;
  float _current_estimate;
  float _last_estimate;
  float _kalman_gain;

  uint8_t  kalman_loop;
  float* kalman_input;
  float* kalman_output;
  float kalman_output_tmp;
}KALMANFILTER_t;



void KalmanFilterInit(uint8_t,  float*,float*,float, float, float, KALMANFILTER_t*);
float updateEstimate(float, KALMANFILTER_t*);
void setMeasurementError(float, KALMANFILTER_t*);
void setEstimateError(float, KALMANFILTER_t*);
void setProcessNoise(float, KALMANFILTER_t*);
float getKalmanGain(KALMANFILTER_t*);
void KalmanFilter (KALMANFILTER_t *kalmanfilter);

#endif
