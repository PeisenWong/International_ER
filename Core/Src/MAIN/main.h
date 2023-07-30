
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "../common.h"

void MainTask(void *argument);
void NaviTask(void *argument);
void PitchTask(void *argument);
void EmergencyTask(void *argument);
void CheckingTask(void *argument);
void TuneTask(void *argument);
//void TestTask(void *argument);
//void LaserNavigateTask(void *argument);
//void SecondaryTask(void *argument);
//void FlywheelPitchPIDTask(void *argument);
//void FlywheelYawPIDTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
