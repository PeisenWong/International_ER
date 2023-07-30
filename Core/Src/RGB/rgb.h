/************************************************
 * Title   : RGB
 * Author  : LokCharming
 * Version : 1.0
 * Date    : 8/3/2020
 * **********************************************
 * Descriptions: Control rgb with software pwm
 *
 *
 * Version History:
 * 1.0 - implemented using hal library
 *
 * Bugs:
 *
 ************************************************/

#ifndef RGB_RGB_H_
#define RGB_RGB_H_

/***************************************
 * 		Include Libraries 			   *
 **************************************/

#include "../BIOS/bios.h"

/**************************************************
 * 		Enumerator							  	  *
 *************************************************/
typedef enum{
	RGB_RED,
	RGB_BLUE,
	RGB_GREEN
}RBG_COLOR;

/**************************************************
 * 		Structure							  	  *
 *************************************************/

typedef struct{
	GPIO_TypeDef*	GPIO[3];
	uint16_t		GPIOPin[3];
	TIM_HandleTypeDef* htim;

	uint32_t		fadeTick;
	uint32_t		pwmtick;
	uint16_t		Hpulse;
	uint16_t		Lpulse;
	uint16_t		Period;
	uint16_t 		fadeSpeed;
}RGB_t;

RGB_t rgb;
/**************************************************
 * 		Function Prototype			  			  *
 *************************************************/

void RGBInit(RGB_t* rgb, GPIO_TypeDef*, uint16_t, GPIO_TypeDef*, uint16_t, GPIO_TypeDef*, uint16_t, TIM_HandleTypeDef*, uint16_t);
void RGB_Breath(RGB_t* rgb);
#endif /* RGB_RGB_H_ */
