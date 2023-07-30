/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "servo_driver.h"


/************************************************/
/*		 	 	Functions		       		  	*/
/************************************************/

/* Function Name		: ServoDriverInit
 * Function Description : This function is called to Initialize the Servo Driver
 * Function Remarks		:
 * Function Arguments	:
 *
 *						  srv_drvx                         Pointer to Servo Driver struct
 * 						  hi2cx        				      Pointer to I2C handle
 * 						  _i2caddr				          Address of the Servo Driver
 * Function Return		: NONE
 * Function Example		:   ServoDriverInit(&srv_drv,&hi2c2,0x40);
 */
void ServoDriverInit(Srv_Drv_t *srv_drvx,I2C_HandleTypeDef *hi2cx,uint16_t _i2caddr){

	srv_drvx->_i2caddr = _i2caddr;
	srv_drvx->hi2cx = hi2cx;

	ServoDriverReset(srv_drvx);

	ServoDriverSetPWMFreq(srv_drvx,50);
}

/* Function Name		: ServoDriverSetPWMFreq
 * Function Description : This function is called to Set the PWM frequency for the entire chip, up to ~1.6 KHz
 * Function Remarks		:
 * Function Arguments	:
 *
 *						  srv_drvx                         Pointer to Servo Driver struct
 * 						  freq        				      Frequency between 1 and 3500
 * Function Return		: NONE
 * Function Example		:  ServoDriverSetPWMFreq(&srv_drv,1000);
 */
void ServoDriverSetPWMFreq(Srv_Drv_t *srv_drvx,float freq){

	if(freq<1)
		freq=1;
	else if(freq>3500)   // Datasheet limit is 3052=50MHz/(4*4096)
		freq=3500;

	float prescaleval = ((26000000 / (freq * 4096.0)) + 0.5) - 1;
	  if (prescaleval < PCA9685_PRESCALE_MIN)
	    prescaleval = PCA9685_PRESCALE_MIN;
	  if (prescaleval > PCA9685_PRESCALE_MAX)
	    prescaleval = PCA9685_PRESCALE_MAX;
	  uint8_t prescale = (uint8_t)prescaleval;

	  uint8_t oldmode = I2CReadReg8(srv_drvx->hi2cx,srv_drvx->_i2caddr,PCA9685_MODE1);

	  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;


	  I2CWriteReg8(srv_drvx->hi2cx,srv_drvx->_i2caddr,PCA9685_MODE1,newmode);

	  HAL_Delay(5);

	  I2CWriteReg8(srv_drvx->hi2cx,srv_drvx->_i2caddr,PCA9685_PRESCALE,prescale);

	  HAL_Delay(5);

	  I2CWriteReg8(srv_drvx->hi2cx,srv_drvx->_i2caddr,PCA9685_MODE1,oldmode);

	  HAL_Delay(5);

	  I2CWriteReg8(srv_drvx->hi2cx,srv_drvx->_i2caddr,PCA9685_MODE1,(oldmode | MODE1_RESTART | MODE1_AI));
}

/* Function Name		: ServoDriverReset
 * Function Description : This function is called to Send a reset command to the PCA9685 chip
 * Function Remarks		:
 * Function Arguments	:
 *
 *						  srv_drvx                         Pointer to Servo Driver struct
 * Function Return		: NONE
 * Function Example		:  ServoDriverReset(&srv_drv);
 */
void ServoDriverReset(Srv_Drv_t *srv_drvx){
	I2CWriteReg8(srv_drvx->hi2cx,srv_drvx->_i2caddr,PCA9685_MODE1,MODE1_RESTART);
//	I2CWriteReg8(srv_drvx->hi2cx,srv_drvx->_i2caddr,PCA9685_MODE2,MODE2_OUTDRV);
}

/* Function Name		: ServoDriverSleep
 * Function Description : This function is called to Put the Servo Driver board into sleep mode
 * Function Remarks		:
 * Function Arguments	:
 *
 *						  srv_drvx                         Pointer to Servo Driver struct
 * Function Return		: NONE
 * Function Example		:  ServoDriverSleep(&srv_drv);
 */
void ServoDriverSleep(Srv_Drv_t *srv_drvx) {

	 uint8_t awake = I2CReadReg8(srv_drvx->hi2cx,srv_drvx->_i2caddr,PCA9685_MODE1);

	 uint8_t sleep = awake | MODE1_SLEEP;

	 I2CWriteReg8(srv_drvx->hi2cx,srv_drvx->_i2caddr,PCA9685_MODE1,sleep);

}

/* Function Name		: ServoDriverWakeup
 * Function Description : This function is called to Wake the Servo Driver board from sleep
 * Function Remarks		:
 * Function Arguments	:
 *
 *						  srv_drvx                         Pointer to Servo Driver struct
 * Function Return		: NONE
 * Function Example		:  ServoDriverWakeup(&srv_drv);
 */
void ServoDriverWakeup(Srv_Drv_t *srv_drvx){

	uint8_t sleep = I2CReadReg8(srv_drvx->hi2cx,srv_drvx->_i2caddr,PCA9685_MODE1);

	uint8_t wakeup = sleep & ~MODE1_SLEEP;

	I2CWriteReg8(srv_drvx->hi2cx,srv_drvx->_i2caddr,PCA9685_MODE1,wakeup);

}

/* Function Name		: ServoDriverReadPrescale
 * Function Description : This function is called to get the prescaler value from the Servo Driver Board
 * Function Remarks		:
 * Function Arguments	:
 *
 *						  srv_drvx                         Pointer to Servo Driver struct
 * Function Return		: NONE
 * Function Example		:  ServoDriverReadPrescale(&srv_drv);
 */
uint8_t ServoDriverReadPrescale(Srv_Drv_t *srv_drvx) {
  return I2CReadReg8(srv_drvx->hi2cx,srv_drvx->_i2caddr,PCA9685_PRESCALE);
}

/* Function Name		: ServoDriverGetPWM
 * Function Description : This function is called to enquire about the PWM of a certain Servo
 * Function Remarks		:
 * Function Arguments	:
 *
 *						  srv_drvx                         Pointer to Servo Driver struct
 *						  num                             number of the Servo from 0 to 15
 * Function Return		: NONE
 * Function Example		:  ServoDriverGetPWM(&srv_drv,0);
 */
int ServoDriverGetPWM(Srv_Drv_t *srv_drvx,uint8_t num){
	int Data;
	HAL_I2C_Mem_Read(srv_drvx->hi2cx,srv_drvx->_i2caddr<<1, (PCA9685_LED0_ON_L + 4 * num), I2C_MEMADD_SIZE_8BIT, (uint8_t*)&Data, 4, 100);
	return (uint16_t)(Data>>16);
}

/* Function Name		: ServoDriverSetOnOff
 * Function Description : This function is called to Set about the PWM of a certain Servo
 * Function Remarks		:
 * Function Arguments	:
 *
 *						  srv_drvx                         Pointer to Servo Driver struct
 *						  num                             number of the Servo from 0 to 15
 *						  on 							  Time to set the wave to high from (0 to 4095)
 *						  off                             Time to set the wave to low (0 to 4095)
 * Function Return		: NONE
 * Function Example		:  ServoDriverSetOnOff(&srv_drv,0,0,1024);  // 25% duty cycle
 */
void ServoDriverSetOnOff(Srv_Drv_t *srv_drvx,uint8_t num, uint16_t on, uint16_t off){

	uint8_t data[5];
	uint8_t ch = 0xff;

	data[0] = 	PCA9685_LED0_ON_L + 4 * num;
	data[1] = on;
	data[2] = on >> 8;
	data[3] = off;
	data[4] = off >> 8;

	HAL_I2C_Master_Transmit(srv_drvx->hi2cx,srv_drvx->_i2caddr<<1,data,5,100);
//	while(HAL_I2C_Master_Transmit(srv_drvx->hi2cx,srv_drvx->_i2caddr<<1,data,5,100)!=HAL_OK);
//	int toomuch = 0;
//	while (ch != HAL_OK) {
//
//		ch = HAL_I2C_Master_Transmit(srv_drvx->hi2cx,srv_drvx->_i2caddr<<1,data,5,100);
//		if (ch == HAL_ERROR || ch == HAL_TIMEOUT) {
//			HAL_I2C_DeInit(srv_drvx->hi2cx);
//			I2CxInit (&hi2c3,main_board_1, CLOCK_SPEED_100KHz,DISABLE);
//			ServoDriverInit(srv_drvx, &hi2c3, 0x40);
//			toomuch++;
//		}
//
//		if(toomuch > 10){
//			break;
//		}
//	}
}

/* Function Name		: ServoDriverSetPWM
 * Function Description : This function is called to Set about the PWM of a certain Servo without dealing wit on and off parameters
 * Function Remarks		:
 * Function Arguments	:
 *
 *						  srv_drvx                         Pointer to Servo Driver struct
 *						  num                             number of the Servo from 0 to 15
 *						  val 							  Time to set active from (0 to 4095)
 *						  invert                          0 or 1 inverts the duty cycle
 * Function Return		: NONE
 * Function Example		: ServoDriverSetPWM(&srv_drv,0,1024,0);  // 25% duty cycle
 */
void ServoDriverSetPWM(Srv_Drv_t *srv_drvx,uint8_t num, uint16_t val, int invert) {

  // Clamp value between 0 and 4095 inclusive.
  if(val>4095)
	  val=4095;
  else if(val<0)
	  val=0;

  if (invert) {
    if (val == 0) {
      // Special value for signal fully on.
    	ServoDriverSetOnOff(srv_drvx,num, 4096, 0);
    } else if (val == 4095) {
      // Special value for signal fully off.
    	ServoDriverSetOnOff(srv_drvx,num, 0, 4096);
    } else {
    	ServoDriverSetOnOff(srv_drvx,num, 0, 4095 - val);
    }
  } else {
    if (val == 4095) {
      // Special value for signal fully on.
    	ServoDriverSetOnOff(srv_drvx,num, 4096, 0);
    } else if (val == 0) {
      // Special value for signal fully off.
    	ServoDriverSetOnOff(srv_drvx,num, 0, 4096);
    } else {
    	ServoDriverSetOnOff(srv_drvx,num, 0, val);
    }
  }
}

/* Function Name		: ServoDriverWriteMicroseconds
 * Function Description : This function is called to Set the PWM output of one of the PCA9685 pins based on the input microseconds, output is not precise
 * Function Remarks		:
 * Function Arguments	:
 *
 *						  srv_drvx                         Pointer to Servo Driver struct
 *						  num                             number of the Servo from 0 to 15
 *						  Microsecounds 				  The number of Microseconds to turn the PWM output ON
 * Function Return		: NONE
 * Function Example		: ServoDriverWriteMicroseconds(&srv_drv,0,300);
 */
void ServoDriverWriteMicroseconds(Srv_Drv_t *srv_drvx, uint8_t num,uint16_t Microseconds) {

  double pulse = Microseconds;
  double pulselength;
  pulselength = 1000000; // 1,000,000 us per second

  // Read prescale
  uint16_t prescale = ServoDriverReadPrescale(srv_drvx);


  // Calculate the pulse for PWM based on Equation 1 from the datasheet section
  // 7.3.5
  prescale += 1;
  pulselength *= prescale;
  pulselength /= 26000000;


  pulse /= pulselength;


  ServoDriverSetOnOff(srv_drvx,num, 0, pulse);
}

/* Function Name		: ServoDriverSetExtClk
 * Function Description : This function is called to Set EXTCLK pin to use the external clock
 * Function Remarks		:
 * Function Arguments	:
 *
 *						  srv_drv                         Pointer to Servo Driver struct
 *						  prescale                        Configures the prescale value to be used by the external clock
 * Function Return		: NONE
 * Function Example		: ServoDriverSetExtClk(&srv_drv,8);
 */
void ServoDriverSetExtClk(Srv_Drv_t *srv_drvx,uint8_t prescale) {
  uint8_t oldmode = I2CReadReg8(srv_drvx->hi2cx,srv_drvx->_i2caddr,PCA9685_MODE1);
  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
  I2CWriteReg8(srv_drvx->hi2cx,srv_drvx->_i2caddr,PCA9685_MODE1,newmode); // go to sleep, turn off internal oscillator

  // This sets both the SLEEP and EXTCLK bits of the MODE1 register to switch to
  // use the external clock.
  I2CWriteReg8(srv_drvx->hi2cx,srv_drvx->_i2caddr,PCA9685_MODE1,(newmode |= MODE1_EXTCLK));

  I2CWriteReg8(srv_drvx->hi2cx,srv_drvx->_i2caddr,PCA9685_PRESCALE,prescale); // set the prescaler

  HAL_Delay(5);

  // clear the SLEEP bit to start
  I2CWriteReg8(srv_drvx->hi2cx,srv_drvx->_i2caddr,PCA9685_MODE1,((newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI));

}

/* Function Name		: ServoDriverSetOutputMode
 * Function Description : This function is called to Sets the output mode of the PCA9685 to either open drain or push pull / totempole.
 * Function Remarks		:
 * Function Arguments	:
 *
 *						  srv_drv                         Pointer to Servo Driver struct
 *						  totempole                       0 or 1
 * Function Return		: NONE
 * Function Example		: ServoDriverSetOutputMode(&srv_drv,0);
 */
void ServoDriverSetOutputMode(Srv_Drv_t *srv_drvx,int totempole) {

  uint8_t oldmode = I2CReadReg8(srv_drvx->hi2cx,srv_drvx->_i2caddr,PCA9685_MODE2);
  uint8_t newmode;
  if (totempole) {
    newmode = oldmode | MODE2_OUTDRV;
  } else {
    newmode = oldmode & ~MODE2_OUTDRV;
  }
  I2CWriteReg8(srv_drvx->hi2cx,srv_drvx->_i2caddr,PCA9685_MODE2,newmode);

}
