#ifndef DONGBEI_H_
#define DONGBEI_H_

/*********************************************/
/*              Include Header               */
/*********************************************/

/*********************************************/
/*                  Define			         */
/*********************************************/
#define uint8_t unsigned char /* not needed in atollic */

/*********************************************/
/*                   Types                   */
/*********************************************/

typedef union {
    char raw[24];
    struct {
        float ang_z; /* yaw */
        float ang_x; /* pitch, encoder send all 0 */
        float ang_y; /* roll,  encoder send all 0 */
        float pos_x; /* displacement at x-axis of encoder */
        float pos_y; /* displacement at y-axis of encoder */
        float vel_a; /* angular velocity of yaw */
    }data;
}dongbei_TypeDef;

typedef void (*dongbei_data_handler)(void);

/*********************************************/
/*              Enumeration                  */
/*********************************************/

typedef enum {
    DONGBEI_CALIBRATE,
    DONGBEI_RESET,
    DONGBEI_SET_YAW,
    DONGBEI_SET_X,
    DONGBEI_SET_Y,
    DONGBEI_SET_DOUBLE,
    DONGBEI_SET_ALL
}dongbei_cmd_t;

/*********************************************/
/*             Extern Variable               */
/*********************************************/

dongbei_TypeDef      DONGBEI_DATAFRAME;
dongbei_data_handler DONGBEIHandleData;

/*********************************************/
/*           Function Prototype              */
/*********************************************/
void     DONGBEIInit        (dongbei_data_handler dh);
void     DONGBEIProcessData (uint8_t input);
uint8_t* DONGBEICmd         (dongbei_cmd_t cmd, float f1, float f2, float f3);

#endif /* DONGBEI_H_ */
