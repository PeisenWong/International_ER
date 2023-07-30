/*
 * TuningInterface.h
 *
 *  Created on: Mar 18, 2022
 *      Author: lokcharming
 */

#ifndef SRC_TUNINGINTERFACE_TUNINGINTERFACE_H_
#define SRC_TUNINGINTERFACE_TUNINGINTERFACE_H_

#include "../BIOS/system.h"
#include "../Eeprom/eeprom.h"
#include "../RNS_interface/RNS_interface.h"
#include "../PSx_Interface/PSx_Interface.h"
#include "../Tune.h"
#include "../MODN/MODN.h"
#include "../adapter.h"
#include "../common.h"


#define varI(__NAME__, __MIN__, __MAX__) {var(__NAME__), (int *)&(__NAME__), __MIN__, __MAX__}
#define varF(__NAME__, __MIN__, __MAX__) {var(__NAME__), (float *)&(__NAME__), __MIN__, __MAX__}

#define TOTAL_PAGE 5

//my laptop screen(15 inch) can have 4 trackbars per workspace(screen), 18 variables per trackbar
#define MAX_VARIABLE_PER_WORKSPACE	72		/*Including Int and Float, can be 72 */

//FLASH SECTOR 6 is used to store tune Int, One Sector(128Kbytes) can save 32000 int or float
#define SECTOR_6_FOR_PAGE_0		(SECTOR6)
#define SECTOR_6_FOR_PAGE_1		(SECTOR_6_FOR_PAGE_0+MAX_VARIABLE_PER_WORKSPACE*4)
#define SECTOR_6_FOR_PAGE_2		(SECTOR_6_FOR_PAGE_1+MAX_VARIABLE_PER_WORKSPACE*4)
#define SECTOR_6_FOR_PAGE_3		(SECTOR_6_FOR_PAGE_2+MAX_VARIABLE_PER_WORKSPACE*4)
#define SECTOR_6_FOR_PAGE_4		(SECTOR_6_FOR_PAGE_3+MAX_VARIABLE_PER_WORKSPACE*4)

//FLASH SECTOR 7 is used to store tune Float
#define SECTOR_7_FOR_PAGE_0		(SECTOR7)
#define SECTOR_7_FOR_PAGE_1		(SECTOR_7_FOR_PAGE_0+MAX_VARIABLE_PER_WORKSPACE*4)
#define SECTOR_7_FOR_PAGE_2		(SECTOR_7_FOR_PAGE_1+MAX_VARIABLE_PER_WORKSPACE*4)
#define SECTOR_7_FOR_PAGE_3		(SECTOR_7_FOR_PAGE_2+MAX_VARIABLE_PER_WORKSPACE*4)
#define SECTOR_7_FOR_PAGE_4		(SECTOR_7_FOR_PAGE_3+MAX_VARIABLE_PER_WORKSPACE*4)

UART_HandleTypeDef* tuneUart;
char tuneCommand;
uint8_t tuneHeader, tuneHeader1, tuneBuffer[7], tunePage, tuneWriteSuccess;

typedef struct{
	char* varName;
	int* ptr;
	int min;
	int max;
}Tune_Int_t;

typedef struct{
	char* varName;
	float* ptr;
	float min;
	float max;
}Tune_Float_t;

typedef enum{
	PAGE0,
	PAGE1,
	PAGE2,
	PAGE3,
	PAGE4
}TUNE_PAGE;

union{
	uint8_t flags;
	struct{
		uint8_t editTerminated	:1;
		uint8_t flag1	:1;
		uint8_t flag2	:1;
		uint8_t flag3	:1;
		uint8_t flag4	:1;
		uint8_t flag5	:1;
		uint8_t flag6	:1;
		uint8_t flag7	:1;

	};
}tune_flag;

extern Tune_Int_t TuneIntList0[NUM_INT_TUNE_LIST0];
extern Tune_Float_t TuneFloatList0[NUM_FLOAT_TUNE_LIST0];
extern Tune_Int_t TuneIntList1[NUM_INT_TUNE_LIST1];
extern Tune_Float_t TuneFloatList1[NUM_FLOAT_TUNE_LIST1];
extern Tune_Int_t TuneIntList2[NUM_INT_TUNE_LIST2];
extern Tune_Float_t TuneFloatList2[NUM_FLOAT_TUNE_LIST2];
extern Tune_Int_t TuneIntList3[NUM_INT_TUNE_LIST3];
extern Tune_Float_t TuneFloatList3[NUM_FLOAT_TUNE_LIST3];
extern Tune_Int_t TuneIntList4[NUM_INT_TUNE_LIST4];
extern Tune_Float_t TuneFloatList4[NUM_FLOAT_TUNE_LIST4];

void tuneRegister(int numPage, UART_HandleTypeDef* huartx);
void tuneEdit(TUNE_PAGE);
void tunePending(void);
void tuneUpdate(TUNE_PAGE);
void tuneInit(TUNE_PAGE);
uint32_t tuneWriteInt(void);
uint32_t tuneWriteFloat(void);
void tuneRead(void);
void tunePID(UART_HandleTypeDef* huartx, PSxBT_t* ps4, RNS_interface_t* rns);
void tunePIDTri(UART_HandleTypeDef* huartx, PSxBT_t* ps4, RNS_interface_t* rns);

#endif /* SRC_TUNINGINTERFACE_TUNINGINTERFACE_H_ */
