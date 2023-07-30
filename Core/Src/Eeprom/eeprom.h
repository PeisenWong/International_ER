

/*******************************************************************************
 * Title   : Emulated Eeprom 
 * Author  : MLok
 * Version : 1.00
 * Date    : Dec 2021
 *******************************************************************************
 * Description:
 * - Use flash memory as emulated Eeprom
 * - 1Kbyte = 1024 bytes = 256 integers = 256 floats
 *
 * Version History:
 * 1.00 by Mlok
 * - Basic function of writing to flash
 *
 * Bugs:
 *
 ******************************************************************************/

#ifndef SRC_EEPROM_EEPROM_H_
#define SRC_EEPROM_EEPROM_H_

#include "../BIOS/bios.h"

/**************************************************
 * 		Macro   							  	  *
 *************************************************/
#define SECTOR5 0x08020000   //128Kbytes
#define SECTOR6 0x08040000   //128Kbytes
#define SECTOR7 0x08060004   //128Kbytes
#define SECTOR8 0x08080004   // 128kbytes

#define WRITE_STRUCT(__ADDRESS__, __EEPROM__)\
    EepromWriteStruct(__ADDRESS__, (uint32_t*)&(__EEPROM__), sizeof(__EEPROM__))

#define READ_STRUCT(__ADDRESS__, __EEPROM__)\
    EepromReadStruct(__ADDRESS__, (uint32_t*)&(__EEPROM__), sizeof(__EEPROM__))

/**************************************************
 * 		Structure							  	  *
 *************************************************/
#define NUM_INT 0
#define NUM_FLOAT 9

#define FUZP       = 0.330000  // varNum 0
#define FUZI       = 0.200000  // varNum 1
#define FUZD       = 0.150000  // varNum 2
#define FUZP_P     = 0.100000  // varNum 3
#define FUZI_P     = 0.020000  // varNum 4
#define FUZD_P     = 0.020000  // varNum 5


struct{
		float fuzP;
		float fuzI;
		float fuzD;
		float fuzP_P;
		float fuzI_P;
		float fuzD_P;
		float rbcP;
		float rbcI;
		float rbcD;
}E;

uint8_t tuneE;
uint8_t editE[7];
int Eedited, Ewritten;
/**************************************************
 * 		Enumerator							  	  *
 *************************************************/

/**************************************************
 * 		Extern	variables					  	  *
 *************************************************/

/**************************************************
 * 		Function Prototype			  			  *
 *************************************************/
uint32_t EepromWriteStruct (uint32_t Address, uint32_t* Eeprom, int size);
void EepromReadStruct(uint32_t Address, uint32_t* Eeprom, int size);
uint32_t GetSector(uint32_t Address);

#endif /* SRC_EEPROM_EEPROM_H_ */
