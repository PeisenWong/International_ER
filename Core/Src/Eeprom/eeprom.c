
/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "eeprom.h"
/*********************************************/

/*********************************************/
/*          MACRO                            */
/*********************************************/

/*********************************************/
/*          Variable                         */
/*********************************************/

/*********************************************/

/*********************************************/
/*           Subroutine Function             */
/*********************************************/

uint32_t GetSector(uint32_t Address)
{
  static uint32_t sector = 0;

  if((Address < 0x08003FFF) && (Address >= 0x08000000))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < 0x08007FFF) && (Address >= 0x08004000))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < 0x0800BFFF) && (Address >= 0x08008000))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < 0x0800FFFF) && (Address >= 0x0800C000))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < 0x0801FFFF) && (Address >= 0x08010000))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < 0x0803FFFF) && (Address >= 0x08020000))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < 0x0805FFFF) && (Address >= 0x08040000))
  {
    sector = FLASH_SECTOR_6;
  }
  else if((Address < 0x0807FFFF) && (Address >= 0x08060000))
  {
    sector = FLASH_SECTOR_7;
  }
  return sector;
}


/*
 * Function Name		: laser_init
 * Function Description : This function is called to calibrate laser_sensor.
 * Function Remarks		: User need to initialize ADC and KF first before calling this function. To Tune Max Distance use 1 meter ruler to check accuracy.
 * Function Arguments	: min_value			minimum value of ADC/KF return (float for KF)
 * 						  max_value			maximum value of ADC/KF return (float for KF)
 * 						  min_distance 		minimum distance,in meter when min_value detected (can take from encoder)
 * 						  max_distance		maximum distance,in meter when max_value detected (can take from encoder)
 * 						  input				pointer to input of laser (can be either from ADC or KF )
 * 						  output			pointer to output of laser
 * 						  laser				pointer to structure LASER_t
 * Function Return		: None
 * Function Example		: EepromWriteStruct(SECTOR7 , (uint32_t *)&E, sizeof(E))
 * 						  or use MACRO
 * 						  WRITE_STRUCT(SECTOR7, E);
 */
uint32_t EepromWriteStruct (uint32_t Address, uint32_t* Eeprom, int size)
{
	uint32_t success;
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError;
	HAL_FLASH_Unlock();

	uint32_t StartSector = GetSector(Address);
	uint32_t EndSectorAddress = Address + size;
	uint32_t EndSector = GetSector(EndSectorAddress);

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector        = StartSector;
	EraseInitStruct.NbSectors     = (EndSector - StartSector) + 1;
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	{
		return HAL_FLASH_GetError ();
	}
	int i;
	for(i=0; i<size/4; i++){
		if(i < NUM_INT){
			if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, *Eeprom++)==HAL_OK){
				success ++;
			}
		}else if(i >= NUM_INT){
			uint32_t temp;
			memcpy(&temp, Eeprom++, 4);
			if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, temp)==HAL_OK){
				success ++;
			}
		}
		Address += 4;
	}
	HAL_FLASH_Lock();
	return success;
}

/*
 * Function Name		: LaserUpdate_min_value
 * Function Description : This function is called to update min_value structure laser_t
 * Function Remarks		: User need to initialize LaserInit first before calling this function.
 * Function Arguments	: min_value			minimum value of ADC/KF return
 * 						  laser				pointer to structure laser_t
 * Function Return		: None
 * Function Example		: EepromReadStruct(SECTOR7, (uint32_t *)&E, sizeof(E))
 * 						  or use MACRO
 * 						  READ_STRUCT(SECTOR7, E);
 */
void EepromReadStruct(uint32_t Address, uint32_t* Eeprom, int size)
{
    memcpy(Eeprom, (__IO uint32_t *)Address, size);
}

