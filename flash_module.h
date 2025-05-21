#ifndef FLASH_MODULE_H
#define FLASH_MODULE_H

#include "config.h"
#include "data_structures.h"
#include "FlashTech_F28P55x_C28x.h"
#include "flash_programming_f28p55x.h"

// Flash initialization
int FlashAPI_init(void);

// Flash operations
void writeToFlash(void);
void Example_EraseSector(void);
void eraseBank4(void);
void writeDefaultCalibrationValues(void);

// Flash status management
void ClearFSMStatus(void);
void Example_Error(Fapi_StatusType status);
void Example_Done(void);
void FMSTAT_Fail(void);
void ECC_Fail(void);

// Sector management
void findReadAndWriteAddress(void);
void eraseSectorOrFindEmptySector(void);
uint16_t addressToSector(uint32_t address);
uint32_t findEmptySector(void);

// Data reading functions
uint16_t readFromFlash(uint32_t address, uint8_t* buffer, uint32_t length);

#endif // FLASH_MODULE_H
