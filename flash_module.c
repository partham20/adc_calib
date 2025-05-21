#include "flash_module.h"
#include "data_structures.h"

// Flash API Initialization
#pragma CODE_SECTION(FlashAPI_init, ".TI.ramfunc");
int FlashAPI_init(void)
{
    Flash_initModule(FLASH0CTRL_BASE, FLASH0ECC_BASE, 3);

    Fapi_StatusType oReturnCheck;

    oReturnCheck = Fapi_initializeAPI(FlashTech_CPU0_BASE_ADDRESS, DEVICE_SYSCLK_FREQ/1000000U);

    if (oReturnCheck != Fapi_Status_Success)
    {
        Example_Error(oReturnCheck);
    }

    oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank0);

    if (oReturnCheck != Fapi_Status_Success)
    {
        Example_Error(oReturnCheck);
    }

    return 0;
}

// Clear FSM Status
#pragma CODE_SECTION(ClearFSMStatus, ".TI.ramfunc");
void ClearFSMStatus(void)
{
    Fapi_FlashStatusType oFlashStatus;
    Fapi_StatusType oReturnCheck;

    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady){}

    oFlashStatus = Fapi_getFsmStatus();
    if (oFlashStatus != 0)
    {
        oReturnCheck = Fapi_issueAsyncCommand(Fapi_ClearStatus);

        while (Fapi_getFsmStatus() != 0) {}

        if (oReturnCheck != Fapi_Status_Success)
        {
            Example_Error(oReturnCheck);
        }
    }
}

// Write to Flash
#pragma CODE_SECTION(writeToFlash, ".TI.ramfunc");
void writeToFlash(void)
{
    uint32_t u32Index = 0;
    uint16_t i = 0;
    Fapi_StatusType oReturnCheck;
    Fapi_FlashStatusType oFlashStatus;
    Fapi_FlashStatusWordType oFlashStatusWord;

    for (i = 0, u32Index = addressToWrite;
         (u32Index < (addressToWrite + datasize));
         i += 8, u32Index += 8)
    {
        ClearFSMStatus();

        Fapi_setupBankSectorEnable(FLASH_WRAPPER_PROGRAM_BASE+FLASH_O_CMDWEPROTA, SEC0TO31);
        Fapi_setupBankSectorEnable(FLASH_WRAPPER_PROGRAM_BASE+FLASH_O_CMDWEPROTB, SEC32To127);

        oReturnCheck = Fapi_issueProgrammingCommand((uint32_t *)u32Index, Buffer+i,
                                               8, 0, 0, Fapi_AutoEccGeneration);

        while (Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);

        if (oReturnCheck != Fapi_Status_Success)
        {
            Example_Error(oReturnCheck);
        }

        oFlashStatus = Fapi_getFsmStatus();
        if (oFlashStatus != 3)
        {
            FMSTAT_Fail();
        }

        oReturnCheck = Fapi_doVerify((uint32_t *)u32Index,
                                     4, (uint32_t *)(uint32_t)(Buffer + i),
                                     &oFlashStatusWord);

        Fapi_setupBankSectorEnable(FLASH_WRAPPER_PROGRAM_BASE+FLASH_O_CMDWEPROTA, 0xFFFFFFFF);

        if (oReturnCheck != Fapi_Status_Success)
        {
            Example_Error(oReturnCheck);
        }
        if (oReturnCheck == Fapi_Status_Success)
        {
            Example_Done();
        }
    }
}

// Erase a sector
#pragma CODE_SECTION(Example_EraseSector, ".TI.ramfunc");
void Example_EraseSector(void)
{
    Fapi_StatusType oReturnCheck;
    Fapi_FlashStatusType oFlashStatus;
    Fapi_FlashStatusWordType oFlashStatusWord;

    ClearFSMStatus();
    Fapi_setupBankSectorEnable(FLASH_WRAPPER_PROGRAM_BASE+FLASH_O_CMDWEPROTA, SEC0TO31);
    Fapi_setupBankSectorEnable(FLASH_WRAPPER_PROGRAM_BASE+FLASH_O_CMDWEPROTB, SEC32To127);

    oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                   (uint32_t *)addressToWrite);

    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady){}

    if (oReturnCheck != Fapi_Status_Success)
    {
        Example_Error(oReturnCheck);
    }

    oFlashStatus = Fapi_getFsmStatus();
    if (oFlashStatus != 3)
    {
        FMSTAT_Fail();
    }

    oReturnCheck = Fapi_doBlankCheck((uint32_t *)addressToWrite,
                   Sector2KB_u32length,
                   &oFlashStatusWord);
    if (oReturnCheck != Fapi_Status_Success)
    {
        Example_Error(oReturnCheck);
    }
}

// Erase Bank 4
void eraseBank4(void)
{
    Fapi_StatusType oReturnCheck;
    Fapi_FlashStatusType oFlashStatus;
    Fapi_FlashStatusWordType oFlashStatusWord;
    uint32_t u32CurrentAddress = 0;

    ClearFSMStatus();

    // Enable program/erase protection for select sectors where this example is
    // located
    // CMDWEPROTA is applicable for sectors 0-31
    // Bits 0-11 of CMDWEPROTB is applicable for sectors 32-127, each bit represents
    // a group of 8 sectors, e.g bit 0 represents sectors 32-39, bit 1 represents
    // sectors 40-47, etc
    Fapi_setupBankSectorEnable(FLASH_WRAPPER_PROGRAM_BASE+FLASH_O_CMDWEPROTA, 0x00000000);
    Fapi_setupBankSectorEnable(FLASH_WRAPPER_PROGRAM_BASE+FLASH_O_CMDWEPROTB, 0xFFFFFFFF);

    u32CurrentAddress = FlashBank4StartAddress;

    // Issue bank erase command
    oReturnCheck = Fapi_issueBankEraseCommand((uint32_t *)u32CurrentAddress);

    // Wait until FSM is done with erase sector operation
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady){}

    if (oReturnCheck != Fapi_Status_Success)
    {
        // Check Flash API documentation for possible errors
        Example_Error(oReturnCheck);
    }

    // Read FMSTAT register contents to know the status of FSM after
    // erase command to see if there are any erase operation related errors
    oFlashStatus = Fapi_getFsmStatus();
    if (oFlashStatus != 3)
    {
        // Check Flash API documentation for FMSTAT and debug accordingly
        // Fapi_getFsmStatus() function gives the FMSTAT register contents.
        // Check to see if any of the EV bit, ESUSP bit, CSTAT bit or
        // VOLTSTAT bit is set (Refer to API documentation for more details).
        FMSTAT_Fail();
    }

    // Do blank check
    // Blank check is performed on all the sectors that are not protected
    // during Bank erase
    // Verify that Bank 2 is erased.
    // The Erase command itself does a verify as it goes.
    // Hence erase verify by CPU reads (Fapi_doBlankCheck()) is optional.
    u32CurrentAddress = FlashBank4StartAddress;
    oReturnCheck = Fapi_doBlankCheck((uint32_t *)u32CurrentAddress,
                   (8*Sector2KB_u32length),
                   &oFlashStatusWord);

    if (oReturnCheck != Fapi_Status_Success)
    {
        // Check Flash API documentation for error info
        Example_Error(oReturnCheck);
    }

    u32CurrentAddress = FlashBank4StartAddress + 0xC000;
    oReturnCheck = Fapi_doBlankCheck((uint32_t *)u32CurrentAddress,
                   (80*Sector2KB_u32length),
                   &oFlashStatusWord);

    if (oReturnCheck != Fapi_Status_Success)
    {
        // Check Flash API documentation for error info
        Example_Error(oReturnCheck);
    }
}

// Set the default calibration values
void writeDefaultCalibrationValues(void)
{
    if (newestData >= (WRAP_LIMIT))
    {
        eraseBank4();
        newestData = 0;
        oldestData = 0xFFFF;
        writeAddress = 0xFFFFFFFF;
        addressToWrite = BANK4_START;
    }

    // Initialize default data structure
    CalibData defaultPduData = { 0 }; // Zero initialize first

    // Set the main fields
    defaultPduData.sectorIndex = ++newestData;
    defaultPduData.BUBoardID = address;

    // Initialize gains arrays
    int i;
    for (i = 0; i < NumberOfBranches; i++)
    {
        defaultPduData.currentGain[i] = 885;
        defaultPduData.kwGain[i] = 9677;
    }

    // Update read/write structure
    memcpy(&pduManager.readWriteData, &defaultPduData, sizeof(CalibData));

    // Copy to Buffer for flash writing
    memcpy(Buffer, &pduManager.readWriteData, sizeof(CalibData));
    datasize = sizeof(CalibData) / sizeof(uint16_t);

    // Mark structures as out of sync
    pduManager.dataSync = false;
}

// Find the read and write addresses in flash
void findReadAndWriteAddress(void)
{
    // Local Variables
    uint32_t addr;
    uint16_t val;

    newestData = 0;
    oldestData = 0xFFFF;
    writeAddress = 0xFFFFFFFF;
    readAddress = BANK4_START;
    emptySectorFound = false;

    for (addr = BANK4_START; addr <= BANK4_END; addr += SECTOR_SIZE)
    {
        val = *(volatile uint16_t*)addr;

        if (val == 0xFFFF)
        {
            emptySectorFound = true;
            continue; // Empty sector
        }

        // Track newest
        if (val <= WRAP_LIMIT && val > newestData)
        {
            newestData = val;
            readAddress = addr;
        }

        // Track oldest
        if (val <= WRAP_LIMIT && val < oldestData && val > 0)
        {
            oldestData = val;
            writeAddress = addr;
        }
    }
}

// Find an empty sector or erase the oldest sector
void eraseSectorOrFindEmptySector(void)
{
    Fapi_StatusType oReturnCheck;
    Fapi_FlashStatusWordType oFlashStatusWord;
    uint32_t i;

    // To find an empty sector
    if (emptySectorFound)
    {
        for (i = BANK4_START; i <= BANK4_END; i += SECTOR_SIZE)
        {
            oReturnCheck = Fapi_doBlankCheck((uint32_t*)i,
                                            Sector2KB_u32length,
                                            &oFlashStatusWord);
            if (oReturnCheck == Fapi_Status_Success)
            {
                // Found an empty sector
                addressToWrite = i;
                return;
            }
        }
    }

    if ((oldestData < WRAP_LIMIT) && (newestData != WRAP_LIMIT))
    {
        addressToWrite = writeAddress;
        Example_EraseSector();  // Erase the oldest sector
        return;
    }
}

// Read from flash
uint16_t readFromFlash(uint32_t address, uint8_t* buffer, uint32_t length)
{
    uint32_t i;
    uint16_t bytesRead = 0;

    if (buffer == NULL)
    {
        return 0;
    }

    // Simple memory copy from flash address to buffer
    for (i = 0; i < length; i++)
    {
        buffer[i] = *((uint8_t*)(address + i));
        bytesRead++;
    }

    return bytesRead;
}

// Find an empty sector
uint32_t findEmptySector(void)
{
    uint32_t addr;
    Fapi_StatusType oReturnCheck;
    Fapi_FlashStatusWordType oFlashStatusWord;

    for (addr = BANK4_START; addr <= BANK4_END; addr += SECTOR_SIZE)
    {
        oReturnCheck = Fapi_doBlankCheck((uint32_t *)addr,
                                         Sector2KB_u32length,
                                         &oFlashStatusWord);
        if (oReturnCheck == Fapi_Status_Success)
        {
            // Found an empty sector
            return addr;
        }
    }

    // No empty sector found
    return 0xFFFFFFFF;
}

// Convert address to sector number
uint16_t addressToSector(uint32_t address)
{
    if (address < BANK4_START || address > BANK4_END)
    {
        return 0xFFFF; // Invalid address
    }

    return (uint16_t)((address - BANK4_START) / SECTOR_SIZE);
}

// Error handling functions
void Example_Error(Fapi_StatusType status)
{
    updateFailed = true;
}

void Example_Done(void)
{
    updateFailed = false;
}

void FMSTAT_Fail(void)
{
    updateFailed = true;
}

void ECC_Fail(void)
{
    __asm("    ESTOP0");
}

// Wrapper function for flash read operations with error checking
uint32_t flashReadWithVerify(uint32_t address, uint8_t* buffer, uint32_t length)
{
    if (buffer == NULL || address < BANK4_START || address > BANK4_END)
    {
        return 0; // Invalid parameters
    }

    // Perform the read operation
    uint32_t bytesRead = readFromFlash(address, buffer, length);

    // Simple verification - could be expanded for more robust checking
    if (bytesRead != length)
    {
        // Reading failed
        return 0;
    }

    return bytesRead;
}

// Function to check flash status
bool isFlashReady(void)
{
    return (Fapi_checkFsmForReady() == Fapi_Status_FsmReady);
}

// Function to check if a sector is empty
bool isSectorEmpty(uint32_t sectorAddress)
{
    Fapi_StatusType oReturnCheck;
    Fapi_FlashStatusWordType oFlashStatusWord;

    oReturnCheck = Fapi_doBlankCheck((uint32_t*)sectorAddress,
                                     Sector2KB_u32length,
                                     &oFlashStatusWord);

    return (oReturnCheck == Fapi_Status_Success);
}

// Function to get the total number of empty sectors
uint16_t getEmptySectorCount(void)
{
    uint16_t count = 0;
    uint32_t addr;

    for (addr = BANK4_START; addr <= BANK4_END; addr += SECTOR_SIZE)
    {
        if (isSectorEmpty(addr))
        {
            count++;
        }
    }

    return count;
}

// Function to check for flash wrapping condition
bool isFlashWrappingNeeded(void)
{
    return (newestData >= WRAP_LIMIT);
}

// Function to prepare flash for writing
bool prepareFlashForWriting(void)
{
    // Check if wraparound is needed
    if (isFlashWrappingNeeded())
    {
        eraseBank4();
        newestData = 0;
        oldestData = 0xFFFF;
        writeAddress = 0xFFFFFFFF;
        addressToWrite = BANK4_START;
    }

    // Find appropriate sector for writing
    findReadAndWriteAddress();

    // Either find an empty sector or erase the oldest one
    eraseSectorOrFindEmptySector();

    // Return success if we have a valid address to write to
    return (addressToWrite != 0xFFFFFFFF);
}

// Function to backup current calibration data before erasing
bool backupCalibrationData(CalibData* backupData)
{
    if (backupData == NULL || readAddress == 0xFFFFFFFF)
    {
        return false;
    }

    // Copy current data from flash to backup
    memcpy(backupData, (void*)readAddress, sizeof(CalibData));
    return true;
}

// Function to restore calibration data from backup
bool restoreCalibrationData(const CalibData* backupData)
{
    if (backupData == NULL)
    {
        return false;
    }

    // Prepare flash for writing
    if (!prepareFlashForWriting())
    {
        return false;
    }

    // Copy backup to read/write structure
    memcpy(&pduManager.readWriteData, backupData, sizeof(CalibData));

    // Increment sector index
    pduManager.readWriteData.sectorIndex = ++newestData;

    // Copy to Buffer for flash writing
    memcpy(Buffer, &pduManager.readWriteData, sizeof(CalibData));
    datasize = sizeof(CalibData) / sizeof(uint16_t);

    // Mark structures as out of sync
    pduManager.dataSync = false;

    // Write to flash
    writeToFlash();

    return !updateFailed;
}

// Function to safely update flash with new calibration data
bool updateCalibrationData(const CalibData* newData)
{
    if (newData == NULL)
    {
        return false;
    }

    // Prepare flash for writing
    if (!prepareFlashForWriting())
    {
        return false;
    }

    // Copy new data to read/write structure
    memcpy(&pduManager.readWriteData, newData, sizeof(CalibData));

    // Make sure sector index is incremented
    pduManager.readWriteData.sectorIndex = ++newestData;

    // Copy to Buffer for flash writing
    memcpy(Buffer, &pduManager.readWriteData, sizeof(CalibData));
    datasize = sizeof(CalibData) / sizeof(uint16_t);

    // Mark structures as out of sync
    pduManager.dataSync = false;

    // Write to flash
    writeToFlash();

    return !updateFailed;
}

// Function to initialize flash and load calibration data
bool initializeFlashAndLoadData(void)
{
    // Initialize Flash API
    FlashAPI_init();

    // Find read and write addresses
    findReadAndWriteAddress();

    // If no calibration data exists, write defaults
    if (newestData == 0)
    {
        writeAddress = BANK4_START;
        writeDefaultCalibrationValues();
        writeToFlash();

        if (!updateFailed)
        {
            // Copy read/write data to working structure
            syncReadWriteToWorking();
            readAddress = BANK4_START;
            return true;
        }
        else
        {
            // Flash write failed
            return false;
        }
    }
    else
    {
        // Load existing data from flash
        memcpy(&pduManager.readWriteData, (void*)readAddress, sizeof(CalibData));
        syncReadWriteToWorking();
        return true;
    }
}

// Memory diagnostic functions
void reportFlashStatus(void)
{
    // For debug purposes - could output to UART, etc.
    // These values would normally be logged or displayed
    uint16_t emptyCount = getEmptySectorCount();
    uint16_t totalSectors = (BANK4_END - BANK4_START) / SECTOR_SIZE;
    uint16_t usedSectors = totalSectors - emptyCount;
    uint16_t percentUsed = (usedSectors * 100) / totalSectors;

    // This function would typically log or display these values
    // For now, these variables are just calculated but not used
}
