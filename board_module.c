#include <bu_adc.h>
#include "board_module.h"
#include "can_module.h"
#include "timer_module.h"
#include "flash_module.h"

// External variable declarations
extern uint16_t firstBUFrameData[64];
extern uint16_t secondBUFrameData[64];
extern bool firstBUFrameReceived;
extern bool secondBUFrameReceived;
extern int frameCounter;
extern bool sequenceStarted;


// Initialize DIP switches for address configuration
void initDIPSwitchGPIO(void)
{
   //
   // Configure GPIO40 - DIP Switch 1
   //
   GPIO_setPadConfig(40, GPIO_PIN_TYPE_PULLUP | GPIO_PIN_TYPE_INVERT); // Enable pullup on GPIO40 //Remove invert to make ON to OFF
   GPIO_setPinConfig(GPIO_40_GPIO40);               // GPIO40 = GPIO40
   GPIO_setDirectionMode(40, GPIO_DIR_MODE_IN);     // GPIO40 = input
   GPIO_setQualificationMode(40, GPIO_QUAL_SYNC);   // Synch to SYSCLKOUT

   //
   // Configure GPIO23 - DIP Switch 2
   //
   GPIO_setPadConfig(23, GPIO_PIN_TYPE_PULLUP | GPIO_PIN_TYPE_INVERT); // Enable pullup on GPIO23
   GPIO_setPinConfig(GPIO_23_GPIO23);               // GPIO23 = GPIO23
   GPIO_setDirectionMode(23, GPIO_DIR_MODE_IN);     // GPIO23 = input
   GPIO_setQualificationMode(23, GPIO_QUAL_SYNC);   // Synch to SYSCLKOUT

   //
   // Configure GPIO41 - DIP Switch 3
   //
   GPIO_setPadConfig(41, GPIO_PIN_TYPE_PULLUP | GPIO_PIN_TYPE_INVERT); // Enable pullup on GPIO41
   GPIO_setPinConfig(GPIO_41_GPIO41);               // GPIO41 = GPIO41
   GPIO_setDirectionMode(41, GPIO_DIR_MODE_IN);     // GPIO41 = input
   GPIO_setQualificationMode(41, GPIO_QUAL_SYNC);   // Synch to SYSCLKOUT

   //
   // Configure GPIO22 - DIP Switch 4
   //
   GPIO_setPadConfig(22, GPIO_PIN_TYPE_PULLUP | GPIO_PIN_TYPE_INVERT); // Enable pullup on GPIO22
   GPIO_setPinConfig(GPIO_22_GPIO22);               // GPIO22 = GPIO22
   GPIO_setDirectionMode(22, GPIO_DIR_MODE_IN);     // GPIO22 = input
   GPIO_setQualificationMode(22, GPIO_QUAL_SYNC);   // Synch to SYSCLKOUT

   //
   // Set qualification sampling period
   //
   GPIO_setQualificationPeriod(22, 6);    // Qualification period = SYSCLKOUT/6
}

// Read individual DIP switch state
uint32_t readDIPSwitch(uint32_t gpioNumber)
{
   //
   // Return inverted value since switches are active low with pullup
   //
   return (!GPIO_readPin(gpioNumber));
}

// Read full CAN address from all DIP switches
void readCANAddress(void)
{
   //
   // Read each switch and compose 4-bit address
   //
   address = (readDIPSwitch(22) << 3) |  // DIP4 - MSB
           (readDIPSwitch(41) << 2) |  // DIP3
           (readDIPSwitch(23) << 1) |  // DIP2
           readDIPSwitch(40);          // DIP1 - LSB
}

// Process BU board data received from CAN
void processBUBoardData(void)
{
   // Each BU board has 18 CTs

   // Calculate starting indices based on board address
   // Board addresses start from 1, so subtract 1 for zero-based indexing
   uint32_t boardIndex = address - 1;

   // Calculate starting positions for current and kW gains
   uint32_t currentGainStartIndex = boardIndex * CTS_PER_BOARD;
   uint32_t kwGainStartIndex = (NUM_BU_BOARDS * CTS_PER_BOARD)
           + (boardIndex * CTS_PER_BOARD);

   // Reset the working data structure
   memset(&pduManager.readWriteData, 0, sizeof(CalibData));

   // Store board ID
   pduManager.readWriteData.BUBoardID = address;

   // Extract current gains for this board
   int i;
   for (i = 0; i < CTS_PER_BOARD; i++)
   {
       pduManager.readWriteData.currentGain[i] =
               receivedBuffer[currentGainStartIndex + i];
       pduManager.readWriteData.kwGain[i] =
               receivedBuffer[kwGainStartIndex + i];
   }

   // Update sector index for flash management
   if (newestData >= (WRAP_LIMIT))
   {
       eraseBank4();
       newestData = 0;
       oldestData = 0xFFFF;
       writeAddress = 0xFFFFFFFF;
       addressToWrite = BANK4_START;
   }

   pduManager.readWriteData.sectorIndex = ++newestData;

   // Copy to Buffer for flash writing
   memcpy(Buffer, &pduManager.readWriteData, sizeof(CalibData));
   datasize = sizeof(CalibData) / sizeof(uint16_t);

   // Mark structures as out of sync
   pduManager.dataSync = false;

   // Write to flash
   findReadAndWriteAddress();
   eraseSectorOrFindEmptySector();
   writeToFlash();

   // If flash write was successful, sync the data
   if (!updateFailed)
   {
       syncReadWriteToWorking();
       calibDataUpdateSuccessAck();
   }
   else
   {
       calibDataUpdateFailAck();
   }
}

// Reset discovery flags
void resetDiscoveryFlags(void)
{
   discoveryInProgress = false;
   discoveryAckReceived = false;
   discoveryRetryCount = 0;
   timerExpired = false;
   CPUTimer_stopTimer(CPUTIMER1_BASE);
}

// Reset frame tracking variables
void resetFrameTrackingVariables(void)
{
   receivedFramesMask = 0;
   allFramesReceived = false;
   sequenceStarted = false;
   frameCounter = 0;
   timerExpired = false;
   requestRetransmission = false;
   retryCounter = 0;
   CPUTimer_stopTimer(CPUTIMER1_BASE);
}

// Handle discovery process - retries if needed
void handleDiscoveryProcess(void)
{
   if (discoveryInProgress && !discoveryAckReceived)
   {
       if (timerExpired)
       {
           discoveryRetryCount++;

           if (discoveryRetryCount >= MAX_DISCOVERY_RETRIES)
           {
               // Max retries reached, reset all flags
               resetDiscoveryFlags();

               // Send failure notification
               maxRetriesExceeded();
           }
           else
           {
               // Retry sending discovery message
               sendDiscoveryMessage();
           }
       }
   }
}

// Handle gain retransmission
void handleGainRetransmission(void)
{
   if (timer2Expired && !gainAckReceived)
   {
       retryCount++;
       if (retryCount >= MAX_GAIN_RETRIES)
       {
           // Max retries reached, handle failure
           CPUTimer_stopTimer(CPUTIMER2_BASE);
           // Add any failure handling code here
           return;
       }

       // Retransmit frames
       transmitGainFrames();

       // Reset timer expired flag and restart timer
       timer2Expired = false;
       CPUTimer_reloadTimerCounter(CPUTIMER2_BASE);
       CPUTimer_startTimer(CPUTIMER2_BASE);
   }
}


// Send gains to BU board
void sendGainsToBuBoard(int buid)
{
   // Implementation would depend on system requirements
   // This is a placeholder function
}

// Read gains from BU board
void readGainsFromBuBoard(int buid)
{
   // Implementation would depend on system requirements
   // This is a placeholder function
}

// Device initialization
void startupfunc(void)
{
   Device_init();
   initDIPSwitchGPIO();
   // Initialize ePWM for ADC triggering
   Board_init();

   initEPWM();

   readCANAddress();
   //Initialize Flash API
   FlashAPI_init();

   //Finds the address to read from and sector Address to write to
   findReadAndWriteAddress();

   // Initialize both data structures
   initializePDUManager();

   //
   // Initialize GPIO and unlock the GPIO configuration registers
   //
   Device_initGPIO();

   Interrupt_initModule();

   Interrupt_initVectorTable();

   BU_ADC_init();

   // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
   EINT;
   ERTM;

   //
   // Configure the divisor for the MCAN bit-clock
   //
   SysCtl_setMCANClk(SYSCTL_MCANA, SYSCTL_MCANCLK_DIV_5);

   //
   // ISR Configuration.
   //
   MCANIntrConfig();

   //
   // Configure GPIO pins for MCANTX/MCANRX operation
   //
   GPIO_setPinConfig(GPIO_0_MCANA_RX);
   GPIO_setPinConfig(GPIO_1_MCANA_TX);

   //
   // Configure the MCAN Module.
   //
   MCANConfig();
   initFrameTracking();
   //
   // Enable Interrupts.
   //
   MCAN_enableIntr(MCANA_DRIVER_BASE, MCAN_INTR_MASK_ALL, 1U);

   //
   // Select Interrupt Line.
   //
   MCAN_selectIntrLine(MCANA_DRIVER_BASE, MCAN_INTR_MASK_ALL,
                       MCAN_INTR_LINE_NUM_1);

   //
   // Enable Interrupt Line.
   //
   MCAN_enableIntrLine(MCANA_DRIVER_BASE, MCAN_INTR_LINE_NUM_1, 1U);

   configureTimer2();  // Initialize Timer2 for retransmission handling
}

// Check if address has changed and take appropriate action
void checkAddressChange(void)
{
   static uint32_t previousAddress = 0xFFFFFFFF;  // Initial invalid value

   // Read the current address
   readCANAddress();

   // If this is the first check or address has changed
   if (previousAddress == 0xFFFFFFFF || previousAddress != address)
   {
       // Only take action if this isn't the first check
       if (previousAddress != 0xFFFFFFFF)
       {
           // Address changed, reset CAN module with new address
           resetCANModule();

           // Reset all state related to communication
           resetDiscoveryFlags();
           resetFrameTrackingVariables();

           // Reload calibration data for the new address
           findReadAndWriteAddress();

           // Update flash with new address
           if (newestData > 0)
           {
               // Update existing data with new address
               pduManager.readWriteData.BUBoardID = address;

               // Prepare and write to flash
               memcpy(Buffer, &pduManager.readWriteData, sizeof(CalibData));
               datasize = sizeof(CalibData) / sizeof(uint16_t);

               eraseSectorOrFindEmptySector();
               writeToFlash();

               if (!updateFailed)
               {
                   syncReadWriteToWorking();
               }
           }
           else
           {
               // No existing data, write defaults
               writeDefaultCalibrationValues();
               writeToFlash();

               if (!updateFailed)
               {
                   syncReadWriteToWorking();
               }
           }

           // Send calibration data format version with new address
           sendCalibrationDataFormatVersion();
       }

       // Update previous address
       previousAddress = address;
   }
}

// Self-test function for board hardware
bool performSelfTest(void)
{
   bool status = true;

   // Test GPIO functionality
   status &= testGPIO();

   // Test CAN functionality
   status &= testCAN();

   // Test flash functionality
   status &= testFlash();

   return status;
}

// Test GPIO functionality
bool testGPIO(void)
{
   // Read all DIP switches to ensure they're working
   readCANAddress();

   // A more comprehensive test would set/read GPIO pins that
   // are configured as outputs, but this depends on hardware design

   // For now, simply return true
   return true;
}

// Test CAN functionality
bool testCAN(void)
{
   // Check if MCAN module is properly initialized
   if (MCAN_getOpMode(MCANA_DRIVER_BASE) != MCAN_OPERATION_MODE_NORMAL)
   {
       return false;
   }

   // More comprehensive tests would send/receive a test message,
   // but this requires a CAN transceiver and another node

   return true;
}

// Test flash functionality
bool testFlash(void)
{
   // Check if flash is ready
   if (Fapi_checkFsmForReady() != Fapi_Status_FsmReady)
   {
       return false;
   }

   // More comprehensive tests would read/write to a test sector,
   // but this requires careful handling to avoid corrupting data

   return true;
}

// Initialize all board hardware
void initBoardHardware(void)
{
   // Initialize device
   Device_init();

   // Initialize GPIO
   Device_initGPIO();

   // Initialize DIP switches
   initDIPSwitchGPIO();

   // Read address from DIP switches
   readCANAddress();

   // Initialize interrupts
   Interrupt_initModule();
   Interrupt_initVectorTable();

   // Enable global interrupts
   EINT;
   ERTM;

   // Initialize flash
   FlashAPI_init();

   // Initialize CAN
   SysCtl_setMCANClk(SYSCTL_MCANA, SYSCTL_MCANCLK_DIV_5);

   // Configure GPIO pins for CAN
   GPIO_setPinConfig(GPIO_0_MCANA_RX);
   GPIO_setPinConfig(GPIO_1_MCANA_TX);

   // Initialize timers
   initCPUTimers();
   configureTimer2();

   // Additional hardware initialization could be added here
}

// Power management function (if supported by hardware)
void setPowerMode(uint8_t mode)
{
   // Implementation depends on hardware support
   // 0 = Normal mode
   // 1 = Low power mode
   // 2 = Ultra low power mode

   switch (mode)
   {
       case 0:  // Normal mode
           // Restore all clocks and peripherals
           break;

       case 1:  // Low power mode
           // Reduce clock speeds, disable non-essential peripherals
           break;

       case 2:  // Ultra low power mode
           // Minimum clock speeds, only essential peripherals active
           break;

       default:
           // Invalid mode, do nothing
           break;
   }
}

// Board diagnostics function
void runBoardDiagnostics(void)
{
    // Test core components and report status
    bool gpioStatus = testGPIO();
    bool canStatus = testCAN();
    bool flashStatus = testFlash();

    // Check overall system health
    if (gpioStatus && canStatus && flashStatus)
    {
        // All systems operational - report success
        // This could send a CAN message or set a status LED
    }
    else
    {
        // Error detected - report failure details
        // This could send error codes via CAN or flash an error pattern on LEDs
    }
}

// Function to configure GPIO for LED status indication
void initStatusLEDs(void)
{
    // This implementation assumes specific GPIO pins are connected to LEDs
    // Modify according to actual hardware configuration

    // Example: Configure GPIO12 as an output for status LED
    GPIO_setPinConfig(GPIO_12_GPIO12);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // Example: Configure GPIO13 as an output for error LED
    GPIO_setPinConfig(GPIO_13_GPIO13);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);
}

// Function to set status LED state
void setStatusLED(bool state)
{
    // Set the status LED (assumed to be on GPIO12)
    GPIO_writePin(12, state ? 1 : 0);
}

// Function to set error LED state
void setErrorLED(bool state)
{
    // Set the error LED (assumed to be on GPIO13)
    GPIO_writePin(13, state ? 1 : 0);
}

// Function to flash LED in a pattern (count times)
void flashLED(uint32_t gpioPin, uint32_t count, uint32_t onTimeMs, uint32_t offTimeMs)
{
    uint32_t i;

    for (i = 0; i < count; i++)
    {
        // Turn LED on
        GPIO_writePin(gpioPin, 1);
        DEVICE_DELAY_US(onTimeMs * 1000);

        // Turn LED off
        GPIO_writePin(gpioPin, 0);
        DEVICE_DELAY_US(offTimeMs * 1000);
    }
}

// Function to display error code via LED flashing pattern
void displayErrorCode(uint8_t errorCode)
{
    // Assuming error LED is on GPIO13
    uint32_t errorLED = 13;

    // First, long flash to indicate start of error code
    flashLED(errorLED, 1, 1000, 500);

    // Small delay before error code
    DEVICE_DELAY_US(1000 * 1000);

    // Flash LED according to error code
    flashLED(errorLED, errorCode, 200, 300);
}

// Function to configure system clock
void configureSystemClock(void)
{
    // Configure system clock according to application requirements
    // This is a placeholder function - implementation depends on specific hardware
}

// Function to initialize external memory if available
void initExternalMemory(void)
{
    // Initialize external memory if present
    // This is a placeholder function - implementation depends on specific hardware
}

// Function to check board hardware version
uint8_t getBoardHardwareVersion(void)
{
    // This could read hardware version from GPIO pins, EEPROM, or other means
    // For this example, return a fixed value
    return 1;
}

// Function to check board firmware version
uint16_t getBoardFirmwareVersion(void)
{
    // Combine major and minor version numbers
    return (majorVersionNumber << 8) | minorVersionNumber;
}

// Function to handle watchdog timeout reset
void handleWatchdogReset(void)
{
    // Check if the device was reset by watchdog
    // This is a placeholder function - implementation depends on specific hardware

    // If watchdog reset occurred, perform recovery actions:
    // 1. Log the event
    // 2. Restore default calibration if needed
    // 3. Reinitialize hardware
}

// Function to initialize and configure ADC if used
void initADC(void)
{
    // Initialize ADC module if used in the application
    // This is a placeholder function - implementation depends on specific hardware
}

// Function to compare board address with stored value
bool hasAddressChanged(void)
{
    // Read current address from DIP switches
    readCANAddress();

    // Compare with stored address in flash
    // This assumes the address is stored in the calibration data
    if (pduManager.workingData.BUBoardID != address)
    {
        return true;
    }

    return false;
}

// Function to update board information with current settings
void updateBoardInfo(void)
{
    // Update board info structure with current settings
    // This includes address, version info, and other board-specific parameters

    // This is a placeholder function - implementation depends on application requirements
}

// Function to perform device reset
void performDeviceReset(void)
{
    // Disable interrupts
    DINT;

    // Disable peripherals

    // Reset the device
    // This is a placeholder - actual implementation depends on hardware

    // The following would typically force a CPU reset
    asm(" ESTOP0");
}

// Function to initialize the DMA controller if used
void initDMA(void)
{
    // Initialize DMA controller if used by the application
    // This is a placeholder function - implementation depends on specific hardware
}

// Function to handle critical faults
void handleCriticalFault(uint8_t faultCode)
{
    // Log the fault

    // Display fault code via LED
    displayErrorCode(faultCode);

    // For critical faults, either:
    // 1. Enter a safe state and wait for reset, or
    // 2. Reset the device

    // For this example, reset the device
    performDeviceReset();
}

// Function to enter a low-power standby mode
void enterStandbyMode(void)
{
    // Configure peripherals for low power

    // Enter low power mode
    // This is a placeholder function - implementation depends on specific hardware
}

// Function to exit low-power standby mode
void exitStandbyMode(void)
{
    // Restore normal operation after standby
    // This is a placeholder function - implementation depends on specific hardware
}

// Function to check if a POR (Power-On Reset) occurred
bool isPowerOnReset(void)
{
    // Check if this is a power-on reset as opposed to other reset types
    // This is a placeholder function - implementation depends on specific hardware
    return true;
}
