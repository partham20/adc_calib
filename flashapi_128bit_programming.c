#include "bu_adc.h"
#include "config.h"
#include "data_structures.h"
#include "can_module.h"
#include "flash_module.h"
#include "timer_module.h"
#include "board_module.h"
#include "utilities.h"

// Define global variables
PDUDataManager pduManager;
CANFrame storedFrames[NUM_FRAMES];
uint16_t buffer[TOTAL_BUFFER_SIZE];
uint16_t receivedBuffer[TOTAL_BUFFER_SIZE];
uint16_t Buffer[WORDS_IN_FLASH_BUFFER];
uint32_t *Buffer32 = (uint32_t *)Buffer;
uint32_t address;

// Flash-related globals
uint32_t addressToWrite = BANK4_START;
uint16_t newestData = 0;
uint16_t oldestData = 0xFFFF;
uint32_t writeAddress = 0xFFFFFFFF;
uint32_t readAddress = BANK4_START;
bool emptySectorFound = false;
size_t datasize;

// CAN-related globals
MCAN_RxBufElement rxMsg[MCAN_RX_BUFF_NUM], rxFIFO0Msg[MCAN_FIFO_0_NUM];
MCAN_TxBufElement txMsg[NUM_OF_MSG];
uint8_t retransmissionData[64];

// Flags and state variables
bool flashWriteEnabled = false;
bool CommandReceived = false;
bool updateFailed = false;
bool bufferFull = false;
bool discoveryInProgress = false;
bool discoveryAckReceived = false;
bool timerExpired = false;
bool allFramesReceived = false;
bool requestRetransmission = false;
uint16_t receivedFramesMask = 0;
uint8_t retryCounter = 0;
uint8_t discoveryRetryCount = 0;




/**
 * Main function - entry point for firmware execution
 */
void main(void)
{
    // Initialize device hardware and peripherals
    startupfunc();

    // Initialize flash data if needed
    if (newestData == 0)
    {
        // Write default calibration data
        writeAddress = BANK4_START;
        writeDefaultCalibrationValues();
        writeToFlash();

        if (!updateFailed)
        {
            // Only sync to working data if flash write was successful
            syncReadWriteToWorking();
        }
        readAddress = BANK4_START;
    }
    else
    {
        // Load existing data from flash
        memcpy(&pduManager.readWriteData, (void*)readAddress,
               sizeof(CalibData));
        syncReadWriteToWorking();
    }

    // Send calibration data format version details
    sendCalibrationDataFormatVersion();

    // Prepare and transmit frames with calibration data
    prepareFramesForTransmission((uint16*)(readAddress + 2),
                                 calibrationdDataSize);
    transmit();

    // Main processing loop
    while (1)
    {
        // Check if discovery process is active
        if (discoveryInProgress)
        {
            DEVICE_DELAY_US(10000);
            handleDiscoveryProcess();
        }

        // Handle gain retransmission if needed
        handleGainRetransmission();

        // Handle frame retransmission requests
        if (requestRetransmission)
        {
            readCANAddress();
            handleMissingFrames();
            requestRetransmission = false;
        }

        // Process received FIFO buffer when full
        if (bufferFull == true)
        {
            readCANAddress();
            copyRxFIFO();
        }

        // Process received commands
        if (CommandReceived)
        {
            readCANAddress();

            // Handle discovery start command
            if (rxMsg[4].data[0] == DISCOVERY_START)
            {
                sendDiscoveryMessage();
            }

            // Handle discovery end command
            if (rxMsg[4].data[0] == DISCOVERY_END)
            {
                resetDiscoveryFlags(); // Reset flags when discovery ends
                findReadAndWriteAddress();
                prepareFramesForTransmission((uint16*)(readAddress + 2),
                                             calibrationdDataSize);
                startGainTransmissionRetry();
            }

            // Handle calibration version check command (1)
            if (rxMsg[4].data[0] == 1)
            {
                sendCalibrationDataFormatVersion();
            }

            // Handle send calibration data command (3)
            else if (rxMsg[4].data[0] == 3)
            {
                findReadAndWriteAddress();
                prepareFramesForTransmission((uint16*)(readAddress + 2),
                                             calibrationdDataSize);
                transmit();
            }

            // Handle erase bank 4 command (9)
            else if (rxMsg[4].data[0] == 9)
            {
                eraseBank4();
                erasedBank4Ack();
            }

            // Handle write default values command (0xA)
            else if (rxMsg[4].data[0] == 0xA)
            {
                // Prepare flash for writing
                findReadAndWriteAddress();
                eraseSectorOrFindEmptySector();

                // Write default calibration values
                writeDefaultCalibrationValues();
                writeToFlash();

                writtenDefaultCalibrationValuesAck();
            }

            // Sync working data if flash write was successful
            if (!updateFailed)
            {
                syncReadWriteToWorking();
            }

            CommandReceived = false;
        }

        // Process any address changes
        checkAddressChange();

        BU_ADC_processLoop();

        // System delay
        DEVICE_DELAY_US(10000);
    }
}


