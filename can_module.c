#include "can_module.h"
#include "data_structures.h"
#include "timer_module.h"
#include "board_module.h"
#include "flash_module.h"
#include "utilities.h"

bool firstBUFrameReceived = false;
bool secondBUFrameReceived = false;
bool sequenceStarted = false;
 uint16_t firstBUFrameData[64];
 uint16_t secondBUFrameData[64];
 int frameCounter;
int error;


// CAN Configuration function
void MCANConfig(void)
{
    MCAN_InitParams initParams;
    MCAN_MsgRAMConfigParams msgRAMConfigParams;
    MCAN_StdMsgIDFilterElement stdFiltelem;
    MCAN_BitTimingParams bitTimes;

    //
    //  Initializing all structs to zero to prevent stray values
    //
    memset(&initParams, 0, sizeof(initParams));
    memset(&msgRAMConfigParams, 0, sizeof(msgRAMConfigParams));
    memset(&stdFiltelem, 0, sizeof(stdFiltelem));
    memset(&bitTimes, 0, sizeof(bitTimes));

    //
    // Configure MCAN initialization parameters
    //
    initParams.fdMode = 0x1U; // FD operation enabled.
    initParams.brsEnable = 0x1U; // Bit rate switching enabled

    //
    // Initialize Message RAM Sections Configuration Parameters
    //
    msgRAMConfigParams.flssa = MCAN_STD_ID_FILT_START_ADDR;
    msgRAMConfigParams.lss = MCAN_STD_ID_FILTER_NUM;
    msgRAMConfigParams.rxBufStartAddr = MCAN_RX_BUFF_START_ADDR;
    msgRAMConfigParams.rxBufElemSize = MCAN_RX_BUFF_ELEM_SIZE;
    msgRAMConfigParams.txStartAddr = MCAN_TX_BUFF_START_ADDR;
    msgRAMConfigParams.txBufNum = MCAN_TX_BUFF_SIZE;
    msgRAMConfigParams.txFIFOSize = MCAN_TX_FQ_SIZE;
    msgRAMConfigParams.txBufElemSize = MCAN_TX_BUFF_ELEM_SIZE;
    msgRAMConfigParams.txEventFIFOStartAddr = MCAN_TX_EVENT_START_ADDR;
    msgRAMConfigParams.txEventFIFOSize = MCAN_TX_EVENT_SIZE;
    msgRAMConfigParams.rxFIFO0startAddr = MCAN_FIFO_0_START_ADDR;
    msgRAMConfigParams.rxFIFO0size = MCAN_FIFO_0_NUM;
    msgRAMConfigParams.rxFIFO0OpMode = 1U; //FIFO Overwrite
    msgRAMConfigParams.rxFIFO0ElemSize = MCAN_FIFO_0_ELEM_SIZE;

    //
    // Initialize bit timings.
    //
    bitTimes.nomRatePrescalar = 0xBU; // Nominal Baud Rate Pre-scaler
    bitTimes.nomTimeSeg1 = 0x2U; // Nominal Time segment before SP
    bitTimes.nomTimeSeg2 = 0x0U; // Nominal Time segment after SP
    bitTimes.nomSynchJumpWidth = 0x0U; // Nominal SJW
    bitTimes.dataRatePrescalar = 0x2U; // Data Baud Rate Pre-scaler
    bitTimes.dataTimeSeg1 = 0x2U; // Data Time segment before SP
    bitTimes.dataTimeSeg2 = 0x0U; // Data Time segment after SP
    bitTimes.dataSynchJumpWidth = 0x0U; // Data SJW

    //
    // Wait for memory initialization to happen.
    //
    while (FALSE == MCAN_isMemInitDone(MCANA_DRIVER_BASE))
    {
    }

    //
    // Put MCAN in SW initialization mode.
    //
    MCAN_setOpMode(MCANA_DRIVER_BASE, MCAN_OPERATION_MODE_SW_INIT);

    //
    // Wait till MCAN is not initialized.
    //
    while (MCAN_OPERATION_MODE_SW_INIT != MCAN_getOpMode(MCANA_DRIVER_BASE))
    {
    }

    // Initialize MCAN module.
    MCAN_init(MCANA_DRIVER_BASE, &initParams);

    // Configure Bit timings.
    MCAN_setBitTime(MCANA_DRIVER_BASE, &bitTimes);

    // Configure Message RAM Sections
    MCAN_msgRAMConfig(MCANA_DRIVER_BASE, &msgRAMConfigParams);

    // Configure Standard Filters for Dedicated RX Buffers
    stdFiltelem.sfec = MCAN_STDFILTEC_RXBUFF;
    stdFiltelem.sft = MCAN_STDFILT_RANGE;

    // First set up filters for ID 3 and 5
    stdFiltelem.sfid2 = 4;
    stdFiltelem.sfid1 = 4;
    MCAN_addStdMsgIDFilter(MCANA_DRIVER_BASE, 0, &stdFiltelem);

    stdFiltelem.sfid2 = 5 + address;
    stdFiltelem.sfid1 = 5 + address;
    MCAN_addStdMsgIDFilter(MCANA_DRIVER_BASE, 1, &stdFiltelem);

    // Standard Filter for RX FIFO Buffers
    stdFiltelem.sfec = MCAN_STDFILTEC_FIFO0;
    stdFiltelem.sfid2 = 0x005U;
    stdFiltelem.sfid1 = 0x005U;
    stdFiltelem.sft = MCAN_STDFILT_RANGE;
    MCAN_addStdMsgIDFilter(MCANA_DRIVER_BASE, 20U, &stdFiltelem);

    // Configure reject-all filter for FIFO 0 (to discard unmatched messages)
    stdFiltelem.sfec = MCAN_STDFILTEC_REJECT;
    stdFiltelem.sfid1 = 0x000; // Don't care
    stdFiltelem.sfid2 = 0x000; // Don't care
    stdFiltelem.sft = MCAN_STDFILT_CLASSIC;
    MCAN_addStdMsgIDFilter(MCANA_DRIVER_BASE, 21U, &stdFiltelem);

    MCAN_setOpMode(MCANA_DRIVER_BASE, MCAN_OPERATION_MODE_NORMAL);

    while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(MCANA_DRIVER_BASE))
    {
    }
}

void MCANIntrConfig(void)
{
    Interrupt_initModule();
    Interrupt_initVectorTable();

    Interrupt_register(INT_MCANA_1, &MCANIntr1ISR);
    Interrupt_enable(INT_MCANA_1);

    Interrupt_enableGlobal();
}

// Transmission function implementations
void transmit(void)
{
    // Send first frame
    MCAN_TxBufElement TxMsg;
    memset(&TxMsg, 0, sizeof(MCAN_TxBufElement));

    // Copy frame 1 data
    memcpy(TxMsg.data, storedFrames[0].data, 64);

    // Set up CAN message properties
    TxMsg.id = ((uint32_t) (5 + address)) << 18U;
    TxMsg.brs = 0x1;
    TxMsg.dlc = 15;
    TxMsg.rtr = 0;
    TxMsg.xtd = 0;
    TxMsg.fdf = 0x1;
    TxMsg.esi = 0U;
    TxMsg.efc = 1U;
    TxMsg.mm = 0xAAU;

    // Write frame 1 to TX buffer
    MCAN_writeMsgRam(MCANA_DRIVER_BASE, MCAN_MEM_TYPE_BUF, 0, &TxMsg);
    MCAN_txBufAddReq(MCANA_DRIVER_BASE, 0U);

    // Wait for transmission to complete
    while (MCAN_getTxBufReqPend(MCANA_DRIVER_BASE))
    {
    }

    // Add delay between frames
    DEVICE_DELAY_US(500000);  // 30ms delay

    // Send second frame - reuse the same TxMsg structure
    memcpy(TxMsg.data, storedFrames[1].data, 64);

    // Write frame 2 to TX buffer
    MCAN_writeMsgRam(MCANA_DRIVER_BASE, MCAN_MEM_TYPE_BUF, 0, &TxMsg);
    MCAN_txBufAddReq(MCANA_DRIVER_BASE, 0U);

    // Wait for transmission to complete
    while (MCAN_getTxBufReqPend(MCANA_DRIVER_BASE))
    {
    }
}

void transmitGainFrames(void)
{
    // Transmit current gains frame
    MCAN_TxBufElement currentGainMsg;
    memset(&currentGainMsg, 0, sizeof(MCAN_TxBufElement));

    currentGainMsg.id = ((uint32_t)(5 + address)) << 18U;
    currentGainMsg.dlc = 15;
    currentGainMsg.fdf = 1U;
    currentGainMsg.brs = 1U;

    // Copy current gain data
    memcpy(currentGainMsg.data, storedFrames[0].data, 64);

    // Write and transmit current gains
    MCAN_writeMsgRam(MCANA_DRIVER_BASE, MCAN_MEM_TYPE_BUF, 0, &currentGainMsg);
    MCAN_txBufAddReq(MCANA_DRIVER_BASE, 0U);

    // Wait for transmission to complete
    while (MCAN_getTxBufReqPend(MCANA_DRIVER_BASE) & (1U << 0)) {}

    DEVICE_DELAY_US(500000);

    // Transmit kW gains frame
    MCAN_TxBufElement kwGainMsg;
    memset(&kwGainMsg, 0, sizeof(MCAN_TxBufElement));

    kwGainMsg.id = ((uint32_t)(5 + address)) << 18U;
    kwGainMsg.dlc = 15;
    kwGainMsg.fdf = 1U;
    kwGainMsg.brs = 1U;

    // Copy kW gain data
    memcpy(kwGainMsg.data, storedFrames[1].data, 64);

    // Write and transmit kW gains
    MCAN_writeMsgRam(MCANA_DRIVER_BASE, MCAN_MEM_TYPE_BUF, 1, &kwGainMsg);
    MCAN_txBufAddReq(MCANA_DRIVER_BASE, 1U);

    // Wait for transmission to complete
    while (MCAN_getTxBufReqPend(MCANA_DRIVER_BASE) & (1U << 1)) {}
}

void prepareFramesForTransmission(uint16_t *buffer, uint16_t bufferSize)
{
    // Prepare frame 1 - Current gains (18 values starting from readAddress + 2)
    CANFrame *frame1 = &storedFrames[0];
    frame1->order = 1;
    frame1->data[0] = 0x64 + address;  // First byte is 100 (0x64) + address (1)
    frame1->data[1] = 0x01;  // Second byte is frame number (1)

    // Copy current gains - 18 values starting from offset 2
    int i;
    for (i = 0; i < GAINS_PER_FRAME; i++)
    {
        uint16_t value = buffer[i];
        frame1->data[2 + i * 2] = (value >> 8) & 0xFF;     // High byte
        frame1->data[2 + i * 2 + 1] = value & 0xFF;        // Low byte
    }

    // Prepare frame 2 - KW gains (18 values following current gains)
    CANFrame *frame2 = &storedFrames[1];
    frame2->order = 2;
    frame2->data[0] = 0x64 + address;  // First byte is 100 (0x64) + address (2)
    frame2->data[1] = 0x02;  // Second byte is frame number (2)

    // Copy kW gains - 18 values starting after current gains
    for (i = 0; i < GAINS_PER_FRAME; i++)
    {
        uint16_t value = buffer[GAINS_PER_FRAME + i];
        frame2->data[2 + i * 2] = (value >> 8) & 0xFF;     // High byte
        frame2->data[2 + i * 2 + 1] = value & 0xFF;        // Low byte
    }
}

// Message processing functions
void processReceivedFrame(uint8_t frameNumber)
{
    // Start/restart timer for any new frame
    startFrameSequenceTimer();

    // Only process the frame if we haven't received it yet
    if (!(receivedFramesMask & (1 << (frameNumber - 1))))
    {
        // Set bit in mask for received frame
        receivedFramesMask |= (1 << (frameNumber - 1));
        frameCounter++;

        // Check if we have all frames
        if (receivedFramesMask == 0x7FFF)
        {
            allFramesReceived = true;
            bufferFull = true;  // Signal main loop to process the data
            CPUTimer_stopTimer(CPUTIMER1_BASE);
            retryCounter = 0;   // Reset retry counter on success
        }
    }
}

void processBUSpecificMessage(uint32_t bufferIndex)
{
    // Validate message identifier
    uint8_t firstByte = rxMsg[bufferIndex].data[0];
    uint8_t secondByte = rxMsg[bufferIndex].data[1];

    // Check if this is first frame (indicated by byte[1] = 1)
    if (secondByte == 0x01)
    {
        memcpy(firstBUFrameData, rxMsg[bufferIndex].data, 64);
        firstBUFrameReceived = true;
    }
    // Check if this is second frame (indicated by byte[1] = 2)
    else if (secondByte == 0x02)
    {
        memcpy(secondBUFrameData, rxMsg[bufferIndex].data, 64);
        secondBUFrameReceived = true;
    }

    // If both frames received, process the data
    if (firstBUFrameReceived && secondBUFrameReceived)
    {
        processBUFrames();
    }
}

void processBUFrames(void)
{
    // Reset the working data structure
    memset(&pduManager.readWriteData, 0, sizeof(CalibData));

    // Store board ID
    pduManager.readWriteData.BUBoardID = address;

    // Process first frame (Current gains)
    int i;
    for (i = 0; i < NumberOfBranches; i++)
    {
        // Extract current gains from first frame
        // Data starts from byte 2 (after command and frame number)
        uint16_t currentGain = (firstBUFrameData[2 + i * 2] << 8) |
                               firstBUFrameData[2 + i * 2 + 1];
        pduManager.readWriteData.currentGain[i] = currentGain;
    }

    // Process second frame (KW gains)
    for (i = 0; i < NumberOfBranches; i++)
    {
        // Extract kW gains from second frame
        // Data starts from byte 2 (after command and frame number)
        uint16_t kwGain = (secondBUFrameData[2 + i * 2] << 8) |
                          secondBUFrameData[2 + i * 2 + 1];
        pduManager.readWriteData.kwGain[i] = kwGain;
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

    // Prepare data for flash writing
    memcpy(Buffer, &pduManager.readWriteData, sizeof(CalibData));
    datasize = sizeof(CalibData) / sizeof(uint16_t);

    // Mark structures as out of sync
    pduManager.dataSync = false;

    // Write to flash
    findReadAndWriteAddress();
    eraseSectorOrFindEmptySector();
    writeToFlash();

    // If flash write was successful, sync the data and send acknowledgment
    if (!updateFailed)
    {
        syncReadWriteToWorking();
        sendBUFrameAcknowledgment(true, true);  // Both current and kW gains received
    }
    else
    {
        sendBUFrameAcknowledgment(false, false);  // Indicate failure
    }

    // Reset frame reception flags
    firstBUFrameReceived = false;
    secondBUFrameReceived = false;
}

void processGainAcknowledgment(void)
{
    gainAckReceived = true;
    CPUTimer_stopTimer(CPUTIMER2_BASE);
    retryCount = 0;
}

void processDiscoveryAck(void)
{
    if (discoveryInProgress)
    {
        discoveryAckReceived = true;
        discoveryInProgress = false;
        discoveryRetryCount = 0;
        CPUTimer_stopTimer(CPUTIMER1_BASE);

        // Reset all discovery flags
        resetDiscoveryFlags();
    }
}

// Error handling functions
void resetCANModule(void)
{
    // Disable interrupts temporarily
    DINT;

    // Stop timer
    CPUTimer_stopTimer(CPUTIMER1_BASE);

    // Clear all flags and counters
    receivedFramesMask = 0;
    allFramesReceived = false;
    sequenceStarted = false;
    frameCounter = 0;
    timerExpired = false;
    requestRetransmission = false;
    retryCounter = 0;

    // Reset MCAN module
    MCAN_setOpMode(MCANA_DRIVER_BASE, MCAN_OPERATION_MODE_SW_INIT);
    while (MCAN_OPERATION_MODE_SW_INIT != MCAN_getOpMode(MCANA_DRIVER_BASE))
    {
    }

    // Reconfigure MCAN
    MCANConfig();

    // Send reset notification message
    MCAN_TxBufElement TxMsg;
    memset(&TxMsg, 0, sizeof(MCAN_TxBufElement));
    TxMsg.id = ((uint32_t) (5 + address)) << 18U;
    TxMsg.brs = 0x1;
    TxMsg.dlc = 15;
    TxMsg.rtr = 0;
    TxMsg.xtd = 0;
    TxMsg.fdf = 0x1;
    TxMsg.esi = 0U;
    TxMsg.efc = 1U;
    TxMsg.mm = 0xAAU;

    // Message indicating CAN reset
    uint8_t resetMsg[64] = { 0x05, 0x01 }; // Command 0x05 for CAN reset notification
    memcpy(TxMsg.data, resetMsg, 64);

    MCAN_writeMsgRam(MCANA_DRIVER_BASE, MCAN_MEM_TYPE_BUF, 0, &TxMsg);
    MCAN_txBufAddReq(MCANA_DRIVER_BASE, 0U);

    // Wait for transmission
    while (MCAN_getTxBufReqPend(MCANA_DRIVER_BASE))
    {
    }

    // Re-enable interrupts
    EINT;
}

void handleMissingFrames(void)
{
    uint16_t missingFrames = 0;
    uint16_t expectedMask = 0x7FFF; // Bits 0-14 set (15 frames)

    // Find missing frames
    missingFrames = expectedMask & ~receivedFramesMask;

    // Increment retry counter
    retryCounter++;

    // Check if we've exceeded max retries
    if (retryCounter >= MAX_RETRIES)
    {
        maxRetriesExceeded();
        resetCANModule();
        return;
    }

    // Prepare and send retransmission request
    if (missingFrames != 0)
    {
        uint8_t retransmitData[64] = { 0x03 }; // Command for retransmission

        // Convert missing frames mask to retransmission request format
        int i;
        for (i = 0; i < 15; i++)
        {
            retransmitData[i + 1] = (missingFrames & (1 << i)) ? 0x00 : 0x01;
        }

        // Send retransmission request
        MCAN_TxBufElement txMsg;
        memset(&txMsg, 0, sizeof(MCAN_TxBufElement));

        txMsg.id = ((uint32_t) (5 + address)) << 18U;
        txMsg.brs = 0x1;
        txMsg.dlc = 15;
        txMsg.rtr = 0;
        txMsg.xtd = 0;
        txMsg.fdf = 0x1;
        txMsg.esi = 0U;
        txMsg.efc = 1U;
        txMsg.mm = 0xAAU;

        memcpy(txMsg.data, retransmitData, 64);

        MCAN_writeMsgRam(MCANA_DRIVER_BASE, MCAN_MEM_TYPE_BUF, 0, &txMsg);
        MCAN_txBufAddReq(MCANA_DRIVER_BASE, 0U);

        // Wait for transmission to complete
        while (MCAN_getTxBufReqPend(MCANA_DRIVER_BASE))
        {
        }

        requestRetransmission = false;
    }
    else
    {
        // All frames received, reset all tracking variables
        sequenceStarted = false;
        frameCounter = 0;
        timerExpired = false;
        requestRetransmission = false;
        allFramesReceived = false;
        retryCounter = 0;  // Reset retry counter on success
        CPUTimer_stopTimer(CPUTIMER1_BASE);
    }
}

void maxRetriesExceeded(void)
{
    MCAN_TxBufElement TxMsg;
    memset(&TxMsg, 0, sizeof(MCAN_TxBufElement));
    TxMsg.id = ((uint32_t) (5 + address)) << 18U;
    TxMsg.brs = 0x1;
    TxMsg.dlc = 15;
    TxMsg.rtr = 0;
    TxMsg.xtd = 0;
    TxMsg.fdf = 0x1;
    TxMsg.esi = 0U;
    TxMsg.efc = 1U;
    TxMsg.mm = 0xAAU;

    uint8_t datare[64] = { 0x05, 0x00 };
    int i;
    for (i = 0; i < 64; i++)
    {
        TxMsg.data[i] = datare[i];
    }

    MCAN_writeMsgRam(MCANA_DRIVER_BASE, MCAN_MEM_TYPE_BUF, 0, &TxMsg);

    //
    // Add transmission request for Tx buffer 0
    //
    MCAN_txBufAddReq(MCANA_DRIVER_BASE, 0U);

    //
    // Wait till the frame is successfully transmitted (and ACKnowledged)
    // "Tx Buffer Transmission Occurred" register is polled.
    //
    while (MCAN_getTxBufReqPend(MCANA_DRIVER_BASE))
    {
    }
}

// Protocol messages
void sendCalibrationDataFormatVersion(void)
{
    MCAN_TxBufElement TxMsg;
    memset(&TxMsg, 0, sizeof(MCAN_TxBufElement));
    TxMsg.id = ((uint32_t) (5 + address)) << 18U;
    TxMsg.brs = 0x1;
    TxMsg.dlc = 15;
    TxMsg.rtr = 0;
    TxMsg.xtd = 0;
    TxMsg.fdf = 0x1;
    TxMsg.esi = 0U;
    TxMsg.efc = 1U;
    TxMsg.mm = 0xAAU;

    uint8_t datare[] = { 0x01, address, majorVersionNumber, minorVersionNumber,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    int i;
    for (i = 0; i < 64; i++)
    {
        TxMsg.data[i] = datare[i];
    }

    MCAN_writeMsgRam(MCANA_DRIVER_BASE, MCAN_MEM_TYPE_BUF, 0, &TxMsg);

    //
    // Add transmission request for Tx buffer 0
    //
    MCAN_txBufAddReq(MCANA_DRIVER_BASE, 0U);

    //
    // Wait till the frame is successfully transmitted (and ACKnowledged)
    // "Tx Buffer Transmission Occurred" register is polled.
    //
    while (MCAN_getTxBufReqPend(MCANA_DRIVER_BASE))
    {
    }
}

void sendDiscoveryMessage(void)
{
    MCAN_TxBufElement TxMsg;
    memset(&TxMsg, 0, sizeof(MCAN_TxBufElement));

    // Configure message properties
    TxMsg.id = ((uint32_t)(5 + address)) << 18U;
    TxMsg.brs = 0x1;
    TxMsg.dlc = 15;
    TxMsg.rtr = 0;
    TxMsg.xtd = 0;
    TxMsg.fdf = 0x1;
    TxMsg.esi = 0U;
    TxMsg.efc = 1U;
    TxMsg.mm = 0xAAU;

    // Prepare discovery message data
    // First byte is DISCOVERY_START (0x0B), second byte is address
    uint8_t discoveryData[64] = {0};
    discoveryData[0] = DISCOVERY_START;
    discoveryData[1] = address;

    memcpy(TxMsg.data, discoveryData, 64);

    // Write message to the TX buffer
    MCAN_writeMsgRam(MCANA_DRIVER_BASE, MCAN_MEM_TYPE_BUF, 0, &TxMsg);
    MCAN_txBufAddReq(MCANA_DRIVER_BASE, 0U);

    // Wait for transmission to complete
    while (MCAN_getTxBufReqPend(MCANA_DRIVER_BASE))
    {
    }

    // Start discovery timeout timer
    startDiscoveryTimer();
}

void sendBUFrameAcknowledgment(bool currentGainOK, bool kwGainOK)
{
    MCAN_TxBufElement TxMsg;
    memset(&TxMsg, 0, sizeof(MCAN_TxBufElement));

    // Configure message properties
    TxMsg.id = ((uint32_t)(5 + address)) << 18U;
    TxMsg.brs = 0x1;
    TxMsg.dlc = 15;
    TxMsg.rtr = 0;
    TxMsg.xtd = 0;
    TxMsg.fdf = 0x1;
    TxMsg.esi = 0U;
    TxMsg.efc = 1U;
    TxMsg.mm = 0xAAU;

    // Prepare acknowledgment data
    uint8_t datare[64] = {0};
    datare[0] = 0x64 + address;  // 101 (0x65) + address
    datare[1] = currentGainOK ? 1 : 0;  // Current gain status
    datare[2] = kwGainOK ? 1 : 0;       // KW gain status

    // Copy acknowledgment data to message
    memcpy(TxMsg.data, datare, 64);

    // Write message to TX buffer
    MCAN_writeMsgRam(MCANA_DRIVER_BASE, MCAN_MEM_TYPE_BUF, 0, &TxMsg);
    MCAN_txBufAddReq(MCANA_DRIVER_BASE, 0U);

    // Wait for transmission to complete
    while (MCAN_getTxBufReqPend(MCANA_DRIVER_BASE))
    {
    }
}

int copyRxFIFO(void)
{
    int i, j;

    for (i = 0; i < NUM_FRAMES; i++)
    {
        uint16_t order = (rxFIFO0Msg[i].data[0] << 8) | rxFIFO0Msg[i].data[1];

        if (order == 0 || order > NUM_FRAMES)
        {
            // Invalid order, possibly end of data or error
            break;
        }

        uint32_t startPos = (order - 1) * (MAX_CAN_DATA_SIZE / 2);

                if (startPos + (MAX_CAN_DATA_SIZE / 2) > TOTAL_BUFFER_SIZE)
                {
                    // Buffer overflow, handle error
                    break;
                }

                // Copy data from this message to receivedBuffer
                for (j = 0; j < MAX_CAN_DATA_SIZE; j += 2)
                {
                    receivedBuffer[startPos++] = (rxFIFO0Msg[i].data[2 + j] << 8) |
                                                 rxFIFO0Msg[i].data[2 + j + 1];
                }
            }

            bufferFull = false;
            receivedFramesMask = 0;
            allFramesReceived = false;
            sequenceStarted = false;
            frameCounter = 0;
            timerExpired = false;
            requestRetransmission = false;
            retryCounter = 0; // Reset the retry counter

            findReadAndWriteAddress();
            eraseSectorOrFindEmptySector();

            // Process BU board data
            processBUBoardData();

            return 0;
        }

        // MCAN interrupt service routine
        __interrupt void MCANIntr1ISR(void)
        {
            uint32_t intrStatus;
            MCAN_RxFIFOStatus rxFIFO0Status;
            MCAN_RxNewDataStatus newData;

            intrStatus = MCAN_getIntrStatus(MCANA_DRIVER_BASE);

            // Clear the interrupt Status
            MCAN_clearIntrStatus(MCANA_DRIVER_BASE, intrStatus);

            // Clearing the interrupt lineNum
            MCAN_clearInterrupt(MCANA_DRIVER_BASE, 0x2);

            // Check to see if the interrupt is caused by a message being
            // received in FIFO 0
            if ((MCAN_IR_RF0N_MASK & intrStatus) == MCAN_IR_RF0N_MASK)
            {
                rxFIFO0Status.num = MCAN_RX_FIFO_NUM_0;
                // Get RX FIFO 0 status
                MCAN_getRxFIFOStatus(MCANA_DRIVER_BASE, &rxFIFO0Status);

                // Process all new messages in RX FIFO 0
                while (rxFIFO0Status.fillLvl)
                {
                    uint32_t getIndex = rxFIFO0Status.getIdx;

                    // Read message from RX FIFO 0
                    MCAN_readMsgRam(MCANA_DRIVER_BASE, MCAN_MEM_TYPE_FIFO, 0U,
                                    MCAN_RX_FIFO_NUM_0, &rxFIFO0Msg[getIndex]);

                    // Acknowledge reading the FIFO element
                    MCAN_writeRxFIFOAck(MCANA_DRIVER_BASE, MCAN_RX_FIFO_NUM_0,
                                        getIndex);

                    uint16_t order = (rxFIFO0Msg[getIndex].data[0] << 8)
                            | rxFIFO0Msg[getIndex].data[1];
                    processReceivedFrame(order);

                    // Process the received message
                    if (getIndex == 14)
                    {
                        bufferFull = true;
                    }

                    // Update RX FIFO 0 status
                    MCAN_getRxFIFOStatus(MCANA_DRIVER_BASE, &rxFIFO0Status);
                }
            }

            // Check if the interrupt is caused by a message received in dedicated RX buffers
            if ((MCAN_IR_DRX_MASK & intrStatus) == MCAN_IR_DRX_MASK)
            {
                // Read the NewData registers
                MCAN_getNewDataStatus(MCANA_DRIVER_BASE, &newData);

                // Check each buffer for new messages
                int i;
                for (i = 0; i < MCAN_RX_BUFF_NUM; i++)
                {
                    if ((newData.statusLow & (1UL << i)) != 0)
                    {
                        MCAN_readMsgRam(MCANA_DRIVER_BASE, MCAN_MEM_TYPE_BUF, i, 0,
                                        &rxMsg[i]);

                        int ID = (rxMsg[i].id >> 18) & 0x1F;
                        if (ID == SBoard)
                        {
                            memcpy(retransmissionData, rxMsg[i].data, 64);
                            CommandReceived = true;
                        }

                        // Handle gain acknowledgment
                        if (ID == 4)
                        {
                            // Check for discovery acknowledgment (0x0D)
                            if (rxMsg[i].data[0] == 0x0D && rxMsg[i].data[1] == (0x64 + address))
                            {
                                processDiscoveryAck();
                            }
                            // Check for BU frame acknowledgment
                            else if ((rxMsg[i].data[0] == (0x64 + address) && rxMsg[i].data[1] == 1) &&
                                     (rxMsg[i].data[0] == (0x64 + address) && rxMsg[i].data[2] == 1))
                            {
                                processGainAcknowledgment();
                            }
                        }

                        // Process messages specific to this BU board
                        if (ID == (5 + address))
                        {
                            processBUSpecificMessage(i);
                        }
                    }
                }

                // Clear the NewData registers
                MCAN_clearNewDataStatus(MCANA_DRIVER_BASE, &newData);
            }
            else
            {
                // Handle other interrupt sources
                error++;
            }

            // Acknowledge this interrupt
            Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
        }

        // Function to start transmission with retry mechanism
        void transmitWithRetry(void)
        {
            startGainTransmissionRetry();
        }

        // Clear FIFO buffer functions
        void ClearRxFIFOBuffer(void)
        {
            MCAN_RxFIFOStatus rxFIFO0Status;
            rxFIFO0Status.num = MCAN_RX_FIFO_NUM_0;

            // Get RX FIFO 0 status
            MCAN_getRxFIFOStatus(MCANA_DRIVER_BASE, &rxFIFO0Status);

            // Clear all messages in RX FIFO 0
            while (rxFIFO0Status.fillLvl)
            {
                uint32_t getIndex = rxFIFO0Status.getIdx;

                // Acknowledge reading the FIFO element
                MCAN_writeRxFIFOAck(MCANA_DRIVER_BASE, MCAN_RX_FIFO_NUM_0, getIndex);

                // Update RX FIFO 0 status
                MCAN_getRxFIFOStatus(MCANA_DRIVER_BASE, &rxFIFO0Status);
            }
        }

        void ClearRxDedicatedBuffer(void)
        {
            MCAN_RxNewDataStatus newData;

            // Get the NewData status
            MCAN_getNewDataStatus(MCANA_DRIVER_BASE, &newData);

            // Clear the NewData registers
            MCAN_clearNewDataStatus(MCANA_DRIVER_BASE, &newData);
        }
