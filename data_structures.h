#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include "config.h"

// CAN Frame structure
typedef struct {
    uint8_t data[64];  // 64 bytes for CAN FD frame
    uint16_t order;    // Order of the frame (1-15)
} CANFrame;

// Three-phase measurements structure
typedef struct {
    uint16_t R;
    uint16_t Y;
    uint16_t B;
} ThreePhaseValues;

// Primary side measurements
typedef struct {
    ThreePhaseValues voltage;      // Primary voltages
    ThreePhaseValues current;      // Primary currents
    uint16_t neutralCurrent;       // Primary neutral current
    uint16_t totalKW;              // Primary total power
} PrimaryMeasurements;

// Secondary side measurements
typedef struct {
    ThreePhaseValues voltage;      // Secondary voltages
    ThreePhaseValues current;      // Secondary currents
    uint16_t neutralCurrent;       // Secondary neutral current
    ThreePhaseValues kw;           // Secondary power per phase
} SecondaryMeasurements;

// Calibration gains structure
typedef struct {
    uint16_t ctGain[numberOfCTs];  // CT current calibration gains
    uint16_t kwGain[numberOfCTs];  // Power calibration gains
} CalibrationGains;

// Main PDU Data structure
typedef struct {
    CalibrationGains gains;        // System calibration gains
    PrimaryMeasurements primary;   // All primary measurements
    SecondaryMeasurements secondary; // All secondary measurements
    uint16_t groundCurrent;        // Ground current measurement
} PDUData;

// Calibration data structure
typedef struct {
    uint16_t sectorIndex;
    uint16_t BUBoardID;
    uint16_t currentGain[TOTAL_BRANCHES];
    uint16_t kwGain[TOTAL_BRANCHES];
} CalibData;

// PDU data manager structure
typedef struct {
    CalibData readWriteData;       // Structure for flash read/write operations
    CalibData workingData;         // Structure for active PDU operations
    bool dataSync;                 // Flag to track if structures are in sync
} PDUDataManager;

// Global variable declarations (extern)
extern PDUDataManager pduManager;
extern CANFrame storedFrames[NUM_FRAMES];
extern uint16_t buffer[TOTAL_BUFFER_SIZE];
extern uint16_t receivedBuffer[TOTAL_BUFFER_SIZE];
extern uint16_t Buffer[WORDS_IN_FLASH_BUFFER];
extern uint32_t address;

// Flash-related globals
extern uint32_t addressToWrite;
extern uint16_t newestData;
extern uint16_t oldestData;
extern uint32_t writeAddress;
extern uint32_t readAddress;
extern bool emptySectorFound;
extern size_t datasize;

// CAN-related globals
extern MCAN_RxBufElement rxMsg[MCAN_RX_BUFF_NUM], rxFIFO0Msg[MCAN_FIFO_0_NUM];
extern MCAN_TxBufElement txMsg[NUM_OF_MSG];
extern uint8_t retransmissionData[64];

// Flags and state variables
extern bool flashWriteEnabled;
extern bool CommandReceived;
extern bool updateFailed;
extern bool bufferFull;
extern bool discoveryInProgress;
extern bool discoveryAckReceived;
extern bool timerExpired;
extern bool allFramesReceived;
extern bool requestRetransmission;
extern uint16_t receivedFramesMask;
extern uint8_t retryCounter;
extern uint8_t discoveryRetryCount;

#endif // DATA_STRUCTURES_H
