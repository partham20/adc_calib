#ifndef CAN_MODULE_H
#define CAN_MODULE_H

#include "config.h"
#include "data_structures.h"

// CAN Configuration functions
void MCANConfig(void);
void MCANIntrConfig(void);

// Transmission functions
void transmit(void);
void transmitWithRetry(void);
void transmitGainFrames(void);
void prepareFramesForTransmission(uint16_t* buffer, uint16_t bufferSize);

// Message processing functions
void processReceivedFrame(uint8_t frameNumber);
void processBUSpecificMessage(uint32_t bufferIndex);
void processBUFrames(void);
void processGainAcknowledgment(void);
void processDiscoveryAck(void);

// Buffer management
void ClearRxFIFOBuffer(void);
void ClearRxDedicatedBuffer(void);
int copyRxFIFO(void);

// Error handling
void resetCANModule(void);
void handleMissingFrames(void);
void maxRetriesExceeded(void);

// Protocol specific messages
void sendCalibrationDataFormatVersion(void);
void sendDiscoveryMessage(void);
void sendBUFrameAcknowledgment(bool currentGainOK, bool kwGainOK);

// Interrupt handlers
__interrupt void MCANIntr1ISR(void);

extern bool firstBUFrameReceived ;
extern bool secondBUFrameReceived ;

extern int error;
extern uint16_t firstBUFrameData[64];
extern uint16_t secondBUFrameData[64];
extern bool  sequenceStarted ;

#endif // CAN_MODULE_H
