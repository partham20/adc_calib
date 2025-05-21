#ifndef UTILITIES_H
#define UTILITIES_H

#include "config.h"
#include "data_structures.h"

// Data structure management
void initializePDUManager(void);
void syncWorkingToReadWrite(void);
void syncReadWriteToWorking(void);
bool updateWorkingData(const CalibData* newData);
void RxBufferToStruct(void);

// Acknowledgment functions
void flashWriteEnabledAck(void);
void flashWriteDisabledAck(void);
void calibDataUpdateSuccessAck(void);
void calibDataUpdateFailAck(void);
void erasedBank4Ack(void);
void writtenDefaultCalibrationValuesAck(void);

// CRC calculation
uint16_t crc16(uint16_t *data, size_t length_in_16bit_units);

// Validation
bool ValidDataReceived(void);

#endif // UTILITIES_H
