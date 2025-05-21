#include "utilities.h"
#include "data_structures.h"
#include "flash_module.h"

// Calculate CRC16 for data validation
uint16_t crc16(uint16_t *data, size_t length_in_16bit_units)
{
   uint16_t crc = 0;
   uint16_t i, j;

   for (i = 0; i < length_in_16bit_units; i++)
   {
       crc ^= data[i];
       for (j = 0; j < 16; j++)
       {
           if (crc & 0x0001)
               crc = (crc >> 1) ^ CRC16_POLYNOMIAL;
           else
               crc >>= 1;
       }
   }

   return crc;
}

// Validate received data
bool ValidDataReceived(void)
{
   // This could perform CRC checks or other data validation
   // For now, just return true as this function was unused in the original code
   return true;
}

// Initialize PDU Manager
void initializePDUManager(void)
{
   // Initialize both structures to zero
   memset(&pduManager.readWriteData, 0, sizeof(CalibData));
   memset(&pduManager.workingData, 0, sizeof(CalibData));
   pduManager.dataSync = true;
}

// Copy working data to read/write structure
void syncWorkingToReadWrite(void)
{
   // Copy working data to read/write structure
   memcpy(&pduManager.readWriteData, &pduManager.workingData,
          sizeof(CalibData));
   pduManager.dataSync = true;
}

// Copy read/write data to working structure
void syncReadWriteToWorking(void)
{
   // Copy read/write data to working structure
   memcpy(&pduManager.workingData, &pduManager.readWriteData,
          sizeof(CalibData));
   pduManager.dataSync = true;
}

// Update working data with new values
bool updateWorkingData(const CalibData *newData)
{
   if (newData == NULL)
   {
       return false;
   }

   // Update working structure
   memcpy(&pduManager.workingData, newData, sizeof(CalibData));
   pduManager.dataSync = false;
   return true;
}

// Convert received buffer to structure
void RxBufferToStruct(void)
{
   if (newestData >= (WRAP_LIMIT))
   {
       eraseBank4();
       newestData = 0;
       oldestData = 0xFFFF;
       writeAddress = 0xFFFFFFFF;
       addressToWrite = BANK4_START;
   }

   // Gains
   int i;
   for (i = 0; i < NumberOfBranches; i++)
   {
       pduManager.readWriteData.currentGain[i] = receivedBuffer[i];
       pduManager.readWriteData.kwGain[i] =
               receivedBuffer[NumberOfBranches + i];
   }

   pduManager.readWriteData.sectorIndex = ++newestData;
   pduManager.readWriteData.BUBoardID = address;

   // Copy to Buffer for flash writing
   memcpy(Buffer, &pduManager.readWriteData, sizeof(CalibData));
   datasize = sizeof(CalibData) / sizeof(uint16_t);

   // Mark structures as out of sync
   pduManager.dataSync = false;
}

// Acknowledgment functions
void flashWriteEnabledAck(void)
{
   MCAN_TxBufElement TxMsg;
   memset(&TxMsg, 0, sizeof(MCAN_TxBufElement));
   TxMsg.id = ((uint32_t) (0x4)) << 18U;
   TxMsg.brs = 0x1;
   TxMsg.dlc = 15;
   TxMsg.rtr = 0;
   TxMsg.xtd = 0;
   TxMsg.fdf = 0x1;
   TxMsg.esi = 0U;
   TxMsg.efc = 1U;
   TxMsg.mm = 0xAAU;

   uint8_t datare[64] = { 0x02, 0x01 };
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

void flashWriteDisabledAck(void)
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
   // 00 - Disabled
   uint8_t datare[64] = { 0x02 };
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

void calibDataUpdateSuccessAck(void)
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

   uint8_t datare[] = { 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00 };

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

void calibDataUpdateFailAck(void)
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

   uint8_t datare[64] = { 0x04, 0x00 };

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

void erasedBank4Ack(void)
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

   uint8_t datare[64] = { 0x09, 0x01 };
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

void writtenDefaultCalibrationValuesAck(void)
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

   uint8_t datare[64] = { 0xA, 0x1 };
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

// Utility functions for string handling

// Convert a string to uppercase
void strToUpper(char* str)
{
    if (str == NULL)
        return;

    while (*str)
    {
        if (*str >= 'a' && *str <= 'z')
            *str = *str - 'a' + 'A';
        str++;
    }
}

// Convert a string to lowercase
void strToLower(char* str)
{
    if (str == NULL)
        return;

    while (*str)
    {
        if (*str >= 'A' && *str <= 'Z')
            *str = *str - 'A' + 'a';
        str++;
    }
}

// Convert integer to string
void intToStr(int value, char* buffer, int bufferSize)
{
    if (buffer == NULL || bufferSize <= 0)
        return;

    // Handle negative numbers
    bool negative = false;
    if (value < 0)
    {
        negative = true;
        value = -value;
    }

    // Find end of buffer for null terminator
    int pos = bufferSize - 1;
    buffer[pos] = '\0';
    pos--;

    // Convert digits from right to left
    do
    {
        if (pos < 0)
            break;

        buffer[pos] = '0' + (value % 10);
        value /= 10;
        pos--;
    } while (value > 0);

    // Add negative sign if needed
    if (negative && pos >= 0)
    {
        buffer[pos] = '-';
        pos--;
    }

    // Shift characters to start of buffer
    if (pos < bufferSize - 2)
    {
        int shift = pos + 1;
        int i;
        for (i = 0; i < bufferSize - shift; i++)
        {
            buffer[i] = buffer[i + shift];
        }
    }
}

// Convert hexadecimal integer to string
void hexToStr(uint32_t value, char* buffer, int bufferSize)
{
    if (buffer == NULL || bufferSize <= 0)
        return;

    // Find end of buffer for null terminator
    int pos = bufferSize - 1;
    buffer[pos] = '\0';
    pos--;

    // Convert digits from right to left
    do
    {
        if (pos < 0)
            break;

        int digit = value & 0xF;
        buffer[pos] = (digit < 10) ? ('0' + digit) : ('A' + digit - 10);
        value >>= 4;
        pos--;
    } while (value > 0);

    // Shift characters to start of buffer
    if (pos < bufferSize - 2)
    {
        int shift = pos + 1;
        int i;
        for (i = 0; i < bufferSize - shift; i++)
        {
            buffer[i] = buffer[i + shift];
        }
    }
}

// Compare two strings (case sensitive)
bool strEquals(const char* str1, const char* str2)
{
    if (str1 == NULL || str2 == NULL)
        return false;

    while (*str1 && *str2)
    {
        if (*str1 != *str2)
            return false;

        str1++;
        str2++;
    }

    return (*str1 == *str2); // Both should be at null terminator
}

// Compare two strings (case insensitive)
bool strEqualsIgnoreCase(const char* str1, const char* str2)
{
    if (str1 == NULL || str2 == NULL)
        return false;

    while (*str1 && *str2)
    {
        char c1 = (*str1 >= 'A' && *str1 <= 'Z') ? (*str1 - 'A' + 'a') : *str1;
        char c2 = (*str2 >= 'A' && *str2 <= 'Z') ? (*str2 - 'A' + 'a') : *str2;

        if (c1 != c2)
            return false;

        str1++;
        str2++;
    }

    return (*str1 == *str2); // Both should be at null terminator
}

// Utility functions for array manipulation

// Find minimum value in integer array
int findMin(const int* array, int size)
{
    if (array == NULL || size <= 0)
        return 0;

    int minValue = array[0];
    int i;

    for (i = 1; i < size; i++)
    {
        if (array[i] < minValue)
            minValue = array[i];
    }

    return minValue;
}

// Find maximum value in integer array
int findMax(const int* array, int size)
{
    if (array == NULL || size <= 0)
        return 0;

    int maxValue = array[0];
    int i;

    for (i = 1; i < size; i++)
    {
        if (array[i] > maxValue)
            maxValue = array[i];
    }

    return maxValue;
}

// Calculate average of integer array
float calculateAverage(const int* array, int size)
{
    if (array == NULL || size <= 0)
        return 0.0f;

    int sum = 0;
    int i;

    for (i = 0; i < size; i++)
    {
        sum += array[i];
    }

    return (float)sum / size;
}

// Utility functions for bit manipulation

// Set a specific bit in a value
uint32_t setBit(uint32_t value, uint8_t bitPosition)
{
    return value | (1UL << bitPosition);
}

// Clear a specific bit in a value
uint32_t clearBit(uint32_t value, uint8_t bitPosition)
{
    return value & ~(1UL << bitPosition);
}

// Toggle a specific bit in a value
uint32_t toggleBit(uint32_t value, uint8_t bitPosition)
{
    return value ^ (1UL << bitPosition);
}

// Check if a specific bit is set
bool isBitSet(uint32_t value, uint8_t bitPosition)
{
    return (value & (1UL << bitPosition)) != 0;
}

// Count number of bits set in a value
uint8_t countSetBits(uint32_t value)
{
    uint8_t count = 0;
    while (value)
    {
        count += value & 1;
        value >>= 1;
    }
    return count;
}

// Utility functions for debugging

// Calculate execution time between two points (assumes timer is available)
uint32_t calculateExecutionTime(uint32_t startTime, uint32_t endTime)
{
    return (endTime >= startTime) ? (endTime - startTime) :
           ((0xFFFFFFFF - startTime) + endTime + 1);
}

// Memory management utilities

// Fill memory with a specific byte value
void memoryFill(void* dest, uint8_t value, uint32_t size)
{
    if (dest == NULL)
        return;

    uint8_t* destPtr = (uint8_t*)dest;
    uint32_t i;

    for (i = 0; i < size; i++)
    {
        destPtr[i] = value;
    }
}

// Compare two memory blocks
bool memoryCompare(const void* block1, const void* block2, uint32_t size)
{
    if (block1 == NULL || block2 == NULL)
        return false;

    const uint8_t* ptr1 = (const uint8_t*)block1;
    const uint8_t* ptr2 = (const uint8_t*)block2;
    uint32_t i;

    for (i = 0; i < size; i++)
    {
        if (ptr1[i] != ptr2[i])
            return false;
    }

    return true;
}
