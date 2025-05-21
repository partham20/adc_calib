#ifndef CONFIG_H
#define CONFIG_H

// Include standard libraries
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Include device-specific headers
#include "driverlib.h"
#include "device.h"
#include "inc/stw_types.h"
#include "inc/stw_dataTypes.h"

// Flash header files
#include "FlashTech_F28P55x_C28x.h"
#include "flash_programming_f28p55x.h"

//
//  CAN configuration constants
//
#define NUM_OF_MSG                      (32U)
#define MCAN_STD_ID_FILTER_NUM          (38U)
#define MCAN_EXT_ID_FILTER_NUM          (0U)
#define MCAN_FIFO_0_NUM                 (15U)
#define MCAN_FIFO_0_ELEM_SIZE           (MCAN_ELEM_SIZE_64BYTES)
#define MCAN_FIFO_1_NUM                 (0U)
#define MCAN_FIFO_1_ELEM_SIZE           (MCAN_ELEM_SIZE_64BYTES)
#define MCAN_RX_BUFF_NUM                (18U)
#define MCAN_RX_BUFF_ELEM_SIZE          (MCAN_ELEM_SIZE_64BYTES)
#define MCAN_TX_BUFF_SIZE               (32U)
#define MCAN_TX_FQ_SIZE                 (0U)
#define MCAN_TX_BUFF_ELEM_SIZE          (MCAN_ELEM_SIZE_64BYTES)
#define MCAN_TX_EVENT_SIZE              (32U)

//
//  Defining Starting Addresses for Message RAM Sections
//
#define MCAN_STD_ID_FILT_START_ADDR     (0x0U)
#define MCAN_EXT_ID_FILT_START_ADDR     (MCAN_STD_ID_FILT_START_ADDR + ((MCAN_STD_ID_FILTER_NUM * MCANSS_STD_ID_FILTER_SIZE_WORDS * 4U)))
#define MCAN_FIFO_0_START_ADDR          (MCAN_EXT_ID_FILT_START_ADDR + ((MCAN_EXT_ID_FILTER_NUM * MCANSS_EXT_ID_FILTER_SIZE_WORDS * 4U)))
#define MCAN_FIFO_1_START_ADDR          (MCAN_FIFO_0_START_ADDR + (MCAN_getMsgObjSize(MCAN_FIFO_0_ELEM_SIZE) * 4U * MCAN_FIFO_0_NUM))
#define MCAN_RX_BUFF_START_ADDR         (MCAN_FIFO_1_START_ADDR + (MCAN_getMsgObjSize(MCAN_FIFO_1_ELEM_SIZE) * 4U * MCAN_FIFO_1_NUM))
#define MCAN_TX_BUFF_START_ADDR         (MCAN_RX_BUFF_START_ADDR + (MCAN_getMsgObjSize(MCAN_RX_BUFF_ELEM_SIZE) * 4U * MCAN_RX_BUFF_NUM))
#define MCAN_TX_EVENT_START_ADDR        (MCAN_TX_BUFF_START_ADDR + (MCAN_getMsgObjSize(MCAN_TX_BUFF_ELEM_SIZE) * 4U * (MCAN_TX_BUFF_SIZE + MCAN_TX_FQ_SIZE)))

//
// Flash and Memory constants
//
// Calibration Data format Version
#define majorVersionNumber              1
#define minorVersionNumber              0

#define SECTOR1                         0x100000U  // Sector 1 bank 4 start address
#define calibrationdDataSize            36
#define TOTAL_BUFFER_SIZE               0x400

// Flash Sector Write Enable Code
#define SEC0TO31                        0x00000000U
#define SEC32To127                      0xFFFFFFFFU

#define WORDS_IN_FLASH_BUFFER           0x400
#define CRC16_POLYNOMIAL                0x8005  // Define the CRC16 polynomial
#define FLASH_SIZE                      FlashBank4EndAddress

// Flash memory addresses
#define BANK4_START                     0x100000U
#define SECTOR_SIZE                     0x400U
#define WRAP_LIMIT                      0xFFFE

//
// Board-specific constants
//
#define BASE_CAN_ADDRESS                100
#define TOTAL_BRANCHES                  18
#define NUM_GPIO_PINS                   4
#define numberOfCTs                     216  // Number of current and kW gain - 216 needed
#define NUM_FRAMES                      15
#define NumberOfBranches                18U
#define SBoard                          4U

//
// Protocol constants
//
#define DISCOVERY_START                 0x0BU
#define DISCOVERY_END                   0x0CU

//
// Retry and timeout constants
//
#define MAX_RETRIES                     4
#define MAX_GAIN_RETRIES                3
#define TIMER2_PERIOD_US                2000000  // 2 seconds in microseconds
#define MAX_DISCOVERY_RETRIES           3
#define DISCOVERY_TIMEOUT_MS            2000

//
// CAN frame and data sizes
//
#define FLASH_READ_SIZE                 bytes_read
#define MAX_CAN_DATA_SIZE               62
#define GAINS_PER_FRAME                 18  // Number of gains to send in each frame

//
// BU Board related constants
//
#define CTS_PER_BOARD                   18
#define NUM_BU_BOARDS                   12

// Debug and error handling
#ifdef DEBUG
#define DEBUG_PRINT(x) printf x
#else
#define DEBUG_PRINT(x)
#endif

#endif // CONFIG_H
