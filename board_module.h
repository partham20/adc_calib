#ifndef BOARD_MODULE_H
#define BOARD_MODULE_H

#include "config.h"
#include "data_structures.h"

// GPIO functions
void initDIPSwitchGPIO(void);
void readCANAddress(void);
uint32_t readDIPSwitch(uint32_t gpioNumber);

// BU Board related functions
void sendGainsToBuBoard(int buid);
void readGainsFromBuBoard(int buid);
void processBUBoardData(void);
void checkAddressChange(void);

// State management
void resetDiscoveryFlags(void);
void resetFrameTrackingVariables(void);
void handleDiscoveryProcess(void);
void handleGainRetransmission(void);


// Device initialization
void startupfunc(void);

#endif // BOARD_MODULE_H
