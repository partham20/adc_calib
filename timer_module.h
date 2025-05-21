#ifndef TIMER_MODULE_H
#define TIMER_MODULE_H

#include "config.h"

// Timer configuration
void initCPUTimers(void);
void configCPUTimer(uint32_t cpuTimer, float freq, float period);
void configureTimer2(void);
void initFrameTracking(void);

// Timer control
void startFrameSequenceTimer(void);
void startDiscoveryTimer(void);
void startGainTransmissionRetry(void);

// Timer interrupts
__interrupt void frameTrackingTimerISR(void);
__interrupt void cpuTimer2ISR(void);

// Timer-related flags
extern volatile bool timer2Expired;
extern volatile bool gainAckReceived;
extern volatile uint8_t retryCount;
extern int cpuTimer1IntCount;

#endif // TIMER_MODULE_H
