//###############################################################################
//
// FILE:   bu_adc.h
//
// TITLE:  Branch Unit ADC Module Header File
//
// PURPOSE: This module handles ADC sampling, RMS calculation, and current
//          measurement for 18 channels with synchronization support
//
//###############################################################################

#ifndef BU_ADC_H
#define BU_ADC_H

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include <math.h>
#include <stdbool.h>
#include "dww_parameter.h"

//
// Defines
//
#define NUM_CHANNELS             18
#define SAMPLE_COUNT_THRESHOLD   100
#define MAX_CT_LENGTH           NUM_CHANNELS

// GPIO Configuration
#define SYNC_GPIO_PIN           28U
#define SYNC_GPIO_CONFIG        GPIO_28_GPIO28
#define LED_GPIO_PIN            29U
#define LED_GPIO_CONFIG         GPIO_29_GPIO29

// Timing Constants
#define DEMAND_1SEC_COUNT       270
#define DEMAND_1HR_COUNT        60    // For testing (normally 3600)
#define DEMAND_24HR_COUNT       5     // For testing (normally 24)

// Default Calibration
#define DEFAULT_CALIB_FACTOR    5265

//
// Global Variable Declarations
//
extern uint32_t myADC0Results[NUM_CHANNELS];
extern unsigned long dwSumPhaseVolt[NUM_CHANNELS];
extern unsigned int sampleCount;
extern float calib;
extern volatile bool syncSignalReceived;

extern DWW_CHANNEL_PARAMETER Branch;
extern unsigned int demand_chk_1sec;
extern unsigned int demand_chk_1hr;
extern unsigned int demand_chk_24hr;
extern unsigned int wDemandSumCntr[4];
extern unsigned long dw_CurrDemandSum[NUM_CHANNELS];
extern dww_structflag *sFlag;
extern int a;
//
// Function Prototypes
//

// Initialization Functions
void BU_ADC_init(void);
void initEPWM(void);
void configureGPIO28(void);
void configureLED(void);

// ADC and Measurement Functions
void BU_ADC_processLoop(void);
uint32_t readGPIO28(void);

// Calculation Functions
void dww_Loadper_Calc(void);
void dww_max_min_Calc(void);
void dww_timer(void);
void dww_demand_Calc(void);

// Interrupt Service Routines
__interrupt void adcA1ISR(void);
__interrupt void gpioISR(void);

#endif // BU_ADC_H
