/*
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef BOARD_H
#define BOARD_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//
// Included Files
//

#include "driverlib.h"
#include "device.h"

//*****************************************************************************
//
// PinMux Configurations
//
//*****************************************************************************

//
// ANALOG -> myANALOGPinMux0 Pinmux
//

//*****************************************************************************
//
// ADC Configurations
//
//*****************************************************************************
#define myADC0_BASE ADCA_BASE
#define myADC0_RESULT_BASE ADCARESULT_BASE
#define myADC0_SOC0 ADC_SOC_NUMBER0
#define myADC0_FORCE_SOC0 ADC_FORCE_SOC0
#define myADC0_SAMPLE_WINDOW_SOC0 66.66666666666667
#define myADC0_TRIGGER_SOURCE_SOC0 ADC_TRIGGER_EPWM1_SOCA
#define myADC0_CHANNEL_SOC0 ADC_CH_ADCIN6
#define myADC0_SOC1 ADC_SOC_NUMBER1
#define myADC0_FORCE_SOC1 ADC_FORCE_SOC1
#define myADC0_SAMPLE_WINDOW_SOC1 66.66666666666667
#define myADC0_TRIGGER_SOURCE_SOC1 ADC_TRIGGER_EPWM1_SOCA
#define myADC0_CHANNEL_SOC1 ADC_CH_ADCIN3
#define myADC0_SOC2 ADC_SOC_NUMBER2
#define myADC0_FORCE_SOC2 ADC_FORCE_SOC2
#define myADC0_SAMPLE_WINDOW_SOC2 66.66666666666667
#define myADC0_TRIGGER_SOURCE_SOC2 ADC_TRIGGER_EPWM1_SOCA
#define myADC0_CHANNEL_SOC2 ADC_CH_ADCIN2
#define myADC0_SOC3 ADC_SOC_NUMBER3
#define myADC0_FORCE_SOC3 ADC_FORCE_SOC3
#define myADC0_SAMPLE_WINDOW_SOC3 66.66666666666667
#define myADC0_TRIGGER_SOURCE_SOC3 ADC_TRIGGER_EPWM1_SOCA
#define myADC0_CHANNEL_SOC3 ADC_CH_ADCIN15
#define myADC0_SOC4 ADC_SOC_NUMBER4
#define myADC0_FORCE_SOC4 ADC_FORCE_SOC4
#define myADC0_SAMPLE_WINDOW_SOC4 66.66666666666667
#define myADC0_TRIGGER_SOURCE_SOC4 ADC_TRIGGER_EPWM1_SOCA
#define myADC0_CHANNEL_SOC4 ADC_CH_ADCIN14
#define myADC0_SOC5 ADC_SOC_NUMBER5
#define myADC0_FORCE_SOC5 ADC_FORCE_SOC5
#define myADC0_SAMPLE_WINDOW_SOC5 66.66666666666667
#define myADC0_TRIGGER_SOURCE_SOC5 ADC_TRIGGER_EPWM1_SOCA
#define myADC0_CHANNEL_SOC5 ADC_CH_ADCIN11
#define myADC0_SOC6 ADC_SOC_NUMBER6
#define myADC0_FORCE_SOC6 ADC_FORCE_SOC6
#define myADC0_SAMPLE_WINDOW_SOC6 66.66666666666667
#define myADC0_TRIGGER_SOURCE_SOC6 ADC_TRIGGER_EPWM1_SOCA
#define myADC0_CHANNEL_SOC6 ADC_CH_ADCIN5
#define myADC0_SOC7 ADC_SOC_NUMBER7
#define myADC0_FORCE_SOC7 ADC_FORCE_SOC7
#define myADC0_SAMPLE_WINDOW_SOC7 66.66666666666667
#define myADC0_TRIGGER_SOURCE_SOC7 ADC_TRIGGER_EPWM1_SOCA
#define myADC0_CHANNEL_SOC7 ADC_CH_ADCIN1
#define myADC0_SOC8 ADC_SOC_NUMBER8
#define myADC0_FORCE_SOC8 ADC_FORCE_SOC8
#define myADC0_SAMPLE_WINDOW_SOC8 66.66666666666667
#define myADC0_TRIGGER_SOURCE_SOC8 ADC_TRIGGER_EPWM1_SOCA
#define myADC0_CHANNEL_SOC8 ADC_CH_ADCIN12
#define myADC0_SOC9 ADC_SOC_NUMBER9
#define myADC0_FORCE_SOC9 ADC_FORCE_SOC9
#define myADC0_SAMPLE_WINDOW_SOC9 66.66666666666667
#define myADC0_TRIGGER_SOURCE_SOC9 ADC_TRIGGER_EPWM1_SOCA
#define myADC0_CHANNEL_SOC9 ADC_CH_ADCIN7
#define myADC0_SOC10 ADC_SOC_NUMBER10
#define myADC0_FORCE_SOC10 ADC_FORCE_SOC10
#define myADC0_SAMPLE_WINDOW_SOC10 66.66666666666667
#define myADC0_TRIGGER_SOURCE_SOC10 ADC_TRIGGER_EPWM1_SOCA
#define myADC0_CHANNEL_SOC10 ADC_CH_ADCIN8
#define myADC0_SOC11 ADC_SOC_NUMBER11
#define myADC0_FORCE_SOC11 ADC_FORCE_SOC11
#define myADC0_SAMPLE_WINDOW_SOC11 66.66666666666667
#define myADC0_TRIGGER_SOURCE_SOC11 ADC_TRIGGER_EPWM1_SOCA
#define myADC0_CHANNEL_SOC11 ADC_CH_ADCIN4
#define myADC0_SOC12 ADC_SOC_NUMBER12
#define myADC0_FORCE_SOC12 ADC_FORCE_SOC12
#define myADC0_SAMPLE_WINDOW_SOC12 66.66666666666667
#define myADC0_TRIGGER_SOURCE_SOC12 ADC_TRIGGER_EPWM1_SOCA
#define myADC0_CHANNEL_SOC12 ADC_CH_ADCIN9
#define myADC0_SOC13 ADC_SOC_NUMBER13
#define myADC0_FORCE_SOC13 ADC_FORCE_SOC13
#define myADC0_SAMPLE_WINDOW_SOC13 66.66666666666667
#define myADC0_TRIGGER_SOURCE_SOC13 ADC_TRIGGER_EPWM1_SOCA
#define myADC0_CHANNEL_SOC13 ADC_CH_ADCIN25
void myADC0_init();

#define myADC1_BASE ADCE_BASE
#define myADC1_RESULT_BASE ADCERESULT_BASE
#define myADC1_SOC0 ADC_SOC_NUMBER0
#define myADC1_FORCE_SOC0 ADC_FORCE_SOC0
#define myADC1_SAMPLE_WINDOW_SOC0 66.66666666666667
#define myADC1_TRIGGER_SOURCE_SOC0 ADC_TRIGGER_EPWM1_SOCA
#define myADC1_CHANNEL_SOC0 ADC_CH_ADCIN1
#define myADC1_SOC1 ADC_SOC_NUMBER1
#define myADC1_FORCE_SOC1 ADC_FORCE_SOC1
#define myADC1_SAMPLE_WINDOW_SOC1 66.66666666666667
#define myADC1_TRIGGER_SOURCE_SOC1 ADC_TRIGGER_EPWM1_SOCA
#define myADC1_CHANNEL_SOC1 ADC_CH_ADCIN2
#define myADC1_SOC2 ADC_SOC_NUMBER2
#define myADC1_FORCE_SOC2 ADC_FORCE_SOC2
#define myADC1_SAMPLE_WINDOW_SOC2 66.66666666666667
#define myADC1_TRIGGER_SOURCE_SOC2 ADC_TRIGGER_EPWM1_SOCA
#define myADC1_CHANNEL_SOC2 ADC_CH_ADCIN12
#define myADC1_SOC3 ADC_SOC_NUMBER3
#define myADC1_FORCE_SOC3 ADC_FORCE_SOC3
#define myADC1_SAMPLE_WINDOW_SOC3 66.66666666666667
#define myADC1_TRIGGER_SOURCE_SOC3 ADC_TRIGGER_EPWM1_SOCA
#define myADC1_CHANNEL_SOC3 ADC_CH_ADCIN0
void myADC1_init();


//*****************************************************************************
//
// ASYSCTL Configurations
//
//*****************************************************************************

//*****************************************************************************
//
// INTERRUPT Configurations
//
//*****************************************************************************

// Interrupt Settings for INT_myADC0_1
// ISR need to be defined for the registered interrupts
#define INT_myADC0_1 INT_ADCA1
#define INT_myADC0_1_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP1
extern __interrupt void adcA1ISR(void);

//*****************************************************************************
//
// Board Configurations
//
//*****************************************************************************
void	Board_init();
void	ADC_init();
void	ASYSCTL_init();
void	INTERRUPT_init();
void	PinMux_init();

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif  // end of BOARD_H definition
