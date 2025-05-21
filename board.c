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

#include "board.h"
#include "bu_adc.h"

//*****************************************************************************
//
// Board Configurations
// Initializes the rest of the modules. 
// Call this function in your application if you wish to do all module 
// initialization.
// If you wish to not use some of the initializations, instead of the 
// Board_init use the individual Module_inits
//
//*****************************************************************************
void Board_init()
{
	EALLOW;

	PinMux_init();
	ASYSCTL_init();
	BU_ADC_init();

	EDIS;
}

//*****************************************************************************
//
// PINMUX Configurations
//
//*****************************************************************************
void PinMux_init()
{
	//
	// PinMux for modules assigned to CPU1
	//
	
	//
	// ANALOG -> myANALOGPinMux0 Pinmux
	//
	// Analog PinMux for A0, B15, C15, DACA_OUT
	GPIO_setPinConfig(GPIO_231_GPIO231);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(231, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A2, B6, C9, PGA1_INP, GPIO224
	GPIO_setPinConfig(GPIO_224_GPIO224);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(224, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A3, B3, PGA2_INP, C5, GPIO242
	GPIO_setPinConfig(GPIO_242_GPIO242);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(242, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A4, B8, C14
	GPIO_setPinConfig(GPIO_225_GPIO225);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(225, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A5, B12, C2, PGA2_INM
	GPIO_setPinConfig(GPIO_244_GPIO244);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(244, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A6, D14, E14, GPIO228
	GPIO_setPinConfig(GPIO_228_GPIO228);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(228, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A7, B30, C3, D12, E30
	GPIO_setPinConfig(GPIO_245_GPIO245);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(245, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A8, B0, C11
	GPIO_setPinConfig(GPIO_241_GPIO241);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(241, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A9, GPIO227
	GPIO_setPinConfig(GPIO_227_GPIO227);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(227, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A10, B1, C10, GPIO230
	GPIO_setPinConfig(GPIO_230_GPIO230);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(230, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A11, B10, C0, PGA2_OUT
	GPIO_setPinConfig(GPIO_237_GPIO237);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(237, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A12, C1, E11, PGA3_INP
	GPIO_setPinConfig(GPIO_238_GPIO238);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(238, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A14, B14, C4, PGA1_OUT
	GPIO_setPinConfig(GPIO_239_GPIO239);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(239, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A15, B9, C7, PGA1_INM
	GPIO_setPinConfig(GPIO_233_GPIO233);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(233, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A17, B17, C17, GPIO20
	GPIO_setPinConfig(GPIO_20_GPIO20);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(20, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A18, B18, C18, GPIO21
	GPIO_setPinConfig(GPIO_21_GPIO21);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(21, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A19, B19, C19, GPIO13
	GPIO_setPinConfig(GPIO_13_GPIO13);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(13, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A20, B20, C20, GPIO12
	GPIO_setPinConfig(GPIO_12_GPIO12);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(12, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A24, D0, E0, GPIO11
	GPIO_setPinConfig(GPIO_11_GPIO11);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(11, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A25, D3, E3, GPIO17
	GPIO_setPinConfig(GPIO_17_GPIO17);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(17, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A26, D6, E6
	GPIO_setPinConfig(GPIO_209_GPIO209);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(209, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A27, D9, E9, GPIO212
	GPIO_setPinConfig(GPIO_212_GPIO212);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(212, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A28, D19, E19, GPIO215
	GPIO_setPinConfig(GPIO_215_GPIO215);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(215, GPIO_ANALOG_ENABLED);
	// Analog PinMux for B2, C6, E12, GPIO226
	GPIO_setPinConfig(GPIO_226_GPIO226);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(226, GPIO_ANALOG_ENABLED);
	// Analog PinMux for B4, C8, GPIO236
	GPIO_setPinConfig(GPIO_236_GPIO236);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(236, GPIO_ANALOG_ENABLED);
	// Analog PinMux for B5, D15, E15, PGA3_OUT
	GPIO_setPinConfig(GPIO_252_GPIO252);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(252, GPIO_ANALOG_ENABLED);
	// Analog PinMux for B11, D16, E16, PGA3_INM
	GPIO_setPinConfig(GPIO_251_GPIO251);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(251, GPIO_ANALOG_ENABLED);
	// Analog PinMux for B24, D1, E1, GPIO33
	GPIO_setPinConfig(GPIO_33_GPIO33);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(33, GPIO_ANALOG_ENABLED);
	// Analog PinMux for B25, D4, E4, GPIO24
	GPIO_setPinConfig(GPIO_24_GPIO24);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(24, GPIO_ANALOG_ENABLED);
	// Analog PinMux for B26, D7, E7
	GPIO_setPinConfig(GPIO_210_GPIO210);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(210, GPIO_ANALOG_ENABLED);
	// Analog PinMux for B27, D10, E10, GPIO213
	GPIO_setPinConfig(GPIO_213_GPIO213);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(213, GPIO_ANALOG_ENABLED);
	// Analog PinMux for C24, D2, E2, GPIO16
	GPIO_setPinConfig(GPIO_16_GPIO16);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(16, GPIO_ANALOG_ENABLED);
	// Analog PinMux for C25, D5, E5
	GPIO_setPinConfig(GPIO_208_GPIO208);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(208, GPIO_ANALOG_ENABLED);
	// Analog PinMux for C26, D8, E8, GPIO211
	GPIO_setPinConfig(GPIO_211_GPIO211);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(211, GPIO_ANALOG_ENABLED);
	// Analog PinMux for C27, D18, E18, GPIO214
	GPIO_setPinConfig(GPIO_214_GPIO214);
	// AGPIO -> Analog mode selected
	GPIO_setAnalogMode(214, GPIO_ANALOG_ENABLED);
	// Analog PinMux for D20, E20, VREFHI
	GPIO_setPinConfig(GPIO_234_GPIO234);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(234, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A13, B13, C13, D13, E13, VREFLO
	GPIO_setPinConfig(GPIO_235_GPIO235);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(235, GPIO_ANALOG_ENABLED);
	// Analog PinMux for A1, B7, D11, CMP1_DACL
	GPIO_setPinConfig(GPIO_232_GPIO232);
	// AIO -> Analog mode selected
	GPIO_setAnalogMode(232, GPIO_ANALOG_ENABLED);

}

//*****************************************************************************
//
// ASYSCTL Configurations
//
//*****************************************************************************
void ASYSCTL_init(){
	//
	// asysctl initialization
	//
	// Disables the temperature sensor output to the ADC.
	//
	ASysCtl_disableTemperatureSensor();
	//
	// Set the analog voltage reference selection to internal.
	//
	ASysCtl_setAnalogReferenceInternal( ASYSCTL_ANAREF_INTREF_ADCA | ASYSCTL_ANAREF_INTREF_ADCB | ASYSCTL_ANAREF_INTREF_ADCC | ASYSCTL_ANAREF_INTREF_ADCD | ASYSCTL_ANAREF_INTREF_ADCE );

	//
	// Set the internal analog voltage reference selection to 1.65V.
	//
	ASysCtl_setAnalogReference1P65( ASYSCTL_ANAREF_ADCA | ASYSCTL_ANAREF_ADCB | ASYSCTL_ANAREF_ADCC | ASYSCTL_ANAREF_ADCD | ASYSCTL_ANAREF_ADCE );
}
