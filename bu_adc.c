//###############################################################################
//
// FILE:   bu_adc.c
//
// TITLE:  Branch Unit ADC Module Implementation
//
// PURPOSE: Implements ADC sampling, RMS calculation, and current measurement
//          for 18 channels with external synchronization support
//
//###############################################################################

//
// Included Files
//
#include <bu_adc.h>

//
// Global Variables
//
uint32_t myADC0Results[NUM_CHANNELS];
unsigned long dwSumPhaseVolt[NUM_CHANNELS] = {0};
unsigned int sampleCount = 0;
float calib = DEFAULT_CALIB_FACTOR;
volatile bool syncSignalReceived = false;

// Branch parameter structure
DWW_CHANNEL_PARAMETER Branch;
unsigned int struct_size;
unsigned long dwwMathBuff;
unsigned int max_curr;

// Demand calculation variables
unsigned int demand_chk_1sec = 0;
unsigned int demand_chk_1hr = 0;
unsigned int demand_chk_24hr = 0;
unsigned int wDemandSumCntr[4] = {0};
unsigned long dw_CurrDemandSum[NUM_CHANNELS] = {0};
dww_structflag *sFlag;

// Debug counter
static int debugCounter = 0;
 int a;
//
// BU_ADC_init - Initialize ADC module and related peripherals
//
 void BU_ADC_init(void)
 {
     int i;

     // Initialize structure size
     struct_size = sizeof(Branch);

     // Configure sync input GPIO
     configureGPIO28();

     // Configure LED output
     configureLED();

     // Enable ADC modules
     ADC_enableConverter(ADCA_BASE);
     ADC_enableConverter(ADCE_BASE);

     // Configure ADC reference voltage
     ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);

     // Delay for ADC power up
     DEVICE_DELAY_US(1000);

     // Configure ADC SOCs for all channels
     // ADCA channels
     ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                  ADC_CH_ADCIN0, 15);
     ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA,
                  ADC_CH_ADCIN1, 15);
     ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA,
                  ADC_CH_ADCIN2, 15);
     ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM1_SOCA,
                  ADC_CH_ADCIN3, 15);
     ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_EPWM1_SOCA,
                  ADC_CH_ADCIN4, 15);
     ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_EPWM1_SOCA,
                  ADC_CH_ADCIN5, 15);
     ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER6, ADC_TRIGGER_EPWM1_SOCA,
                  ADC_CH_ADCIN14, 15);
     ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER7, ADC_TRIGGER_EPWM1_SOCA,
                  ADC_CH_ADCIN15, 15);
     ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER8, ADC_TRIGGER_EPWM1_SOCA,
                  ADC_CH_ADCIN8, 15);
     ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER9, ADC_TRIGGER_EPWM1_SOCA,
                  ADC_CH_ADCIN9, 15);
     ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER10, ADC_TRIGGER_EPWM1_SOCA,
                  ADC_CH_ADCIN10, 15);
     ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER11, ADC_TRIGGER_EPWM1_SOCA,
                  ADC_CH_ADCIN11, 15);
     ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER12, ADC_TRIGGER_EPWM1_SOCA,
                  ADC_CH_ADCIN12, 15);
     ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER13, ADC_TRIGGER_EPWM1_SOCA,
                  ADC_CH_ADCIN13, 15);

     // ADCE channels
     ADC_setupSOC(ADCE_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                  ADC_CH_ADCIN2, 15);
     ADC_setupSOC(ADCE_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA,
                  ADC_CH_ADCIN3, 15);
     ADC_setupSOC(ADCE_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA,
                  ADC_CH_ADCIN4, 15);
     ADC_setupSOC(ADCE_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM1_SOCA,
                  ADC_CH_ADCIN5, 15);

     // Configure ADC interrupt after last conversion
     ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_INT_TRIGGER_EOC13);
     ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
     ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);

     // Register interrupt handler
     Interrupt_register(INT_ADCA1, &adcA1ISR);
     Interrupt_enable(INT_ADCA1);

     // Initialize the accumulators and sample counter
     sampleCount = 0;
     for(i = 0; i < NUM_CHANNELS; i++)
     {
         dwSumPhaseVolt[i] = 0;
         dw_CurrDemandSum[i] = 0;
     }

     // Initialize demand counters
     for(i = 0; i < 4; i++)
     {
         wDemandSumCntr[i] = 0;
     }

     // Initialize sFlag structure
     static dww_structflag flagStruct = {0};
     sFlag = &flagStruct;

     // Start ePWM in UP counting mode
     EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
     EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP);
 }
//
// configureGPIO28 - Configure GPIO 28 as sync input with interrupt
//
void configureGPIO28(void)
{
    // Configure GPIO28 as input with pullup and invert
    GPIO_setPadConfig(SYNC_GPIO_PIN, GPIO_PIN_TYPE_PULLUP | GPIO_PIN_TYPE_INVERT);
    GPIO_setPinConfig(SYNC_GPIO_CONFIG);
    GPIO_setDirectionMode(SYNC_GPIO_PIN, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(SYNC_GPIO_PIN, GPIO_QUAL_SYNC);

    // Configure interrupt on rising edge
    GPIO_setInterruptType(GPIO_INT_XINT1, GPIO_INT_TYPE_RISING_EDGE);
    GPIO_setInterruptPin(SYNC_GPIO_PIN, GPIO_INT_XINT1);

    // Register the interrupt handler and enable interrupt
    Interrupt_register(INT_XINT1, &gpioISR);
    Interrupt_enable(INT_XINT1);
    GPIO_enableInterrupt(GPIO_INT_XINT1);
}

//
// configureLED - Configure LED GPIO for status indication
//
void configureLED(void)
{
    GPIO_setPadConfig(LED_GPIO_PIN, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(LED_GPIO_PIN, GPIO_DIR_MODE_OUT);
}

//
// readGPIO28 - Read sync input state
//
uint32_t readGPIO28(void)
{
    return GPIO_readPin(SYNC_GPIO_PIN);
}

//
// initEPWM - Configure ePWM1 to trigger ADC conversions
//
void initEPWM(void)
{
    // Disable SOCA before configuration
    EPWM_disableADCTrigger(EPWM1_BASE, EPWM_SOC_A);

    // Configure the SOC to occur on the first up-count event
    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPA);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);

    // Set the compare A value and period
    // With 100MHz ePWM clock and these values, we get 50kHz sampling rate
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 10000);
    EPWM_setTimeBasePeriod(EPWM1_BASE, 19999);

    // Set the local ePWM module clock divider to /16 (dividers /4 * /4)
    EPWM_setClockPrescaler(EPWM1_BASE, EPWM_CLOCK_DIVIDER_4, EPWM_HSCLOCK_DIVIDER_4);

    // Freeze the counter initially
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_STOP_FREEZE);
}

//
// gpioISR - Interrupt service routine for sync signal
//
__interrupt void gpioISR(void)
{
    // Set the flag indicating we've received a sync signal
    syncSignalReceived = true;

    // Acknowledge the interrupt
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

//
// adcA1ISR - ADC interrupt service routine for RMS calculation
//
__interrupt void adcA1ISR(void)
{
    int i;
    float avgSquare, rms;
    unsigned long square;

    //
    // Read ADC conversion results for NUM_CHANNELS.
    // Offsets are applied as in your original code.
    //
    myADC0Results[0]  = (signed int)(ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0) - 2048);
    myADC0Results[1]  = (signed int)ADC_readResult(ADCERESULT_BASE, ADC_SOC_NUMBER2) - 2048;
    myADC0Results[2]  = (signed int)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1) - 2048;
    myADC0Results[3]  = (signed int)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER2) - 2048;
    myADC0Results[4]  = (signed int)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER3) - 2048;
    myADC0Results[5]  = (signed int)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER4) - 2048;
    myADC0Results[6]  = (signed int)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER5) - 2048;
    myADC0Results[7]  = (signed int)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER6) - 2048;
    myADC0Results[8]  = (signed int)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER7) - 2048;
    myADC0Results[9]  = (signed int)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER8) - 2048;
    myADC0Results[10] = (signed int)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER9) - 2048;
    myADC0Results[11] = (signed int)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER10) - 2048;
    myADC0Results[12] = (signed int)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER11) - 2048;
    myADC0Results[13] = (signed int)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER12) - 2048;
    myADC0Results[14] = (signed int)ADC_readResult(ADCERESULT_BASE, ADC_SOC_NUMBER3) - 2048;
    myADC0Results[15] = (signed int)ADC_readResult(ADCERESULT_BASE, ADC_SOC_NUMBER0) - 2048;
    myADC0Results[16] = (signed int)ADC_readResult(ADCERESULT_BASE, ADC_SOC_NUMBER1) - 2048;
    myADC0Results[17] = (signed int)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER13) - 2048;

    //
    // For each channel, square the result and accumulate it.
    //
    for(i = 0; i < NUM_CHANNELS; i++)
    {
        square = (unsigned long)myADC0Results[i] * (unsigned long)myADC0Results[i];
        dwSumPhaseVolt[i] += square;
    }

    //
    // Increment the sample counter.
    //
    sampleCount++;

    //
    // Once 100 samples have been collected, compute the RMS for each channel.
    //
    if(sampleCount >= SAMPLE_COUNT_THRESHOLD)
    {
        for(i = 0; i < NUM_CHANNELS; i++)
        {
            avgSquare = (float)dwSumPhaseVolt[i] / (float)SAMPLE_COUNT_THRESHOLD;
            rms = sqrtf(avgSquare) * calib/10000;
            Branch.dww_channel[i].Current = (unsigned int)rms;

            // Reset accumulator for next set of samples
            dwSumPhaseVolt[i] = 0;
        }
        sampleCount = 0;
    }

    //
    // Clear the ADC interrupt flags.
    //
    ADC_clearInterruptStatus(myADC0_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(myADC1_BASE, ADC_INT_NUMBER1);

    //
    // Check and clear overflow flags if necessary.
    //
    if(ADC_getInterruptOverflowStatus(myADC0_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(myADC0_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(myADC0_BASE, ADC_INT_NUMBER1);
    }

    if(ADC_getInterruptOverflowStatus(myADC1_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(myADC1_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(myADC1_BASE, ADC_INT_NUMBER1);
    }

    //
    // Acknowledge the interrupt.
    //
    Interrupt_clearACKGroup(INT_myADC0_1_INTERRUPT_ACK_GROUP);
}
//
// BU_ADC_processLoop - Main processing loop for ADC module
//
void BU_ADC_processLoop(void)
{ a++;
    int pinstate;

    // Increment debug counter
    debugCounter++;

    // Read sync pin state and update LED
    pinstate = GPIO_readPin(SYNC_GPIO_PIN);
    GPIO_writePin(LED_GPIO_PIN, pinstate);

    // Check if sync signal was received
    if (syncSignalReceived)
    {
        // Reset the flag first
        syncSignalReceived = false;

        // Process the sync signal
        if (pinstate == 1)
        {
            // Re-initialize ePWM
            initEPWM();

            // Restart ePWM triggering
            EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
            EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP);
        }

        // Re-enable the GPIO interrupt for the next sync signal
        GPIO_enableInterrupt(GPIO_INT_XINT1);
    }

    // Process calculations

    dww_max_min_Calc();
    dww_timer();
    dww_demand_Calc();
}



//
// dww_max_min_Calc - Calculate maximum and minimum values
//
void dww_max_min_Calc(void)
{
    unsigned int i;
    for (i = 0; i < MAX_CT_LENGTH; i++)
    {
        if (((Branch.dww_channel[i].Current != 0xFFFF) &&
             ((Branch.dww_channel[i].Maxcurrent < Branch.dww_channel[i].Current) ||
              (Branch.dww_channel[i].Maxcurrent == 0xFFFF))))
        {
            Branch.dww_channel[i].Maxcurrent = Branch.dww_channel[i].Current;
        }

        if (((Branch.dww_channel[i].Current != 0x0000) &&
             ((Branch.dww_channel[i].Mincurrent > Branch.dww_channel[i].Current) ||
              (Branch.dww_channel[i].Mincurrent == 0x0000))))
        {
            Branch.dww_channel[i].Mincurrent = Branch.dww_channel[i].Current;
        }

        if (((Branch.dww_channel[i].CTCurrDemand_1hr != 0xFFFF) &&
             ((Branch.dww_channel[i].CTMaxCurrDemand_1hr < Branch.dww_channel[i].CTCurrDemand_1hr) ||
              (Branch.dww_channel[i].CTMaxCurrDemand_1hr == 0xFFFF))))
        {
            Branch.dww_channel[i].CTMaxCurrDemand_1hr = Branch.dww_channel[i].CTCurrDemand_1hr;
        }
    }
}

//
// dww_timer - Handle timing for demand calculations
//
void dww_timer(void)
{
    // 1 second timer
    if (demand_chk_1sec >= DEMAND_1SEC_COUNT)
    {
        sFlag->demand_chk_1sec = 1;
        demand_chk_1sec = 0;
        demand_chk_1hr++;
    }
    else
    {
        demand_chk_1sec++;
    }

    // 1 hour timer
    if(demand_chk_1hr >= DEMAND_1HR_COUNT)
    {
        sFlag->demand_chk_1hr = 1;
        demand_chk_1hr = 0;
        demand_chk_24hr++;
    }

    // 24 hour timer
    if(demand_chk_24hr >= DEMAND_24HR_COUNT)
    {
        sFlag->demand_chk_24hr = 1;
        demand_chk_24hr = 0;
    }
}

//
// dww_demand_Calc - Calculate demand values
//
void dww_demand_Calc(void)
{
    int i;
    if (sFlag->demand_chk_1sec == 1)
    {
        for (i = 0; i < MAX_CT_LENGTH; i++)
        {
            dw_CurrDemandSum[i] += Branch.dww_channel[i].Current;
        }
        sFlag->demand_chk_1sec = 0;
        wDemandSumCntr[0]++;
    }

    if (sFlag->demand_chk_1hr == 1)
    {
        for (i = 0; i < MAX_CT_LENGTH; i++)
        {
            Branch.dww_channel[i].CTCurrDemand_1hr = (unsigned int)(dw_CurrDemandSum[i] / wDemandSumCntr[0]);
            dw_CurrDemandSum[i] = 0;
        }
        sFlag->demand_chk_1hr = 0;
        wDemandSumCntr[0] = 0;
    }

    if (sFlag->demand_chk_24hr == 1)
    {
        for (i = 0; i < MAX_CT_LENGTH; i++)
        {
            Branch.dww_channel[i].CTMaxCurrDemand_24hr = Branch.dww_channel[i].CTMaxCurrDemand_1hr;
            Branch.dww_channel[i].CTMaxCurrDemand_1hr = 0;
        }
        sFlag->demand_chk_24hr = 0;
    }
}
