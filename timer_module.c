#include "timer_module.h"
#include "data_structures.h"
#include "can_module.h"
#include "board_module.h"

// Timer-related flags
volatile bool timer2Expired = false;
volatile bool gainAckReceived = false;
volatile uint8_t retryCount = 0;
extern uint8_t frameCounter;
extern bool sequenceStarted;
extern bool allFramesReceived;
 int cpuTimer1IntCount;

// Initialize all CPU timers
void initCPUTimers(void)
{
    // Initialize timer periods to maximum
    CPUTimer_setPeriod(CPUTIMER1_BASE, 0xFFFFFFFF);
    CPUTimer_setPeriod(CPUTIMER2_BASE, 0xFFFFFFFF);

    // Initialize pre-scale counters to divide by 1 (SYSCLKOUT)
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0);
    CPUTimer_setPreScaler(CPUTIMER2_BASE, 0);

    // Make sure timers are stopped
    CPUTimer_stopTimer(CPUTIMER1_BASE);
    CPUTimer_stopTimer(CPUTIMER2_BASE);

    // Reload all counter registers with period values
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER2_BASE);

    // Reset interrupt counters
    cpuTimer1IntCount = 0;
}

// Configure a specific CPU timer
void configCPUTimer(uint32_t cpuTimer, float freq, float period)
{
    uint32_t temp;

    // Initialize timer period:
    temp = (uint32_t)((freq / 1000000) * period);
    CPUTimer_setPeriod(cpuTimer, temp - 1);

    // Set pre-scale counter to divide by 1 (SYSCLKOUT):
    CPUTimer_setPreScaler(cpuTimer, 0);

    // Initializes timer control register. The timer is stopped, reloaded,
    // free run disabled, and interrupt enabled.
    // Additionally, the free and soft bits are set
    CPUTimer_stopTimer(cpuTimer);
    CPUTimer_reloadTimerCounter(cpuTimer);
    CPUTimer_setEmulationMode(cpuTimer,
                             CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_enableInterrupt(cpuTimer);

    // Reset interrupt counters for the specified timer
    if (cpuTimer == CPUTIMER1_BASE)
    {
        cpuTimer1IntCount = 0;
    }
}

// Initialize frame tracking timer
void initFrameTracking(void)
{
    // Initialize CPUTimer1 for 2-second timeout
    initCPUTimers();
    configCPUTimer(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, 2000000); // 2 seconds

    // Register timer interrupt
    Interrupt_register(INT_TIMER1, &frameTrackingTimerISR);
    CPUTimer_enableInterrupt(CPUTIMER1_BASE);
    Interrupt_enable(INT_TIMER1);
}

// Configure Timer2 for gain transmission retry
void configureTimer2(void)
{
    // Initialize CPUTimer2
    CPUTimer_setPeriod(CPUTIMER2_BASE, 0xFFFFFFFF);
    CPUTimer_setPreScaler(CPUTIMER2_BASE, 0);
    CPUTimer_stopTimer(CPUTIMER2_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER2_BASE);

    // Configure for 2 second period
    float freq = DEVICE_SYSCLK_FREQ;
    configCPUTimer(CPUTIMER2_BASE, freq, TIMER2_PERIOD_US);

    // Register and enable Timer2 interrupt
    Interrupt_register(INT_TIMER2, &cpuTimer2ISR);
    CPUTimer_enableInterrupt(CPUTIMER2_BASE);
    Interrupt_enable(INT_TIMER2);
}

// Start/restart frame sequence timer
void startFrameSequenceTimer(void)
{
    // Always stop and reload the timer before starting
    CPUTimer_stopTimer(CPUTIMER1_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    timerExpired = false;
    CPUTimer_startTimer(CPUTIMER1_BASE);
}

// Start discovery timer
void startDiscoveryTimer(void)
{
    CPUTimer_stopTimer(CPUTIMER1_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    timerExpired = false;
    discoveryInProgress = true;
    CPUTimer_startTimer(CPUTIMER1_BASE);
}

// Start gain transmission retry sequence
void startGainTransmissionRetry(void)
{
    // Reset flags and counter
    timer2Expired = false;
    gainAckReceived = false;
    retryCount = 0;

    // Start initial transmission
    transmitGainFrames();

    // Start timer
    CPUTimer_startTimer(CPUTIMER2_BASE);
}

// Timer1 ISR for frame tracking and discovery timeout
__interrupt void frameTrackingTimerISR(void)
{
    // Handle existing frame tracking timeout
    if (!allFramesReceived && !discoveryInProgress)
    {
        timerExpired = true;
        requestRetransmission = true;
    }
    // Handle discovery timeout
    else if (discoveryInProgress && !discoveryAckReceived)
    {
        timerExpired = true;
    }

    // Stop timer - it will be restarted when needed
    CPUTimer_stopTimer(CPUTIMER1_BASE);

    // Acknowledge interrupt
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

// Timer2 ISR for gain transmission retry
__interrupt void cpuTimer2ISR(void)
{
    timer2Expired = true;

    // Acknowledge interrupt
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

