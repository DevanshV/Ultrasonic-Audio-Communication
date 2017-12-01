//*****************************************************************************
//
// main.c - High frequency audio communication
//
// Github @devanshvaid - Devansh Vaid
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_adc.h"
#include "inc/hw_udma.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/adc.h"
#include "driverlib/systick.h"

//*****************************************************************************
// Sampling rate for microphone. Based on Nyquist, we need atleast 2x our max
// frequency of 22kHz. We have the resources overshoot to avoid issues
//*****************************************************************************
#define NUM_SAMPLES 1024
uint16_t sampling_rate = NUM_SAMPLES * 10 * 5;

//*****************************************************************************
// Buffer for the 5 FFT calculations, representing 1 Bit of data
//*****************************************************************************
uint16_t bit_buffer[5] = { 0 };

//*****************************************************************************
// Flags that contain the current value of the interrupt indicator as displayed
// on the UART.
//*****************************************************************************
uint8_t g_ui32Flags;

//*****************************************************************************
// Store data for the frames and bytes collected
//*****************************************************************************
bool transfer_status = false;
int amplitude_buffer[5] = { 0 };
volatile int bit_output_index = 0;
int frame_count = 0;
char data_byte = 0;

//*****************************************************************************
// Create buffer for PingPong uDMA and create enum for status
//*****************************************************************************
int16_t ADC_Out1[NUM_SAMPLES];
int16_t ADC_Out2[NUM_SAMPLES];
enum BUFFERSTATUS {
    EMPTY,
    FILLING,
    FULL
};
enum BUFFERSTATUS BufferStatus[2];

//*****************************************************************************
// This enum is used to ensure synchronization with initial bits/frames
//*****************************************************************************
enum SYNCHRONIZATION {
    FAILED = -1,
    NONE,
    ONE,
    ZERO,
    COMPLETE
};
volatile enum SYNCHRONIZATION byte_sync = ONE;

//*****************************************************************************
// Store some coefficients for goertzel.These are formatted in Q14 format to
// achieve fixed point calculations
//*****************************************************************************
#define goe_coeff(TARGET_FREQ) ((2.0 * cos((2.0 * pi * (0.5 + ((NUM_SAMPLES * (TARGET_FREQ)) / NUM_SAMPLES))) / 46080)) * pow(2, 14))
#define twenty_khz -25331
#define twenty_one_khz -27685

//*****************************************************************************
// To debug, we can store the last 500 frames
//*****************************************************************************
uint8_t output[5000];

//*****************************************************************************
// Control table for uDMA transfers
//*****************************************************************************
#pragma DATA_ALIGN(ucControlTable, 1024)
uint8_t ucControlTable[1024];

int goertzel(int16_t* data, int sz, int coeff)
{
    int32_t delay;
    int32_t delay_1 = 0;
    int32_t delay_2 = 0;
    int goertzel_value = 0;
    int prod1, prod2, prod3;
    uint32_t input;
    int32_t coef_1 = coeff;
    int i = 0;

    for (i = 0; i < sz; i++) {
        input = data[i] >> 4; // Scale down input to prevent overflow
        delay = input + (short)((delay_1 * coef_1) >> 14) - delay_2;
        delay_2 = delay_1;
        delay_1 = delay;
    }

    prod1 = (delay_1 * delay_1);
    prod2 = (delay_2 * delay_2);
    prod3 = (delay_1 * coef_1) >> 14;
    prod3 = prod3 * delay_2;
    goertzel_value = (prod1 + prod2 - prod3) >> 15;
    goertzel_value <<= 6; // Scale up value for sensitivity

    return goertzel_value;
}

void ADC3IntHandler(void)
{
    ADCIntClear(ADC0_BASE, 0);

    if ((uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT) == UDMA_MODE_STOP)
        && (BufferStatus[0] == FILLING)) {
        BufferStatus[0] = FULL;
        BufferStatus[1] = FILLING;
        uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void*)(ADC0_BASE + ADC_O_SSFIFO0), &ADC_Out1, NUM_SAMPLES);
    }
    else if ((uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT) == UDMA_MODE_STOP)
        && (BufferStatus[1] == FILLING)) {
        BufferStatus[0] = FILLING;
        BufferStatus[1] = FULL;
        uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void*)(ADC0_BASE + ADC_O_SSFIFO0), &ADC_Out2, NUM_SAMPLES);
    }
}

//*****************************************************************************
// Configure the ADC Port 4 Pin 5 (AIN8)
//*****************************************************************************
void ConfigureADCuDMA(void)
{
    // Enables uDMA
    uDMAEnable();
    uDMAControlBaseSet(ucControlTable);

    uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC0, UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);

    // Only allow burst transfers
    uDMAChannelAttributeEnable(UDMA_CHANNEL_ADC0, UDMA_ATTR_USEBURST);

    uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
    uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);

    uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void*)(ADC0_BASE + ADC_O_SSFIFO0), &ADC_Out1, NUM_SAMPLES);
    uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void*)(ADC0_BASE + ADC_O_SSFIFO0), &ADC_Out2, NUM_SAMPLES);

    // Enables DMA channel so it can perform transfers
    uDMAChannelEnable(UDMA_CHANNEL_ADC0);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5);
    SysCtlDelay(80u);

    // Use ADC0 sequence 0 to sample channel 0 once for each timer period
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_HALF, 1);

    // Time for the clock configuration to set
    SysCtlDelay(10);

    IntDisable(INT_ADC0SS0);
    ADCIntDisable(ADC0_BASE, 0u);

    // With sequence disabled, it is now safe to load the new configuration parameters
    ADCSequenceDisable(ADC0_BASE, 0u);

    ADCSequenceConfigure(ADC0_BASE, 0u, ADC_TRIGGER_TIMER, 0u);
    ADCSequenceStepConfigure(ADC0_BASE, 0u, 0u, ADC_CTL_CH8 | ADC_CTL_END | ADC_CTL_IE);

    //Once configuration is set, re-enable the sequencer
    ADCSequenceEnable(ADC0_BASE, 0u);

    ADCIntClear(ADC0_BASE, 0u);
    ADCSequenceDMAEnable(ADC0_BASE, 0);
    IntEnable(INT_ADC0SS0);
}

//*****************************************************************************
// Configure the UART and its pins.
//*****************************************************************************
void ConfigureUART(void)
{
    // Enable the GPIO Peripheral used by the UART.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
// Configure the Sampling Timer
//*****************************************************************************
void ConfigureSamplingTimer(void)
{
    // Enable the peripherals used by this example.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // Configure the two 32-bit periodic timers.
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_PERIODIC);

    // Set the timer to ADC
    ROM_TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
    // Set rate
    ROM_TimerControlStall(TIMER0_BASE, TIMER_A, true); //Assist in debug by stalling timer at breakpoints

    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet() / sampling_rate - 1);

    // These aren't needed as TimerControlTrigger is enabled above, could be used in the future
    // ROM_IntEnable(INT_TIMER0A);
    // ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Enable the timers.
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
}

void process_data(int16_t* ADC_Out)
{
    int sum = 0, n_loop = 0, reset_flags = 0, amplitude = 0;

    //use FFT to calculate magnitude for a single bin
    //The start condition is an out-of-band 21khz byte
    if (byte_sync == COMPLETE) {
        amplitude = (goertzel(ADC_Out, NUM_SAMPLES, twenty_khz) >= 100) ? 1 : 0;
    }
    else {
        amplitude = (goertzel(ADC_Out, NUM_SAMPLES, twenty_one_khz) >= 100) ? 1 : 0;
    }

    //if we detect a high bit (transmission starts with double "1")
    if (amplitude && !transfer_status)
        transfer_status = 1;

    //Transfer mode - store moving sum of bits
    if (transfer_status) {

        //calculate sum/buffer of last 5 frames (5 frames = 1 bit = 100 ms)
        for (n_loop = 0; n_loop < 4; n_loop++) {
            amplitude_buffer[n_loop] = amplitude_buffer[n_loop + 1];
            sum += amplitude_buffer[n_loop];
        }
        amplitude_buffer[4] = amplitude; // add latest value to end of buffer
        sum += amplitude;

        //determine if next bit can be determined (5 frames have been received)
        if (!((frame_count + 1) % 5)) {
            if (byte_sync != COMPLETE) {
                if (byte_sync == ONE && sum == 5) {
                    byte_sync = ZERO;
                }
                else if (byte_sync == ZERO && sum == 0) {
                    byte_sync = ONE;
                }
                else if (byte_sync != FAILED) {
                    byte_sync = FAILED;
                    UARTprintf("Synchronization Failed.. Trying Again \n");
                }
            }

            if (sum > 2)
                data_byte |= 1;

            if (!((bit_output_index + 1) % 8)) {
                if (!data_byte) { // stop condition
                    reset_flags = 1;
                    UARTprintf("\n\n");
                } else if (byte_sync == ONE) {
                    byte_sync = COMPLETE;
                } else if (byte_sync == COMPLETE){
                    UARTprintf("%c", data_byte);
                }
                data_byte = 0;
            }

            bit_output_index++;
            data_byte <<= 1;
        }
        frame_count++;

        if (reset_flags == 1) {
            byte_sync = ONE;
            transfer_status = 0;
            frame_count = 0;
            bit_output_index = 0;
        }
    }
}

//*****************************************************************************
// This example application demonstrates the use of the timers to generate
// periodic interrupts.
//*****************************************************************************
int main(void)
{
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    ROM_FPULazyStackingEnable();

    // Set the clock with PLL for ADC and run at max rate 80 Mhz
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    SysCtlDelay(20u);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); //Enable the clock to TIMER0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); //Enable the clock to ADC module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA); //Enable the clock to uDMA
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //Enables the clock to PORT E
    SysCtlDelay(30u);

    // Enable processor interrupts.
    ROM_IntMasterEnable();

    // Configure ADC8, UART and Sampling Timer
    ConfigureUART();
    ConfigureADCuDMA();
    ConfigureSamplingTimer();

    BufferStatus[0] = FILLING;
    BufferStatus[1] = EMPTY;

    // Infinite loop, interrupts handle the logic
    while (1) {
        if (BufferStatus[0u] == FULL) {
            process_data(ADC_Out1);
            BufferStatus[0u] = EMPTY;
        }
        if (BufferStatus[1u] == FULL) {
            process_data(ADC_Out2);
            BufferStatus[1u] = EMPTY;
        }
    }
}
