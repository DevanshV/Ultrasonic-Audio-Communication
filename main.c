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
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/adc.h"

//*****************************************************************************
// Sampling rate for microphone. Based on Nyquist, we need atleast 2x our max
// frequency of 22kHz. We have the resources overshoot to avoid issues
//*****************************************************************************
uint16_t sampling_rate = 46080;

//*****************************************************************************
// Buffer for the 5 FFT calculations, representing 1 Bit of data
//*****************************************************************************
uint16_t bit_buffer[5] = {0};

//*****************************************************************************
// Flags that contain the current value of the interrupt indicator as displayed
// on the UART.
//*****************************************************************************
uint8_t g_ui32Flags;

//*****************************************************************************
// 1024 points of microphone data
//*****************************************************************************
int32_t data[1024];

bool start_received;

#define goe_coeff(TARGET_FREQ) ((2.0 * cos((2.0 * pi * (0.5 + ((1024 * (TARGET_FREQ)) / 1024))) / 46080)) >> 14)

int index = 0;

int goertzel(int32_t* data, int sz)
{
    int32_t delay;
    int32_t delay_1 = 0;
    int32_t delay_2 = 0;
    int goertzel_value = 0;
    int prod1, prod2, prod3;
    uint32_t input;
    int32_t coef_1 = -29993;
    int i = 0;

    for (i = 0; i < sz; i++) {
        input = data[i] >> 4;     // Scale down input to prevent overflow
        delay = input + (short)((delay_1*coef_1) >> 14) - delay_2;
        delay_2 = delay_1;
        delay_1 = delay;
    }

    prod1 = (delay_1 * delay_1);
    prod2 = (delay_2 * delay_2);
    prod3 = (delay_1 *  coef_1) >> 14;
    prod3 = prod3 * delay_2;
    goertzel_value = (prod1 + prod2 - prod3) >> 15;
    goertzel_value <<= 6;    // Scale up value for sensitivity

    return goertzel_value;
}

void ADC3IntHandler(void){
    //TimerDisable(TIMER0_BASE, TIMER_A);

    uint32_t pui32ADC0Value[3];
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);

    data[index] = pui32ADC0Value[0];
    index ++;

    if (index == 1024) {
        int amplitude = goertzel(data, index);
        amplitude = (amplitude >= 100) ? 1 : 0;
        ROM_IntMasterDisable();
        UARTprintf("%i \n", amplitude);
        ROM_IntMasterEnable();
        index = 0;
    }
}

//*****************************************************************************
// Configure the ADC Port 4 Pin 5 (AIN8)
//*****************************************************************************
void ConfigureADC8 (void)
{
    // Enable ADC0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);

    // Specify ADC pin and enable ADC type
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5);

    // Select sequence 3 (1 sample) and Trigger Processor (enabled by function call)
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH8 | ADC_CTL_IE | ADC_CTL_END);

    // Finally, enable sequence
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntEnable(ADC0_BASE, 3);
    IntEnable(INT_ADC0SS3);
    ADCIntClear(ADC0_BASE, 3);
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
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    // Set the timer to ADC
    ROM_TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
    // Set rate
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet()/sampling_rate);

    // These aren't needed as TimerControlTrigger is enabled above
    // ROM_IntEnable(INT_TIMER0A);
    // ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Enable the timers.
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
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

    // Enable processor interrupts.
    ROM_IntMasterEnable();

    // Configure ADC8, UART and Sampling Timer
    ConfigureUART();
    ConfigureADC8();
    ConfigureSamplingTimer();

    // Enable port-pin for STATUS LED
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1);

    // Infinite loop, interrupts handle the logic
    while(1) {}
}
