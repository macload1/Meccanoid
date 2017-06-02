/*
 * delay.c - Delay in millisecond and microsecond functions
 *
 *  Created on: Jul 7, 2014
 *      Author: Cuong T. Nguyen
 */
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"

#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"

#include "delay.h"

volatile uint32_t delayCounter;



//*****************************************************************************
//
// The global system tick counter.
//
//*****************************************************************************
volatile uint32_t g_ui32SysTickCount = 0;


//*****************************************************************************
//
// The system tick rate expressed both as ticks per second and a millisecond
// period.
//
//*****************************************************************************
#define SYSTICKS_PER_SECOND 100
#define SYSTICK_PERIOD_MS (1000 / SYSTICKS_PER_SECOND)

//*****************************************************************************
//
// Interrupt handler for the system tick counter.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Update our system tick counter.
    //
    g_ui32SysTickCount++;
}

//*****************************************************************************
//
// The interrupt handler for the fifth timer interrupt.
//
//*****************************************************************************
void
Timer5IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);

    delayCounter++;
}

void delay_init(void)
{
    //
    // Enable the peripherals used by this example.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Configure the 32-bit periodic timers.
    //
    TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER5_BASE, TIMER_A, SysCtlClockGet()/100000);

    //
    // Setup the interrupt for the timer timeout.
    //
    IntEnable(INT_TIMER5A);
    TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Enable the timers.
    //
    TimerLoadSet(TIMER5_BASE, TIMER_A, 100);
    TimerEnable(TIMER5_BASE, TIMER_A);

    //
    // Enable the system tick.
    //
    ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / SYSTICKS_PER_SECOND);
    ROM_SysTickIntEnable();
    ROM_SysTickEnable();

	return;
}

void delayMs(uint32_t ui32Ms) {

    // 1 clock cycle = 1 / SysCtlClockGet() second
    // 1 SysCtlDelay = 3 clock cycle = 3 / SysCtlClockGet() second
    // 1 second = SysCtlClockGet() / 3
    // 0.001 second = 1 ms = SysCtlClockGet() / 3 / 1000

    SysCtlDelay(ui32Ms * (SysCtlClockGet() / 3350)); // 2us compensation
}

void delayUs(uint32_t ui32Us) {
    SysCtlDelay(ui32Us *10);
    // Not very good!
}

void delay1ms5(void) {
    delayCounter = 0;
    TimerLoadSet(TIMER5_BASE, TIMER_A, 75000);
    TimerEnable(TIMER5_BASE, TIMER_A);
    while(delayCounter == 0){}
    TimerDisable(TIMER5_BASE, TIMER_A);
}

void delay417us(void) {
    delayCounter = 0;
    TimerLoadSet(TIMER5_BASE, TIMER_A, 20650);
    TimerEnable(TIMER5_BASE, TIMER_A);
    while(delayCounter == 0){}
    TimerDisable(TIMER5_BASE, TIMER_A);
}
