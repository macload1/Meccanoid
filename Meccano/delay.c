/*
 * delay.c - Delay in millisecond and microsecond functions
 *
 *  Created on: Jul 7, 2014
 *      Author: Cuong T. Nguyen
 */
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"

#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#include "delay.h"

extern volatile uint32_t delayCounter;

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
