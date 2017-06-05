//*****************************************************************************
//
// blinky.c - Simple example to blink the on-board LED.
//
// Copyright (c) 2012-2016 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.1.3.156 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_gpio.h"

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#include "utils/uartstdio.h"
#include "utils/cmdline.h"

#include "Meccano.h"
#include "delay.h"
#include "usb_handler.h"

#define LED_RED         GPIO_PIN_1
#define LED_BLUE        GPIO_PIN_2
#define LED_GREEN       GPIO_PIN_3


//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Meccano (Meccano)</h1>
//!
//! A very simple example that blinks the on-board LED using direct register
//! access.
//
//*****************************************************************************

// Input buffer for commands
static char g_cInput[128];
bool processCmdLine;


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    UARTprintf("Error at line %d of %s\n", ui32Line, pcFilename);
    while(1)
    {
    }
}
#endif



//*****************************************************************************
//
// Passes control to the bootloader and initiates a remote software update.
//
// This function passes control to the bootloader and initiates an update of
// the main application firmware image via UART0 or USB depending
// upon the specific boot loader binary in use.
//
// \return Never returns.
//
//*****************************************************************************
void
JumpToBootLoader(void)
{
    //
    // Disable all processor interrupts.  Instead of disabling them
    // one at a time, a direct write to NVIC is done to disable all
    // peripheral interrupts.
    //
    HWREG(NVIC_DIS0) = 0xffffffff;
    HWREG(NVIC_DIS1) = 0xffffffff;

    //
    // Return control to the boot loader.  This is a call to the SVC
    // handler in the boot loader.
    //
    (*((void (*)(void))(*(uint32_t *)0x2c)))();
}

//*****************************************************************************
//
// Command: help
//
// Print the help strings for all commands.
//
//*****************************************************************************
int CMD_help(int argc, char **argv)
{
    int32_t i32Index;

    (void)argc;
    (void)argv;

    //
    // Start at the beginning of the command table
    //
    i32Index = 0;

    //
    // Get to the start of a clean line on the serial output.
    //
    UARTprintf("\nAvailable Commands\n------------------\n\n");

    //
    // Display strings until we run out of them.
    //
    while(g_psCmdTable[i32Index].pcCmd)
    {
        UARTprintf("%17s %s\n", g_psCmdTable[i32Index].pcCmd,
                   g_psCmdTable[i32Index].pcHelp);
        i32Index++;
    }

    //
    // Leave a blank line after the help strings.
    //
    UARTprintf("\n");

    return (0);
}

//*****************************************************************************
//
// Command: echo
//
//
//*****************************************************************************
int CMD_echo(int argc, char **argv) {
    int i;
    for (i = 0 ; i < argc; i++)
    {
        UARTprintf("%s%s", i ? " " : "",argv[i]);
    }
    UARTprintf("\n");
    return (0);
}

// Toggle led
int CMD_led(int argc, char **argv) {
    if (argc < 2)
        return (CMDLINE_TOO_FEW_ARGS);

    if (argc > 2)
        return (CMDLINE_TOO_MANY_ARGS);

    if (strcmp(argv[1],"on") == 0)  {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    }
    else if (strcmp(argv[1],"off") == 0 ) {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
    }
    else
        return (CMDLINE_INVALID_ARG);
    return (0);
}

// Change Meccano led color
int CMD_mec_led(int argc, char **argv) {
    int r, g, b, t;
    if (argc < 5)
        return (CMDLINE_TOO_FEW_ARGS);

    if (argc > 5)
        return (CMDLINE_TOO_MANY_ARGS);

    r = atoi(argv[1]);
    g = atoi(argv[2]);
    b = atoi(argv[3]);
    t = atoi(argv[4]);
    setLEDColor(r, g, b, t);
    return (0);
}

// Change Meccano servo color
int CMD_mec_sled(int argc, char **argv) {
    int n, clr;
    if (argc < 3)
        return (CMDLINE_TOO_FEW_ARGS);

    if (argc > 3)
        return (CMDLINE_TOO_MANY_ARGS);

    n = atoi(argv[1]);
    clr = atoi(argv[2]);
    if((clr < 240) | (clr > 247))
        return (CMDLINE_INVALID_ARG);
    setServoColor(n, clr);
    return (0);
}

// Change Meccano servo position
int CMD_mec_spos(int argc, char **argv) {
    int n, pos;
    if (argc < 3)
        return (CMDLINE_TOO_FEW_ARGS);

    if (argc > 3)
        return (CMDLINE_TOO_MANY_ARGS);

    n = atoi(argv[1]);
    pos = atoi(argv[2]);
    if(pos > 239)
        return (CMDLINE_INVALID_ARG);
    setServoPosition(n, pos);
    return (0);
}

// Jump to the bootloader to update the MCU
int CMD_bootloader(int argc, char **argv) {
    JumpToBootLoader();
    return (0);
}

// Table of valid command strings, callback functions and help messages.

tCmdLineEntry g_psCmdTable[] =
{
    {"help",     CMD_help,      " : Display list of commands" },
    {"echo",     CMD_echo,      " : Echo Arguments"},
    {"led",      CMD_led,       " : set led [on|off]"},
    {"mecled",   CMD_mec_led,   " : set mecled [r] [g] [b] [t] => [0..7]"},
    {"mecsled",  CMD_mec_sled,  " : set mecsled [n] [clr] => n = [0..3], clr = [240..247]"},
    {"mecspos",  CMD_mec_spos,  " : set mecspos [n] [pos] => n = [0..3], pos = [0..239]"},
    {"bootloader",  CMD_bootloader,  " : bootloader - jumps to the bootloader for update"},
    {0,0,0}
};

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}






//*****************************************************************************
//
// Blink the on-board LED.
//
//*****************************************************************************
int
main(void)
{
    volatile uint32_t ui32Loop;
    int32_t i32CommandStatus;

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPULazyStackingEnable();

    //
    // Setup the system clock to run at 80 Mhz from PLL with crystal reference
    // Use "SYSCTL_SYSDIV_4" for 50 MHz
    //
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|
                    SYSCTL_OSC_MAIN);

    //
    // Initialize the UART.
    //
    ConfigureUART();

    UARTprintf("\033[2JMy Meccanoid\n");

    //
    // Initialise USB
    //
    usb_init();

    //
    // Initialise delay functions
    //
    delay_init();

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Check if the peripheral access is enabled.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    //
    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    //
    // Initialise the Meccano Servos and LEDs
    //
    MeccanoInit();

    //
    // Loop forever.
    //
    while(1)
    {
//        //
//        // Turn on the LED.
//        //
//        GPIOPinWrite(GPIO_PORTF_BASE, LED_BLUE, LED_BLUE);
//
//        //
//        // Delay for a bit
//        //
//        SysCtlDelay(5000000);
//        //
//        // Turn off the LED.
//        //
//        GPIOPinWrite(GPIO_PORTF_BASE, LED_BLUE, 0x0);
//
//        //
//        // Delay for a bit
//        //
//        SysCtlDelay(5000000);





        //communicate();
        //
        // Check if a carriage return is present in the UART Buffer.
        //
        if (UARTPeek('\r') != -1)
        {
            UARTgets(g_cInput, sizeof(g_cInput));

            i32CommandStatus = CmdLineProcess(g_cInput);

            switch (i32CommandStatus)
            {
            case CMDLINE_BAD_CMD:
                UARTprintf("Bad command!\n");
                break;
            case CMDLINE_TOO_MANY_ARGS:
                UARTprintf("Too many arguments for command processor!\n");
                break;
            case CMDLINE_TOO_FEW_ARGS:
                UARTprintf("Too few arguments for command processor!\n");
                break;
            case CMDLINE_INVALID_ARG:
                UARTprintf("Invalid argument for command processor!\n");
                break;
            default:
                break;
            }
        }

//       //
//       // Pass control to whichever flavor of boot loader the board is configured
//       // with.
//       //
//       JumpToBootLoader();



//        delay1ms5();
//        GPIOPinWrite(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN, GPIO_MECCANO_PIN);
//        delay1ms5();
//        GPIOPinWrite(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN, 0);
    }
}
