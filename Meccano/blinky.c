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
#include "driverlib/timer.h"
#include "driverlib/uart.h"

#include "usblib/usblib.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdbulk.h"
#include "usb_bulk_structs.h"

#include "utils/uartstdio.h"
#include "utils/cmdline.h"

#include "Meccano.h"
#include "delay.h"

#define LED_RED         GPIO_PIN_1
#define LED_BLUE        GPIO_PIN_2
#define LED_GREEN       GPIO_PIN_3

//*****************************************************************************
//
// The global system tick counter.
//
//*****************************************************************************
volatile uint32_t g_ui32SysTickCount = 0;

//*****************************************************************************
//
// Variables tracking transmit and receive counts.
//
//*****************************************************************************
volatile uint32_t g_ui32TxCount = 0;
volatile uint32_t g_ui32RxCount = 0;
#ifdef DEBUG
uint32_t g_ui32UARTRxErrors = 0;
#endif

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Meccano (Meccano)</h1>
//!
//! A very simple example that blinks the on-board LED using direct register
//! access.
//
//*****************************************************************************
volatile uint32_t delayCounter;
volatile uint32_t g_ui32Flags;

// Input buffer for commands
static char g_cInput[128];
bool processCmdLine;

//*****************************************************************************
//
// Debug-related definitions and declarations.
//
// Debug output is available via UART0 if DEBUG is defined during build.
//
//*****************************************************************************
#ifdef DEBUG
//*****************************************************************************
//
// Map all debug print calls to UARTprintf in debug builds.
//
//*****************************************************************************
#define DEBUG_PRINT UARTprintf

#else

//*****************************************************************************
//
// Compile out all debug print calls in release builds.
//
//*****************************************************************************
#define DEBUG_PRINT while(0) ((int32_t (*)(char *, ...))0)
#endif

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
// Flags used to pass commands from interrupt context to the main loop.
//
//*****************************************************************************
#define COMMAND_PACKET_RECEIVED 0x00000001
#define COMMAND_STATUS_UPDATE   0x00000002

volatile uint32_t g_ui32Flags = 0;
char *g_pcStatus;

//*****************************************************************************
//
// Global flag indicating that a USB configuration has been set.
//
//*****************************************************************************
static volatile bool g_bUSBConfigured = false;

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
// Receive new data and echo it back to the host.
//
// \param psDevice points to the instance data for the device whose data is to
// be processed.
// \param pi8Data points to the newly received data in the USB receive buffer.
// \param ui32NumBytes is the number of bytes of data available to be processed.
//
// This function is called whenever we receive a notification that data is
// available from the host. We read the data, byte-by-byte and swap the case
// of any alphabetical characters found then write it back out to be
// transmitted back to the host.
//
// \return Returns the number of bytes of data processed.
//
//*****************************************************************************
static uint32_t
EchoNewDataToHost(tUSBDBulkDevice *psDevice, uint8_t *pi8Data,
                  uint_fast32_t ui32NumBytes)
{
    uint_fast32_t ui32Loop, ui32Space, ui32Count;
    uint_fast32_t ui32ReadIndex;
    uint_fast32_t ui32WriteIndex;
    tUSBRingBufObject sTxRing;

    //
    // Get the current buffer information to allow us to write directly to
    // the transmit buffer (we already have enough information from the
    // parameters to access the receive buffer directly).
    //
    USBBufferInfoGet(&g_sTxBuffer, &sTxRing);

    //
    // How much space is there in the transmit buffer?
    //
    ui32Space = USBBufferSpaceAvailable(&g_sTxBuffer);

    //
    // How many characters can we process this time round?
    //
    ui32Loop = (ui32Space < ui32NumBytes) ? ui32Space : ui32NumBytes;
    ui32Count = ui32Loop;

    //
    // Update our receive counter.
    //
    g_ui32RxCount += ui32NumBytes;

    //
    // Dump a debug message.
    //
    DEBUG_PRINT("Received %d bytes\n", ui32NumBytes);

    //
    // Set up to process the characters by directly accessing the USB buffers.
    //
    ui32ReadIndex = (uint32_t)(pi8Data - g_pui8USBRxBuffer);
    ui32WriteIndex = sTxRing.ui32WriteIndex;

    while(ui32Loop)
    {
        //
        // Copy from the receive buffer to the transmit buffer converting
        // character case on the way.
        //

        //
        // Is this a lower case character?
        //
        if((g_pui8USBRxBuffer[ui32ReadIndex] >= 'a') &&
           (g_pui8USBRxBuffer[ui32ReadIndex] <= 'z'))
        {
            //
            // Convert to upper case and write to the transmit buffer.
            //
            g_pui8USBTxBuffer[ui32WriteIndex] =
                (g_pui8USBRxBuffer[ui32ReadIndex] - 'a') + 'A';
        }
        else
        {
            //
            // Is this an upper case character?
            //
            if((g_pui8USBRxBuffer[ui32ReadIndex] >= 'A') &&
               (g_pui8USBRxBuffer[ui32ReadIndex] <= 'Z'))
            {
                //
                // Convert to lower case and write to the transmit buffer.
                //
                g_pui8USBTxBuffer[ui32WriteIndex] =
                    (g_pui8USBRxBuffer[ui32ReadIndex] - 'Z') + 'z';
            }
            else
            {
                //
                // Copy the received character to the transmit buffer.
                //
                g_pui8USBTxBuffer[ui32WriteIndex] =
                    g_pui8USBRxBuffer[ui32ReadIndex];
            }
        }

        //
        // Move to the next character taking care to adjust the pointer for
        // the buffer wrap if necessary.
        //
        ui32WriteIndex++;
        ui32WriteIndex =
            (ui32WriteIndex == BULK_BUFFER_SIZE) ? 0 : ui32WriteIndex;

        ui32ReadIndex++;
        ui32ReadIndex = (ui32ReadIndex == BULK_BUFFER_SIZE) ? 0 : ui32ReadIndex;

        ui32Loop--;
    }

    //
    // We've processed the data in place so now send the processed data
    // back to the host.
    //
    USBBufferDataWritten(&g_sTxBuffer, ui32Count);

    DEBUG_PRINT("Wrote %d bytes\n", ui32Count);

    //
    // We processed as much data as we can directly from the receive buffer so
    // we need to return the number of bytes to allow the lower layer to
    // update its read pointer appropriately.
    //
    return(ui32Count);
}

//*****************************************************************************
//
// Handles bulk driver notifications related to the transmit channel (data to
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    //
    // We are not required to do anything in response to any transmit event
    // in this example. All we do is update our transmit counter.
    //
    if(ui32Event == USB_EVENT_TX_COMPLETE)
    {
        g_ui32TxCount += ui32MsgValue;
    }

    //
    // Dump a debug message.
    //
    DEBUG_PRINT("TX complete %d\n", ui32MsgValue);

    return(0);
}

//*****************************************************************************
//
// Handles bulk driver notifications related to the receive channel (data from
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    //
    // Which event are we being sent?
    //
    switch(ui32Event)
    {
        //
        // We are connected to a host and communication is now possible.
        //
        case USB_EVENT_CONNECTED:
        {
            g_bUSBConfigured = true;
            g_pcStatus = "Host connected.";
            g_ui32Flags |= COMMAND_STATUS_UPDATE;

            //
            // Flush our buffers.
            //
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);

            break;
        }

        //
        // The host has disconnected.
        //
        case USB_EVENT_DISCONNECTED:
        {
            g_bUSBConfigured = false;
            g_pcStatus = "Host disconn.";
            g_ui32Flags |= COMMAND_STATUS_UPDATE;
            break;
        }

        //
        // A new packet has been received.
        //
        case USB_EVENT_RX_AVAILABLE:
        {
            tUSBDBulkDevice *psDevice;

            //
            // Get a pointer to our instance data from the callback data
            // parameter.
            //
            psDevice = (tUSBDBulkDevice *)pvCBData;

            //
            // Read the new packet and echo it back to the host.
            //
            return(EchoNewDataToHost(psDevice, pvMsgData, ui32MsgValue));
        }

        //
        // Ignore SUSPEND and RESUME for now.
        //
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
            break;

        //
        // Ignore all other events and return 0.
        //
        default:
            break;
    }

    return(0);
}


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
// The interrupt handler for the first timer interrupt.
//
//*****************************************************************************
void
Timer0IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Use the flags to Toggle the LED for this timer
    //
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, g_ui32Flags);
    g_ui32Flags ^= 1;

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
    // Setup the system clock to run at 50 Mhz from PLL with crystal reference
    // Use "SYSCTL_SYSDIV_2_5" for 80 MHz
    //
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|
                    SYSCTL_OSC_MAIN);

    //
    // Initialize the UART.
    //
    ConfigureUART();

    UARTprintf("\033[2JMy Meccanoid\n");


    //
    // Not configured initially.
    //
    g_bUSBConfigured = false;

    //
    // Enable the GPIO peripheral used for USB, and configure the USB
    // pins.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_GPIOPinTypeUSBAnalog(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    ROM_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    //
    // Erratum workaround for silicon revision A1.  VBUS must have pull-down.
    //
    if(CLASS_IS_TM4C123 && REVISION_IS_A1)
    {
        HWREG(GPIO_PORTB_BASE + GPIO_O_PDR) |= GPIO_PIN_1;
    }

    //
    // Enable the system tick.
    //
    ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / SYSTICKS_PER_SECOND);
    ROM_SysTickIntEnable();
    ROM_SysTickEnable();

    //
    // Initialize the transmit and receive buffers.
    //
    USBBufferInit(&g_sTxBuffer);
    USBBufferInit(&g_sRxBuffer);

    //
    // Initialize the USB stack for device mode.
    //
    USBStackModeSet(0, eUSBModeDevice, 0);

    //
    // Pass our device information to the USB library and place the device
    // on the bus.
    //
    USBDBulkInit(0, &g_sBulkDevice);

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
    // Enable the peripherals used by this example.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Configure the 32-bit periodic timers.
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/10);
    TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER5_BASE, TIMER_A, SysCtlClockGet()/100000);

    //
    // Setup the interrupt for the timer timeout.
    //
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER5A);
    TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);


    //
    // Initialise the Meccano Servos and LEDs
    //
    MeccanoInit();

    //
    // Enable the timers.
    //
    TimerLoadSet(TIMER0_BASE, TIMER_A, 23500);
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerLoadSet(TIMER5_BASE, TIMER_A, 100);
    TimerEnable(TIMER5_BASE, TIMER_A);

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





        communicate();
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
