#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "utils/uartstdio.h"

#include "Meccano.h"
#include "delay.h"



volatile uint32_t state;
volatile uint32_t mask;

uint8_t meccanoType[3][4];
uint8_t meccanoOutputByte[3][4] = {{0xFE, 0xFE, 0xFE, 0xFE},
								   {0xFE, 0xFE, 0xFE, 0xFE},
								   {0xFE, 0xFE, 0xFE, 0xFE}};

uint8_t meccanoChecksum[3];
uint8_t meccanoTempByte[3];

uint8_t meccanoModuleNum[3] = {0, 0, 0};

volatile bool chargeNewValue;
volatile bool meccanoTimeout;


extern volatile uint32_t delayCounter;
extern uint32_t g_ui32SysClock;

volatile uint32_t g_ui32Flags;



#define HIGH                    1
#define LOW                     0

uint8_t LEDoutputByte1 = 0x03;
uint8_t LEDoutputByte2 = 0x07;

uint8_t moduleNum = 0;
uint8_t inputByte;
uint8_t checkSum;
int ledOrder = 0;

uint8_t moduleType[4];
uint8_t outputByte[4];



//*****************************************************************************
//
// The interrupt handler for the first timer interrupt.
//
//*****************************************************************************
void
Timer0IntHandler(void)
{
	uint8_t outputValue;
    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Treat the Interrupt
    //
    if(state == 0)
    {
    	outputValue = 0xFF;
    	meccanoChecksum[0] = calculateCheckSum(meccanoOutputByte[0][0],
											   meccanoOutputByte[0][1],
											   meccanoOutputByte[0][2],
											   meccanoOutputByte[0][3]);
    }
    else if(state < 5)
    {
    	outputValue = meccanoOutputByte[0][state-1];
    }
    else if(state == 5)
    {
    	outputValue = meccanoChecksum[0];
    }

    if((mask == 0) && (state < 6))
    {
        GPIOPinTypeGPIOOutput(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN);
        GPIOPinWrite(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN, LOW);						// send 0
        TimerLoadSet(TIMER0_BASE, TIMER_A, TIMER_LOADVALUE_417US);
        mask = 0x01;
    }
    else if((mask < 0x100) && (mask > 0) && (state < 6))
    {
    	if (outputValue & mask)
    	{   // if bitwise AND resolves to true
			GPIOPinWrite(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN, GPIO_MECCANO_PIN);  	// send 1
		}else
		{   // if bitwise and resolves to false
			GPIOPinWrite(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN, LOW);               	// send 0
		}
        mask <<= 1;
    }
    else if((mask == 0x100) && (state < 6))
    {
    	GPIOPinWrite(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN, GPIO_MECCANO_PIN);  		// send 1
    	mask <<= 1;
    }
    else if((mask == 0x200) && (state < 6))
    {
    	GPIOPinWrite(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN, GPIO_MECCANO_PIN);  		// send 1
    	mask = 0;
    	state += 1;
    }
    else if((mask == 0) && (state == 6))
    {
		GPIOPinTypeGPIOInput(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN);  			// Init GPIO as input
		mask = 0x01;
		meccanoTimeout = true;
        TimerLoadSet(TIMER0_BASE, TIMER_A, TIMER_LOADVALUE_3000US);
		GPIOIntClear(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN);      // Clear pending interrupts for GPIO
		GPIOIntEnable(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN);     // Enable interrupt for GPIO
    }
    else if((mask < 0x100) && (mask > 0) && (state == 6))
    {
    	if(meccanoTimeout)
		{
			/* Error: no Meccano response */
			mask = 0;
			state = 0;
			chargeNewValue = true;
			GPIOIntDisable(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN);    // Disable interrupt GPIO (in case it was enabled)
			GPIOIntClear(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN);  	// Clear interrupt flag
			TimerLoadSet(TIMER0_BASE, TIMER_A, TIMER_LOADVALUE_10MS);
		}
		else
		{
			if(GPIOPinRead(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN) != LOW)
			{
				meccanoTempByte[0] = meccanoTempByte[0] + mask;
			}
			meccanoTimeout = true;
			TimerLoadSet(TIMER0_BASE, TIMER_A, TIMER_LOADVALUE_1500US);
			GPIOIntClear(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN);      // Clear pending interrupts for GPIO
			GPIOIntEnable(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN);     // Enable interrupt for GPIO
			mask <<= 1;
		}
    }

    if((mask >= 0x100) && (state == 6))
    {
    	// if received back 0xFE, then the module exists so get ID number
    	if (meccanoTempByte[0] == 0xFE)
    	{
    		meccanoOutputByte[0][meccanoModuleNum[0]] = 0xFC;
    	}

    	// if received back 0x01 (module ID is a servo), then change servo color to Blue
		if (meccanoTempByte[0] == 0x01 && meccanoType[0][meccanoModuleNum[0]] == '_')
		{
			meccanoOutputByte[0][meccanoModuleNum[0]] = 0xF4;
			meccanoType[0][meccanoModuleNum[0]] = 'S';
		}

		if (meccanoType[0][meccanoModuleNum[0]] == 'L')
		{
			if(ledOrder == 0)
			{
				meccanoOutputByte[0][meccanoModuleNum[0]] = LEDoutputByte1;
				ledOrder = 1;
			}else
			{
				meccanoOutputByte[0][meccanoModuleNum[0]] = LEDoutputByte2;
				ledOrder = 0;
			}
		}

		// if received back 0x01 (module ID is a LED), then change servo color to Blue
		if (meccanoTempByte[0] == 0x02 && meccanoType[0][meccanoModuleNum[0]] == '_')
		{
			LEDoutputByte1 = 0x04;
			LEDoutputByte2 = 0x47;
			meccanoOutputByte[0][meccanoModuleNum[0]] = LEDoutputByte1;
			ledOrder = 1;
			meccanoType[0][meccanoModuleNum[0]] = 'L';
		}

		if(meccanoTempByte[0] == 0x00)
		{
			int x;
			for(x = meccanoModuleNum[0]; x < 4; x++)
			{
				meccanoOutputByte[0][x] = 0xFE;
				meccanoType[0][x] = '_';
			}
		}

		meccanoModuleNum[0]++;                             // increment to next module ID
		if (meccanoModuleNum[0] > 3)
		{
			meccanoModuleNum[0] = 0;
		}

    	/* Finished */
		state = 0;
		mask = 0;
		chargeNewValue = true;
		GPIOIntDisable(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN);    // Disable interrupt GPIO (in case it was enabled)
		GPIOIntClear(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN);  	// Clear interrupt flag
        TimerLoadSet(TIMER0_BASE, TIMER_A, TIMER_LOADVALUE_10MS);
    }
}


void onMeccanoPinUp(void) {
    if (GPIOIntStatus(GPIO_MECCANO_BASE, false) & GPIO_MECCANO_PIN) {
        // GPIO_MECCANO_PIN was interrupt cause
    	meccanoTimeout = false;
        TimerLoadSet(TIMER0_BASE, TIMER_A, TIMER_LOADVALUE_500US);
        GPIOIntDisable(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN);    // Disable interrupt GPIO (in case it was enabled)
        GPIOIntClear(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN);  	// Clear interrupt flag
    }
}


void MeccanoInit(void){
	state = 0;
	mask = 0;
	chargeNewValue = false;

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_MECCANO_GPIO);
    SysCtlDelay(10);

    //
    // Enable the GPIO pin for the communication with the Meccano Servos and LEDs
    //
    GPIOPinTypeGPIOOutput(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN);
    GPIOPadConfigSet(GPIO_MECCANO_BASE,GPIO_MECCANO_PIN,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD);

    //
    // Initialise the communication pin to low.
    //
    GPIOPinWrite(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN, 0);


    //
    // Enable the peripherals used by this example.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Configure the 32-bit periodic timers.
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/10);

    //
    // Setup the interrupt for the timer timeout.
    //
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Enable the timers.
    //
    TimerLoadSet(TIMER0_BASE, TIMER_A, TIMER_LOADVALUE_10MS);
    TimerEnable(TIMER0_BASE, TIMER_A);

	// GPIO Interrupt setup
	GPIOIntDisable(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN);    // Disable interrupt GPIO (in case it was enabled)
	GPIOIntClear(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN);      // Clear pending interrupts for GPIO
	GPIOIntRegister(GPIO_MECCANO_BASE, onMeccanoPinUp);     // Register our handler function for GPIO port
	GPIOIntTypeSet(GPIO_MECCANO_BASE,
				   GPIO_MECCANO_PIN,
				   GPIO_RISING_EDGE);             			// Configure GPIO for rising edge trigger

    return;
}




//unsigned long pulseIn(int32_t val, unsigned long timeout)
//{
//    // Max supported pulse length is 7 minutes
//    delayCounter = 0;
//    TimerLoadSet(TIMER5_BASE, TIMER_A, SysCtlClockGet()/100000);
//    ROM_TimerEnable(TIMER5_BASE, TIMER_A);
//
//    while (GPIOPinRead(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN) == LOW)
//    {
//        if(delayCounter > timeout)
//        {
//            ROM_TimerDisable(TIMER5_BASE, TIMER_A);
//            return 0;
//        }
//    }
//
//    ROM_TimerDisable(TIMER5_BASE, TIMER_A);
//    delayCounter = 0;
//    ROM_TimerEnable(TIMER5_BASE, TIMER_A);
//    while (GPIOPinRead(GPIO_MECCANO_BASE, GPIO_MECCANO_PIN) != LOW)
//    {
//        if(delayCounter > timeout)
//        {
//            ROM_TimerDisable(TIMER5_BASE, TIMER_A);
//            return 0;
//        }
//    }
//
//    ROM_TimerDisable(TIMER5_BASE, TIMER_A);
//    return delayCounter;
//}

/***** Methods to interact with Smart Modules ******/

/*    setLEDColor(byte red, byte green, byte blue, byte fadetime)  ->  sets the color and transition/fade time of the Meccano Smart LED module
The bytes RED, GREEN and BLUE  should have a value from 0 - 7   =  a total of 512 options.
There are 8 levels of brightness for each color where 0 is OFF and 7 is full brightness.

The byte FADETIME should have a value from 0 - 7.
These are preset time values to transition between the current color to the new color.
The bytes are as such:
0 -  0 seconds (no fade, change immediately)
1 -  200ms   (very very fast fade)
2 -  500ms   (very fast fade)
3 -  800ms  (fast fade)
4 -  1 second (normal fade)
5 -  2 seconds (slow fade)
6  - 3 seconds (very slow fade)
7 -  4 seconds  (very very slow fade)

   end  */

void setLEDColor(uint8_t red, uint8_t green, uint8_t blue, uint8_t fadetime){
    // values from 0-7
    LEDoutputByte1 = 0 ; LEDoutputByte2 = 0;
    LEDoutputByte1 =  0x3F & ( ( (green<<3) & 0x38) | (red & 0x07) );
    LEDoutputByte2 =  0x40 | ( ( (fadetime<<3) & 0x38) | (blue & 0x07) );

}

/*   setServoColor(int servoNum, byte color)  ->  sets the color of the Meccano Smart Servo module
The byte SERVONUM refers to the order of the servo in the chain.  The first servo plugged into your Arduino is 0.
The next servo is 1.  The third servo is 2.   The last servo in a chain of 4 servos is 3.

The byte COLOR is a value  0xF0 - 0xF7 which has 8 possible colors (including all off and all on)
 0xF7  =  R, G,B – all On
 0xF6  =  G, B  - On;   R – Off
 0xF5  =  R, B  - On;   G – Off
 0xF4  =  B  - On;   R, G – Off
 0xF3  =  R,G – On;   B - Off
 0xF2  =  G  - On;   R, B – Off
 0xF1  =  R – On,   G, B – Off
 0xF0  =  R, G,B  – all Off

For example, if you want to set the servo at position 0 to Red and the servo at position 2 to Blue-Green, you would send the following two commands
setServoColor(0,0xF1)
setServoColor(2,0xF6)

  end  */

void setServoColor(uint8_t servoNum, uint8_t color){
    if(moduleType[servoNum] == 'S'){
        outputByte[servoNum] = color;
    }
}


/* setServoPosition(int servoNum, byte pos)  ->   sets a specific servo to a certain position
The byte SERVONUM refers to the order of the servo in the chain.  The first servo plugged into your Arduino is 0.
The next servo is 1.  The third servo is 2.   The last servo in a chain of 4 servos is 3.

The byte POS refers to the servo position  0x00 - 0xEF, which equals to a full 180 degree range.
0x00 = full clockwise
0xEF = full counter clockwise

  end */

void setServoPosition(uint8_t servoNum, uint8_t pos){
    uint8_t servoPos = 0;
    if(moduleType[servoNum] == 'S'){

        if(pos < 0x18){
            servoPos = 0x18;
        }else if(pos > 0xE8){
            servoPos = 0xE8;
        }else{
            servoPos = pos;
        }

    outputByte[servoNum] = servoPos;
    }
}


/* setServotoLIM(int servoNum)  ->   sets a specific servo to LIM mode
The byte SERVONUM refers to the order of the servo in the chain.  The first servo plugged into your Arduino is 0.
The next servo is 1.  The third servo is 2.   The last servo in a chain of 4 servos is 3.

LIM stands for Learned Intelligent Movement.  It is a special mode where the servo IC stops driving the motor and just sends back the position of the servo.

  end */


void setServotoLIM(uint8_t servoNum){
    if(moduleType[servoNum] == 'S'){
        outputByte[servoNum] = 0xFA;
    }

}


/* getServoPosition(int servoNum)  ->   returns a byte that is the position of a specific servo
The byte SERVONUM refers to the order of the servo in the chain.  The first servo plugged into your Arduino is 0.
The next servo is 1.  The third servo is 2.   The last servo in a chain of 4 servos is 3.

First you must set the specific servo to LIM mode.  Then use this command.
The returned byte POS is the servo's position  0x00 - 0xEF, which equals to a full 180 degree range.
0x00 = full clockwise
0xEF = full counter clockwise

  end */

uint8_t getServoPosition(uint8_t servoNum){
    int temp = 0;
    if(moduleType[servoNum] == 'S'){
        if (meccanoModuleNum[0] > 0){
            temp = meccanoModuleNum[0] - 1;
        }
        else{
            temp = 0;
        }
        if(temp == servoNum){
            return inputByte;
        }
    }
    return 0x00;
}



/*    communicate()  -  this is the main method that takes care of initializing, sending data to and receiving data from Meccano Smart modules

  The datastream consists of 6 output bytes sent to the Smart modules and one return input byte from the Smart modules.
  Since there can be a maximum of 4 Smart modules in a chain, each module takes turns replying along the single data wire.

  Output bytes:
      0xFF - the first byte is always a header byte of 0xFF
      Data 1 -  the second byte is the data for the Smart module at the 1st position in the chain
      Data 2 -  the third byte is the data for the Smart module at the 2nd position in the chain
      Data 3 -  the fourth byte is the data for the Smart module at the 3rd position in the chain
      Data 4 -  the fifth byte is the data for the Smart module at the 4th position in the chain
      Checksum  -  the sixth byte is part checksum, part module ID.  The module ID tells which of the modules in the chain should reply
   end  */


uint8_t calculateCheckSum(uint8_t Data1, uint8_t Data2, uint8_t Data3, uint8_t Data4){
    int CS;
    CS =  Data1 + Data2 + Data3 + Data4;  // ignore overflow
    CS = CS + (CS >> 8);                  // right shift 8 places
    CS = CS + (CS << 4);                  // left shift 4 places
    CS = CS & 0xF0;                     // mask off top nibble
    CS = CS | meccanoModuleNum[0];
    return CS;
}



