//***********************************************************
//  University: FAMU-FSU College of Engineering
//  Course: EEL-4742L - Advanced Microprocessors Lab
//  Lab Section: 0001
//  Assignment: Lab #10
//  Authors: Guillermo Aznar, Atilla Sulker, Alejandro Chong
//  Instructor: Dr. Reginald Perry
//  TA: Ayodeji Ogundana
//  Semester: Spring 2022
//  Creation Date: 04/18/2022
//  Target Device: MSP432P401R
//  Tool Version: CCS 11.1.0
//***********************************************************

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

#define PERIOD 100
#define DUTY 25
#define CLOCKDIVIDER TIMER_A_CLOCKSOURCE_DIVIDER_48
#define LEFTCHANNEL TIMER_A_CAPTURECOMPARE_REGISTER_4
#define RIGHTCHANNEL TIMER_A_CAPTURECOMPARE_REGISTER_3

typedef enum {motorForward, motorReverse, motorOFF, motorTest} motorStates;
typedef enum {slightLeft, slightRight, hardLeft, hardRight, noSkew, noTrack} skew;
typedef enum {RED, YELLOW, GREEN, CYAN, BLUE, WHITE, OFF} LED2;
typedef enum {standby, tracking, lost} robotModes; // note: lost mode is actually never used
// -> it is treated as a sub-category of "tracking mode"
typedef enum {pressed, unpressed} buttonStates;

buttonStates b0State;

skew trackSkew;

LED2 led2States;

robotModes botMode;

motorStates leftMotorState, rightMotorState;

uint8_t toggleCount = 0;

uint16_t lostCount = 0;

uint8_t data[8];

// Function Prototypes
void config432IO(void);
void configRobotIO(void);
void configPWMTimer(uint16_t, uint16_t, uint16_t, uint16_t);
void RSLKLEDs(motorStates, motorStates);
void motorController(motorStates, motorStates);
void bumperHandler();
void readSensor();
void processData();
void PIDController();
void LED2Driver(LED2);
void powerUp();

Timer_A_PWMConfig timerPWMConfig;

int main(void)
 {
    // Halt WDT:
    MAP_WDT_A_holdTimer();

    // Configure pertinent I/O pins:
    config432IO();
    configRobotIO();

    // Note: motors enabled in powerUp() function (amongst other things):
    powerUp(); // this is used to implement initial power up mode

    // enable global interrupts;
    __enable_interrupts();


    while(1)
    {
        readSensor();
        processData(); // this will determines states for PID controller and LED2

        // determine LED2 state:
        LED2Driver(led2States);

        // determines RSLKLEDs state:
        RSLKLEDs(leftMotorState,rightMotorState);

        // original sampling rate 100 times a second (i.e., 30000)
        // this sample rate is 500 times a second and seems to work best
        __delay_cycles(6000);

        toggleCount++;


        if(botMode == standby)
        {
            // turn wheels off
            leftMotorState = motorOFF;
            rightMotorState = motorOFF;
            motorController(leftMotorState,rightMotorState);

            if(trackSkew != noTrack)
            {

                // toggle count of 50 ensures that LED on blinks at 0.5 Hz frequency
                if(toggleCount == 50)
                {
                    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1,GPIO_PIN0);
                    toggleCount = 0;
                }


            }
            else
            {
                // if in standby mode, sensors don't see the line, turn LED1 OFF
                MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);

            }


        }
        else if(botMode == tracking)
        {
            // turn motor on:
            leftMotorState = motorForward;
            rightMotorState = motorForward;
            motorController(leftMotorState,rightMotorState);

            // call PID Controller function to determine motion as appropriate:
            PIDController(); // note: states for PID controller predetermined in processData() function

            // i.e., lost state
            if(trackSkew == noTrack)
            {
                MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0); // set LED1 low, indicating lost mode

                lostCount++;

                // i.e., 10 sec has elapsed
                if(lostCount == 1000)
                {
                    lostCount = 0;
                    botMode = standby; // go back into standby mode
                }


            }
            else // this is tracking mode here
            {
                MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0); // set LED1 high, indicating tracking mode

                lostCount = 0; // if we ever leave lost mode, reset the 10 s counter
            }

        }

    }


}



/*
config432IO()
This function is used to configure the Launchpad
I/O pins
Input parameters: None
Return value: None
Author: Guillermo Aznar
 */
void config432IO()
{
    // Set RED LED 1 and RGB LEDs are outputs
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2);

    // As a default condition, set all LEDs low
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2);
}


/*
configRobotIO()
This function is used to configure the pertinent I/O
for the RSLK robot
Input parameters: None
Return value: None
Author: Guillermo Aznar
 */

void configRobotIO()
{
    // Configure enable pins, left and right, respectively:
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5,GPIO_PIN2|GPIO_PIN0);

    // Configure direction pins, left and right, respectively:
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5,GPIO_PIN4|GPIO_PIN5);

    // Configure sleep pins, left and right, respectively:
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN7|GPIO_PIN6);

    // Connect these pins to the timer module:
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION); // left PWM
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION); // right PWM

    // Set everything low by default:
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN2|GPIO_PIN0); // disable motors
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN4|GPIO_PIN5); // set direction to forward
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN7|GPIO_PIN6); // put in sleep mode
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN7|GPIO_PIN6); // set PWM low

    // Configure the timers for PWM
    configPWMTimer(PERIOD, CLOCKDIVIDER, DUTY, LEFTCHANNEL);
    configPWMTimer(PERIOD, CLOCKDIVIDER, DUTY, RIGHTCHANNEL);

    // start timer - timer doesn't use interrupts, rather it sends a signal to
    // a particular pin(s)

    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

    // Here, we configure the Robot's LEDs (left, then right):
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P8,GPIO_PIN0|GPIO_PIN5); // front LEDs here
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8,GPIO_PIN0|GPIO_PIN5);

    // rear LEDs here (L & R):
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P8,GPIO_PIN6|GPIO_PIN7);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8,GPIO_PIN6|GPIO_PIN7);

    // Now, we configure the bumper switches:
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4,GPIO_PIN0|GPIO_PIN2|GPIO_PIN3|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7); // configure switches

    MAP_GPIO_enableInterrupt(GPIO_PORT_P4,GPIO_PIN0|GPIO_PIN2|GPIO_PIN3|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7); // enable interrupts
    // select high->low transition
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4,GPIO_PIN0|GPIO_PIN2|GPIO_PIN3|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7,GPIO_HIGH_TO_LOW_TRANSITION);

    // clear the interrupt flags
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4,GPIO_PIN0|GPIO_PIN2|GPIO_PIN3|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    // now, set the callback functions for the interrupts:
    MAP_GPIO_registerInterrupt(GPIO_PORT_P4, bumperHandler);

    // Here, configure the pins for the sensors (these control the IR LEDs):
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN3); // turns on infrared LEDs for even sensors
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_PIN2); // turns on infrared LEDs for odd sensors



}

/*
configPWMTimer()
This function is used to reconfigure the PWM timer
for the given wheel
Input parameters: clockPeriod, uint16_t, period of timer
clockDivider, uint16_t, chosen clock divider
duty, uint16_t, chosen duty cycle
channel, uint16_t, capture compare register
Return value: None
Author: Guillermo Aznar
 */


void configPWMTimer(uint16_t clockPeriod, uint16_t clockDivider, uint16_t duty, uint16_t channel)
{
    const uint32_t TIMER=TIMER_A0_BASE;
    uint16_t dutyCycle = duty*clockPeriod/100;
    timerPWMConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    timerPWMConfig.clockSourceDivider = clockDivider;
    timerPWMConfig.timerPeriod = clockPeriod;
    timerPWMConfig.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_SET;
    timerPWMConfig.compareRegister = channel;
    timerPWMConfig.dutyCycle = dutyCycle;
    MAP_Timer_A_generatePWM(TIMER, &timerPWMConfig);
    MAP_Timer_A_stopTimer(TIMER);

    Timer_A_startCounter(TIMER, TIMER_A_UP_MODE);
}


/*
RSLKLEDs()
This function is used to reconfigure the robot LEDs
based on the direction the robot is moving in
Input parameters: leftMotorState, motorStates, direction of left motor
rightMotorState, motorStates, direction of right motor
Return value: None
Author: Guillermo Aznar
 */


void RSLKLEDs(motorStates leftMotorState, motorStates rightMotorState)
{
    switch(rightMotorState)
    {
        case motorForward:
            // Front Right LED set high
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8,GPIO_PIN5);

            // Back Right LED set Low
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8,GPIO_PIN7);
            break;
        case motorReverse:
            // Back right LED set high
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8,GPIO_PIN7);

            // Front right LED set low
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8,GPIO_PIN5);
            break;
        case motorTest:
            // Turn both right side LEDs high (front and back, respectively)
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8,GPIO_PIN5|GPIO_PIN7);
            break;
        case motorOFF:
            // set both right side LEDs off (front and back, respectively)
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8,GPIO_PIN5|GPIO_PIN7);
            break;
    }

    switch(leftMotorState)
    {
        case motorForward:
            // Front Left LED set high
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8,GPIO_PIN0);

            // Back Left LED set low
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8,GPIO_PIN6);
            break;
        case motorReverse:
            // Back Left LED set high
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8,GPIO_PIN6);

            // Front Left LED set low
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8,GPIO_PIN0);
            break;
        case motorTest:
            // Both Left LEDs set high (front and back, respectively)
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8,GPIO_PIN0|GPIO_PIN6);
            break;
        case motorOFF:
            // Both left LEDs set low (front and back, respectively)
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8,GPIO_PIN0|GPIO_PIN6);
            break;
    }
}


/*
motorController()
This function is used to set the robot's wheel motion in
the appropriate direction
Input parameters: None
Return value: None
Author: Guillermo Aznar
 */

void motorController(motorStates leftMotorState, motorStates rightMotorState)
{
    switch(rightMotorState)
    {
        case motorForward:
            // direction bit set low to enable forward motion
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5);

            // sleep bit set high to allow movement
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);
            break;

        case motorReverse:
            // direction bit set high for backwards motion
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN5);

            // sleep bit set high to allow movement
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);
            break;
        case motorOFF:
            // sleep bit set low to prevent motion
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);
            break;
    }

    switch(leftMotorState)
    {
        case motorForward:
            // direction bit set low forward motion
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4);

            // sleep bit set high to allow motion
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);
            break;

        case motorReverse:
            // direction bit set high for backwards motion
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN4);

            // sleep set high to allow motion
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);
            break;
        case motorOFF:
            // sleep set low to prevent motion
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);
            break;
    }
}


/*
bumperHandler()
This function is used to determine the state of the bumper switches
Input parameters: None
Return value: None
Author: Guillermo Aznar
 */

void bumperHandler()
{
    #define DELAY1S 3000000

    uint16_t status;
    __delay_cycles(30000); // debounce switch here

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);


    if((status & GPIO_PIN0) == GPIO_PIN0)
    {
        if(b0State == pressed)
        {
            b0State = unpressed;

            // if the button is unpressed, re-enter standby mode:
            botMode = standby;
        }
        else
        {
            b0State = pressed;

            // delay for one second:
            __delay_cycles(DELAY1S);

            // if button zero has been pressed, enter tracking mode
            botMode = tracking;
        }
    } // this condition signifies any other button besides button 0 was pressed
    else if(((status & GPIO_PIN0) != GPIO_PIN0) && (status != 0x0000))
    {

        if(botMode == tracking)
        {
            botMode = standby;
        }

    }

}


/*
readSensor()
This function is used to read in the results from each
 of the sensors on the sensor line
Input parameters: None
Return value: None
Author: Guillermo Aznar
 */

void readSensor(){
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3); // turn on infrared LEDs for even sensors
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P9, GPIO_PIN2); // turn on infrared LEDs for odd sensors
    // Set all the sensors as outputs so as to charge the capacitors
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    // Set all the sensors high, allowing us to charge the capacitors associated with each individual sensors
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    // Wait for the capacitor to charge
    __delay_cycles(30); // delay for 10 us
    // Once the capacitor has been charged, set the sensor pins as inputs, removing the voltage that was used to charge
    // them, allowing the capacitors to discharge
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    // Wait briefly for the capacitors to discharge (not fully - to a certain extent, which will allow us to distinguish the terrain color)
    __delay_cycles(3000); // delay for 1ms
    // Read the values at the input pins
    data[0] = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN0);
    data[1] = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN1);
    data[2] = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN2);
    data[3] = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN3);
    data[4] = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN4);
    data[5] = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN5);
    data[6] = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN6);
    data[7] = MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN7);
    // to save power, after we are done reading the sensor, we turn the IR LEDs off
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN2);
}

/*
processData()
This functions is used to process the sensor data, and set
the LEDs to the right color, and set the robot in the correct motion
Input parameters: None
Return value: None
Author: Guillermo Aznar
 */

void processData(){
    volatile uint8_t i;
    uint8_t leftCounter = 0, rightCounter = 0;
    for(i=0; i<8; i++){
        if(i<4){
            if(data[i] == 1)
                rightCounter++;
        }

        else{
            if(data[i] == 1)
                leftCounter++;
        }
    }

    if((leftCounter == 0) && (rightCounter == 0)){
        led2States = WHITE;
        trackSkew = noTrack;


    }
    else if((leftCounter != 0) && (rightCounter == 0)){
        led2States = RED;
        trackSkew = hardLeft;
    }
    else if(((leftCounter == 0) && (rightCounter != 0)) || (rightCounter >= 3)){
        led2States = BLUE;
        trackSkew = hardRight;
    }
    else if(leftCounter > rightCounter){
        led2States = YELLOW;
        trackSkew = slightLeft;
    }
    else if(leftCounter < rightCounter){
        led2States = CYAN;
        trackSkew = slightRight;
    }

    else if(leftCounter == rightCounter){
        led2States = GREEN;
        trackSkew = noSkew;
    }
}



/*
PIDController()
This function is used to reconfigure the direction bits and PWM
timers, based on which direction the robot is to go in
Input parameters: None
Return value: None
Author: Guillermo Aznar
 */

void PIDController()
{

        switch(trackSkew)
        {
        case noSkew:
            // both wheels should be equally energized
            configPWMTimer(PERIOD, CLOCKDIVIDER, DUTY, LEFTCHANNEL);
            configPWMTimer(PERIOD, CLOCKDIVIDER, DUTY, RIGHTCHANNEL);
            // both wheels should be moving forward
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5);
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4);
        break;
        case slightLeft:
            // the right wheel should be energized slightly more
            configPWMTimer(PERIOD, CLOCKDIVIDER, DUTY, LEFTCHANNEL);
            configPWMTimer(PERIOD, CLOCKDIVIDER, DUTY+10, RIGHTCHANNEL);
            // both wheels should be moving forward
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5);
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4);
        break;
        case slightRight:
            // the left wheel should be energized slightly more
            configPWMTimer(PERIOD, CLOCKDIVIDER, DUTY+10, LEFTCHANNEL);
            configPWMTimer(PERIOD, CLOCKDIVIDER, DUTY, RIGHTCHANNEL);
            // both wheels should be moving forward
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5);
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4);
        break;
        case hardLeft:
            // both wheels should be energized equally
            configPWMTimer(PERIOD, CLOCKDIVIDER, DUTY, LEFTCHANNEL);
            configPWMTimer(PERIOD, CLOCKDIVIDER, DUTY, RIGHTCHANNEL);
            // right forward left reverse
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5);
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN4);
        break;
        case hardRight:
            // both wheels should be energized equally
            configPWMTimer(PERIOD, CLOCKDIVIDER, DUTY, LEFTCHANNEL);
            configPWMTimer(PERIOD, CLOCKDIVIDER, DUTY, RIGHTCHANNEL);
            // right reverse left forward
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN5);
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4);
        break;
        case noTrack: // if no track is being picked up by sensors
            // both wheels should be energized equally
            configPWMTimer(PERIOD, CLOCKDIVIDER, DUTY, LEFTCHANNEL);
            configPWMTimer(PERIOD, CLOCKDIVIDER, DUTY, RIGHTCHANNEL);
            // both moving in forward direction
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5);
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4);
        break;

        }

}

/*
powerUp()
This function is used for the powerup phase of the design
Input parameters: None
Return value: None
Author: Guillermo Aznar
 */

void powerUp()
{
   #define DELAY2s 6000000

   // enable motors (we'll use sleep to actually control the motors)
   MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN0|GPIO_PIN2);

   // Ensure sleep is low
   MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN6|GPIO_PIN7);

   // turn off motor and button states:
   leftMotorState = motorOFF;
   rightMotorState = motorOFF;

   // button initialized as unpressed
   b0State = unpressed;
   //bnButtonState = buttonOFF;

   // LEDs initialized as off
   led2States = OFF;

   // robot initialized to be seeing "no track"
   trackSkew = noTrack;

   // start in standby mode after powerup:
   botMode = standby;

   RSLKLEDs(motorTest,motorTest); // turn on all left and right LEDs
   LED2Driver(WHITE); // turn on all RGB LEDs (i.e., creates white via mixing of colors)
   MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);

   //wait 2 seconds:
   __delay_cycles(DELAY2s);

   RSLKLEDs(motorOFF,motorOFF); // turn off all left and right LEDs
   LED2Driver(OFF); // turn off all RGB LEDs
   MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);

   // wait 2 seconds:
   __delay_cycles(DELAY2s);

}

/*
LED2Driver
This function turns on the appropriate MCU pins, based
on the state of LED2
Input parameters: led2State, LED2, typedef enum for LED2 state
Return value: None
Author: Guillermo Aznar
 */
void LED2Driver(LED2 led2States)
{
    switch(led2States)
    {
    case RED:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN0);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN1|GPIO_PIN2);
    break;
    case YELLOW:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN0|GPIO_PIN1);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN2);
    break;
    case GREEN:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN1);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0|GPIO_PIN2);
    break;
    case CYAN:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN1|GPIO_PIN2);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0);
    break;
    case BLUE:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN2);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0|GPIO_PIN1);
    break;
    case WHITE:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2);
    break;
    case OFF:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2);
    break;
    }

}
