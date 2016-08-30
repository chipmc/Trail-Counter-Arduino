///
/// @mainpage	Trail-Counter-Arduino
///
/// @details	Trail Traffic Counter - Arduino Side
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Charles McClelland
/// @author		Charles McClelland
/// @date		5/10/16 9:01 AM
/// @version	<#version#>
///
/// @copyright	(c) Charles McClelland, 2016
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
///


///
/// @file		Trail_Counter_Arduino.ino
/// @brief		Main sketch
///
/// @details	<#details#>
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Charles McClelland
/// @author		Charles McClelland
/// @date		5/10/16 9:01 AM
/// @version	<#version#>
///
/// @copyright	(c) Charles McClelland, 2016
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
/// @n
///


// Core library for code-sense - IDE-based
#if defined(WIRING) // Wiring specific
    #include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
    #include "WProgram.h"
#elif defined(ROBOTIS) // Robotis specific
    #include "libpandora_types.h"
    #include "pandora.h"
#elif defined(MPIDE) // chipKIT specific
    #include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
    #include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
    #include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
    #include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
    #include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
    #include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
    #include "Arduino.h"
#elif defined(RFDUINO) // RFduino specific
    #include "Arduino.h"
#elif defined(SPARK) || defined(PARTICLE) // Particle / Spark specific
    #include "application.h"
#elif defined(ESP8266) // ESP8266 specific
    #include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
    #include "Arduino.h"
#else // error
    #   error Platform not defined
#endif // end IDE

// Set parameters
// The SparkFun MMA8452 breakout board defaults to 1, set to 0 if SA0 jumper on the bottom of the board is set
#define SA0 1
#if SA0
#define MMA8452_ADDRESS 0x1D  // SA0 is high, 0x1C if low
#else
#define MMA8452_ADDRESS 0x1C
#endif
// There are some new pin assignments when using the new v9 board
#define V9BOARD 1
#if V9BOARD                    // These are the pin assignments for the v9 board
#define ALARMPIN 3         // This one will be used for the RTC Alarm in v9
#define INT2PIN 2         // This is the interrupt pin that registers taps
#define I2CPIN 5            // This is a pin which connects to the i2c header - future use
#else                      // These are the pin assignments for the v8b board
#define INT1PIN 2         // Not used now but wired for future use
#define INT2PIN 3         // This is the interrupt pin that registers taps
#define ALARMPIN 5         // This is the pin with the RTC Alarm clock - not used on Arduino side
#endif

//Time Period Deinifinitions - used for debugging
#define HOURLYPERIOD hour(t)   // Normally hour(t) but can use minute(t) for debugging
#define DAILYPERIOD day(t) // Normally day(t) but can use minute(t) or hour(t) for debugging

//These defines let me change the memory map without hunting through the whole program
#define VERSIONNUMBER 7      // Increment this number each time the memory map is changed
#define WORDSIZE 8            // For the Word size
#define PAGESIZE 4096         // Memory size in bytes / word size - 256kb FRAM
// First Word - 8 bytes for setting global values
#define DAILYOFFSET 2        // First word of daily counts
#define HOURLYOFFSET 30        // First word of hourly counts (remember we start counts at 1)
#define DAILYCOUNTNUMBER 28    // used in modulo calculations - sets the # of days stored
#define HOURLYCOUNTNUMBER 4064 // used in modulo calculations - sets the # of hours stored - 256k (4096-14-2)
#define VERSIONADDR 0x0       // Memory Locations By Name not Number
#define SENSITIVITYADDR 0x1   // For the 1st Word locations
#define DEBOUNCEADDR 0x2        // Two bytes for debounce
#define DAILYPOINTERADDR 0x4    // One byte for daily pointer
#define HOURLYPOINTERADDR 0x5   // Two bytes for hourly pointer
#define CONTROLREGISTER 0x7     // This is the control register acted on by both Simblee and Arduino
//Second Word - 8 bytes for storing current counts
#define CURRENTHOURLYCOUNTADDR 0x8
#define CURRENTDAILYCOUNTADDR 0xA
#define CURRENTCOUNTSTIME 0xC
//These are the hourly and daily offsets that make up the respective words
#define DAILYDATEOFFSET 1         //Offsets for the value in the daily words
#define DAILYCOUNTOFFSET 2        // Count is a 16-bt value
#define DAILYBATTOFFSET 4
#define HOURLYCOUNTOFFSET 4         // Offsets for the values in the hourly words
#define HOURLYBATTOFFSET 6

// LED Pin Value Variables
#define REDLED 4          // led connected to digital pin 4
#define YELLOWLED 6       // The yellow LED
#define LEDPWR 7          // This pin turns on and off the LEDs


// Include application, user and local libraries
#include "i2c.h"                // not the wire library, can't use pull-ups
#include <avr/sleep.h>          // For Sleep Code
#include "MAX17043.h"           // Drives the LiPo Fuel Gauge
#include <Wire.h>               //http://arduino.cc/en/Reference/Wire (included with Arduino IDE)
#include "DS3232RTC.h"          //http://github.com/JChristensen/DS3232RTC
#include "Time.h"               //http://www.arduino.cc/playground/Code/Time
#include "Adafruit_FRAM_I2C.h"  // Library for FRAM functions
#include "FRAMcommon.h"         // Where I put all the common FRAM read and write extensions


// Prototypes
// Prototypes From the included libraries
MAX17043 batteryMonitor;                      // Init the Fuel Gauge

// Prototypes for i2c functions
byte readRegister(uint8_t address); // Read an i2c device register
void writeRegister(unsigned char address, unsigned char data); // Writes to an i2c device register
void MMA8452Standby(); // Puts the i2c module into standby mode
void MMA8452Active();  // Bring the MMA8452 back on-line
void initMMA8452(byte fsr, byte dataRate);  // Initialize the MMA8452


// Prototypes for General Functions
void StartStopTest(boolean startTest); // Since the test can be started from the serial menu or the Simblee - created a function
void BlinkForever(); // Ends execution
void LogHourlyEvent(time_t LogTime); // Log Hourly Event()
void LogDailyEvent(time_t LogTime); // Log Daily Event()
void CheckForBump(); // Check for bump
void WakeUpNow();      // here the interrupt is handled after wakeup
void sleepNow();  // Puts the Arduino to Sleep
void NonBlockingDelay(int millisDelay);  // Used for a non-blocking delay
int freeRam ();  // Debugging code, to check usage of RAM



// Prototypes for Date and Time Functions
void SetTimeDate(); // Sets the RTC date and time
void PrintTimeDate(); // Prints to the console
void toArduinoTime(time_t unixT); //Converts to Arduino Time for use with the RTC and program


// FRAM and Unix time variables
unsigned int  framAddr;
time_t t;
int lastHour = 0;  // For recording the startup values
int lastDate = 0;
unsigned int hourlyPersonCount = 0;  // hourly counter
unsigned int dailyPersonCount = 0;   //  daily counter
byte currentHourlyPeriod;    // This is where we will know if the period changed
byte currentDailyPeriod;     // We will keep daily counts as well as period counts
int countTemp = 0;          // Will use this to see if we should display a day or hours counts

// Variables for the control byte
// Control Register  (8 - 6 Reserved, 5-Clear Counts, 4-Toggle LEDs, 3-Start / Stop Test, 2-Set Sensitivity, 1-Set Delay)
byte signalDebounceChange = B00000001;
byte clearDebounceChange = B11111110;
byte signalSentitivityChange = B00000010;
byte clearSensitivityChange = B11111101;
byte toggleStartStop = B00000100;
byte toggleLEDs = B00001000;
byte signalClearCounts = B00010000;
byte clearClearCounts = B11101111;
byte controlRegisterValue;
byte oldControlRegisterValue;
unsigned long lastCheckedControlRegister;
int controlRegisterDelay = 1000;


// Accelerometer Variables
const byte accelFullScaleRange = 2;  // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
const byte dataRate = 5;             // output data rate - 0=800Hz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56
byte accelInputValue = 1;            // Raw sensitivity input (0-9);
byte accelSensitivity;               // Hex variable for sensitivity
byte accelThreshold = 100;           // accelThreshold value to decide when the detected sound is a knock or not
unsigned int debounce;               // This is a minimum debounce value - additional debounce set using pot or remote terminal
volatile byte source;

// Battery monitor
float stateOfCharge = 0;

//Menu and Program Variables
unsigned long lastBump = 0;    // set the time of an event
int ledState = LOW;            // variable used to store the last LED status, to toggle the light
int delaySleep = 3000;         // Wait until going back to sleep so we can enter commands
int menuChoice=0;              // Menu Selection
boolean refreshMenu = true;       //  Tells whether to write the menu
boolean inTest = false;            // Are we in a test or not
boolean LEDSon = false;             // Are the LEDs on or off
int numberHourlyDataPoints;   // How many hourly counts are there
int numberDailyDataPoints;   // How many daily counts are there

// Add setup code
void setup()
{
    Wire.begin();
    Serial.begin(9600);                   // Note: Required baud for Bluetooth UART Friend
    Serial.println(F("Trail-Counter-Arduino"));
    pinMode(REDLED, OUTPUT);              // declare the Red LED Pin as an output
    pinMode(YELLOWLED, OUTPUT);           // declare the Yellow LED Pin as as OUTPUT
    pinMode(LEDPWR, OUTPUT);            // declare the Bluetooth Dongle Power pin as as OUTPUT
    digitalWrite(LEDPWR, LOW);          // Turn on the power to the LEDs
    pinMode(INT2PIN, INPUT);            // Set up the interrupt pins, they're set as active low with an external pull-up
    
    
    //  Arduino takes the bus for this entire section...
    TakeTheBus(); // Need th i2c bus for initializations
        batteryMonitor.reset();               // Initialize the battery monitor
        batteryMonitor.quickStart();
        setSyncProvider(RTC.get);
        Serial.println(F("RTC Sync"));
        if (timeStatus() != timeSet) {
            Serial.println(F(" time sync fail!"));
            BlinkForever();
        }
        enable32Khz(1); // turns on the 32k squarewave - need to test effect on power
    
        if (fram.begin()) {  // you can stick the new i2c addr in here, e.g. begin(0x51);
            Serial.println(F("Found I2C FRAM"));
        } else {
            Serial.println(F("No I2C FRAM found ... check your connections"));
            BlinkForever();
        }
    
    
        // We need to set an Alarm or Two in order to ensure that the Simblee is put to sleep at night
        RTC.squareWave(SQWAVE_NONE);            //Disable the default square wave of the SQW pin.
        RTC.alarm(ALARM_1);                     // This will clear the Alarm flags
        RTC.alarm(ALARM_2);                     // This will clear the Alarm flags
        RTC.setAlarm(ALM1_MATCH_HOURS,00,00,22,0); // Set the evening Alarm
        // RTC.setAlarm(ALM2_EVERY_MINUTE,0x00,0x00,0x00); // Set the alarm to go off every minute for testing
        RTC.setAlarm(ALM2_MATCH_HOURS,00,00,6,0); // Set the moringin Alarm
        RTC.alarmInterrupt(ALARM_2, true);      // Connect the Interrupt to the Alarms (or not)
        RTC.alarmInterrupt(ALARM_1, true);
    GiveUpTheBus(); // Done with i2c initializations Arduino gives up the bus here.
    
    
    if (FRAMread8(VERSIONADDR) != VERSIONNUMBER) {  // Check to see if the memory map in the sketch matches the data on the chip
        Serial.print(F("FRAM Version Number: "));
        Serial.println(FRAMread8(VERSIONADDR));
        Serial.read();
        Serial.println(F("Memory/Sketch mismatch! Erase FRAM? (Y/N)"));
        while (!Serial.available());
        switch (Serial.read()) {    // Give option to erase and reset memory
            case 'Y':
                ResetFRAM();
                break;
            case 'y':
                ResetFRAM();
                break;
            default:
                Serial.println(F("Cannot proceed"));
                BlinkForever();
        }
    }
    
    // Import the accelSensitivity and Debounce values from memory
    Serial.print(F("Sensitivity set to: "));
    accelSensitivity = FRAMread8(SENSITIVITYADDR);
    Serial.println(accelSensitivity);
    Serial.print(F("Debounce set to: "));
    debounce = FRAMread16(DEBOUNCEADDR);
    Serial.println(debounce);
    
    FRAMwrite8(CONTROLREGISTER, B00000000);       // Reset the control register
    
    TakeTheBus();  // Need to initialize the accelerometer
        // Read the WHO_AM_I register of the Accelerometer, this is a good test of communication
        byte c = readRegister(0x0D);  // Read WHO_AM_I register
        if (c == 0x2A) // WHO_AM_I should always be 0x2A
        {
            initMMA8452(accelFullScaleRange, dataRate);  // init the accelerometer if communication is OK
            Serial.println(F("MMA8452Q is online..."));
        }
        else
        {
            Serial.print(F("Could not connect to MMA8452Q: 0x"));
            Serial.println(c, HEX);
            BlinkForever() ; // Loop forever if communication doesn't happen
        }
    GiveUpTheBus(); // Done!
    
    Serial.print(F("Free memory: "));
    Serial.println(freeRam());
    
    
}

// Add loop code
void loop()
{
    if (refreshMenu) {
        refreshMenu = 0;
        Serial.println(F("Remote Trail Counter Program Menu"));
        Serial.println(F("0 - Display Menu"));
        Serial.println(F("1 - Display status"));
        Serial.println(F("2 - Set the clock"));
        Serial.println(F("3 - Change the sensitivitiy"));
        Serial.println(F("4 - Change the debounce"));
        Serial.println(F("5 - Reset the counter"));
        Serial.println(F("6 - Reset the memory"));
        Serial.println(F("7 - Start / stop counting"));
        Serial.println(F("8 - Dump hourly counts"));
        Serial.println(F("9 - Last 14 day's counts"));
        delay(100);
    }
    
    if (Serial.available() >> 0) {      // Only enter if there is serial data in the buffer
        switch (Serial.read()) {          // Read the buffer
            case '0':
                refreshMenu = 1;
                break;
            case '1':   // Display Current Status Information
                Serial.print(F("Current Time:"));
                PrintTimeDate();  // Give and take the bus are in this function as it gets the current time
                TakeTheBus();
                    stateOfCharge = batteryMonitor.getSoC();
                GiveUpTheBus();
                Serial.print(F("State of charge: "));
                Serial.print(stateOfCharge);
                Serial.println("%");
                Serial.print(F("Sensitivity set to: "));
                Serial.println(FRAMread8(SENSITIVITYADDR));
                Serial.print(F("Debounce set to: "));
                Serial.println(FRAMread16(DEBOUNCEADDR));
                Serial.print(F("Hourly count: "));
                Serial.println(FRAMread16(CURRENTHOURLYCOUNTADDR));
                Serial.print(F("Daily count: "));
                Serial.println(FRAMread16(CURRENTDAILYCOUNTADDR));
                Serial.print(F("Free memory: "));
                Serial.println(freeRam());
                break;
            case '2':     // Set the clock
                SetTimeDate();
                PrintTimeDate();
                Serial.println(F("Date and Time Set"));
                break;
            case '3':  // Change the sensitivity
                Serial.println(F("Enter 0 (most) to 16 (least)"));
                while (Serial.available() == 0) {  // Look for char in serial queue and process if found
                    continue;
                }
                accelInputValue = (Serial.parseInt());
                accelSensitivity = byte(accelInputValue);
                Serial.print(F("accelSensitivity set to: "));
                Serial.println(accelInputValue);
                FRAMwrite8(SENSITIVITYADDR, accelSensitivity);
                TakeTheBus();
                    initMMA8452(accelFullScaleRange, dataRate);  // init the accelerometer if communication is OK
                GiveUpTheBus();
                Serial.println(F("MMA8452Q is online..."));
                break;
            case '4':  // Change the debounce value
                Serial.println(F("Enter 0 to 9"));
                while (Serial.available() == 0) {  // Look for char in serial queue and process if found
                    continue;
                }
                debounce = 50*(Serial.parseInt());
                Serial.print(F("Debounce set to: "));
                Serial.println(debounce);
                FRAMwrite16(DEBOUNCEADDR, debounce);
                break;
            case '5':  // Reset the current counters
                Serial.println(F("Counter Reset!"));
                FRAMwrite16(CURRENTDAILYCOUNTADDR, 0);   // Reset Daily Count in memory
                FRAMwrite16(CURRENTHOURLYCOUNTADDR, 0);  // Reset Hourly Count in memory
                hourlyPersonCount = 0;
                dailyPersonCount = 0;
                break;
            case '6': // Reset FRAM Memory
                ResetFRAM();
                break;
            case '7':  // Start or stop the test
                if (inTest == 0) {
                    FRAMwrite8(CONTROLREGISTER, toggleStartStop | controlRegisterValue);    // Toggle the start stop bit high
                    StartStopTest(1);
                }
                else {
                    FRAMwrite8(CONTROLREGISTER, toggleStartStop ^ controlRegisterValue);    // Toggle the start stop bit low
                    StartStopTest(0);
                    refreshMenu = 1;
                }
                break;
            case '8':   // Dump the hourly data to the monitor
                numberHourlyDataPoints = FRAMread16(HOURLYPOINTERADDR); // Put this here to reduce FRAM reads
                Serial.println(F("Hour Ending -   Count  - Battery %"));
                for (int i=0; i < HOURLYCOUNTNUMBER; i++) { // Will walk through the hourly count memory spots - remember pointer is already incremented
                    unsigned int address = (HOURLYOFFSET + (numberHourlyDataPoints + i) % HOURLYCOUNTNUMBER)*WORDSIZE;
                    countTemp = FRAMread16(address+HOURLYCOUNTOFFSET);
                    if (countTemp > 0) {
                        time_t unixTime = FRAMread32(address);
                        toArduinoTime(unixTime);
                        Serial.print(F(" - "));
                        Serial.print(countTemp);
                        Serial.print(F("  -  "));
                        Serial.print(FRAMread8(address+HOURLYBATTOFFSET));
                        Serial.println(F("%"));
                    }
                }
                Serial.println(F("Done"));
                break;
            case '9':  // Download all the daily counts
                numberDailyDataPoints = FRAMread8(DAILYPOINTERADDR);        // Put this here to reduce FRAM reads
                Serial.println(F("Date - Count - Battery %"));
                for (int i=0; i < DAILYCOUNTNUMBER; i++) {                  // Will walk through the 30 daily count memory spots - remember pointer is already incremented
                    int address = (DAILYOFFSET + (numberDailyDataPoints + i) % DAILYCOUNTNUMBER)*WORDSIZE;      // Here to improve readabiliy - with Wrapping
                    countTemp = FRAMread16(address+DAILYCOUNTOFFSET);       // This, again, reduces FRAM reads
                    if (countTemp > 0) {                                    // Since we will step through all 30 - don't print empty results
                        Serial.print(FRAMread8(address));
                        Serial.print(F("/"));
                        Serial.print(FRAMread8(address+DAILYDATEOFFSET));
                        Serial.print(F(" - "));
                        Serial.print(countTemp);
                        Serial.print(F("  -  "));
                        Serial.print(FRAMread8(address+DAILYBATTOFFSET));
                        Serial.println(F("%"));
                    }
                }
                Serial.println(F("Done"));
                break;
            default:
                Serial.println(F("Invalid choice - try again"));
        }
        Serial.read();  // Clear the serial buffer
    }
    if (inTest == 1) {
        CheckForBump();
        if (millis() >= lastBump + delaySleep) {
            Serial.println(F("Serial: Entering Sleep mode"));
            delay(100);     // this delay is needed, the sleep function will provoke a Serial error otherwise!!
            sleepNow();     // sleep function called here
        }
    }
    if (millis() >= lastCheckedControlRegister + controlRegisterDelay) {
        TakeTheBus();
            boolean alarmInt1 = RTC.alarm(ALARM_1);
            boolean alarmInt2 = RTC.alarm(ALARM_2);
        GiveUpTheBus();
        if (alarmInt1 || alarmInt2) {
            Serial.print("Alarm Flag Set - ");
            PrintTimeDate();  // Give and take the bus are in this function as it gets the current time
        }
        controlRegisterValue = FRAMread8(CONTROLREGISTER);
        if (controlRegisterValue != oldControlRegisterValue) {       // debugging code 
            Serial.print(F("ControlRegisterValue = "));
            Serial.println(controlRegisterValue);
            oldControlRegisterValue = controlRegisterValue;
        }
        lastCheckedControlRegister = millis();
        if ((controlRegisterValue & toggleStartStop) >> 2 && !inTest)
        {
            StartStopTest(1);  // If the control says start but we are stopped
            Serial.println(F("Starting the test"));
        }
        else if (!((controlRegisterValue & toggleStartStop) >> 2) && inTest)
        {
            StartStopTest(0); // If the control bit says stop but we have started
            Serial.println(F("Stopping the test"));
        }
        else if (controlRegisterValue & signalDebounceChange)   // If we changed the debounce value on the Simblee side
        {
            debounce = FRAMread16(DEBOUNCEADDR);
            Serial.print(F("Updated debounce value to:"));
            Serial.println(debounce);
            controlRegisterValue &= clearDebounceChange;
            FRAMwrite8(CONTROLREGISTER, controlRegisterValue);
            Serial.print(F("Debounce Updated Control Register Value ="));
            Serial.println(controlRegisterValue);
        }
        else if (controlRegisterValue & signalSentitivityChange)   // If we changed the debounce value on the Simblee side
        {
            accelSensitivity = FRAMread8(SENSITIVITYADDR);
            TakeTheBus();
                initMMA8452(accelFullScaleRange, dataRate);  // init the accelerometer if communication is OK
            GiveUpTheBus();
            Serial.println(F("MMA8452Q is online..."));
            Serial.print(F("Updated sensitivity value to:"));
            Serial.println(accelSensitivity);
            controlRegisterValue &= clearSensitivityChange;
            FRAMwrite8(CONTROLREGISTER, controlRegisterValue);
            Serial.print(F("Sensitivty Updated Control Register Value ="));
            Serial.println(controlRegisterValue);
        }
        else if (controlRegisterValue & signalClearCounts)
        {
            controlRegisterValue &= clearClearCounts;
            hourlyPersonCount = 0;
            dailyPersonCount = 0;
            Serial.println(F("Current Counts Cleared as Ordered"));
        }
        else if (((controlRegisterValue & toggleLEDs) >> 3) && !LEDSon)
        {
            digitalWrite(LEDPWR,LOW);
            LEDSon = true; // This keeps us from entering this conditional until there is a change
            Serial.println(F("Turn on the LEDs"));
        }
        else if (!((controlRegisterValue & toggleLEDs) >> 3) && LEDSon)
        {
            digitalWrite(LEDPWR,HIGH);
            LEDSon = false; // This keeps us from entering this conditional until there is a change
            Serial.println(F("Turn off the LEDs"));
        }
    }
}


void CheckForBump() // This is where we check to see if an interrupt is set when not asleep
{
    if (digitalRead(INT2PIN)==0)    // If int2 goes HIGH, either p/l has changed or there's been a single/double tap
    {
        if ((source & 0x08)==0x08) { // We are only interested in the TAP register so read that
            if (millis() >= lastBump + debounce) {
                TakeTheBus();
                    t = RTC.get();
                GiveUpTheBus();
                if (HOURLYPERIOD != currentHourlyPeriod) {
                    Serial.print(F(" Hour: "));
                    Serial.print(currentHourlyPeriod);
                    Serial.print(F(" - count: "));
                    Serial.println(hourlyPersonCount);
                    LogHourlyEvent(t);
                    if (currentHourlyPeriod >= 19 ||  currentHourlyPeriod <= 6) {
                        // Turn things off at night
                    }
                    else {
                        // Turn things  on during the day
                    }
                }
                if (DAILYPERIOD != currentDailyPeriod) {
                    Serial.print(F("Day: "));
                    Serial.print(currentDailyPeriod);
                    Serial.print(F(" - count: "));
                    Serial.println(dailyPersonCount);
                    LogDailyEvent(t);
                }
                hourlyPersonCount++;                    // Increment the PersonCount
                FRAMwrite16(CURRENTHOURLYCOUNTADDR, hourlyPersonCount);  // Load Hourly Count to memory
                dailyPersonCount++;                    // Increment the PersonCount
                FRAMwrite16(CURRENTDAILYCOUNTADDR, dailyPersonCount);   // Load Daily Count to memory
                FRAMwrite32(CURRENTCOUNTSTIME, t);   // Write to FRAM - this is so we know when the last counts were saved
                lastBump = millis();              // Reset last bump timer
                Serial.print(F("Hourly: "));
                Serial.print(hourlyPersonCount);
                Serial.print(F(" Daily: "));
                Serial.print(dailyPersonCount);
                Serial.print(F("  Time: "));
                PrintTimeDate();
                ledState = !ledState;              // toggle the status of the LEDPIN:
                digitalWrite(REDLED, ledState);    // update the LED pin itself
            }
            readRegister(0x22);  // Reads the PULSE_SRC register to reset it
        }
    }
}

void StartStopTest(boolean startTest)  // Since the test can be started from the serial menu or the Simblee - created a function
{
    tmElements_t tm;
    if (startTest) {
        inTest = true;
        Serial.print(F("Starting Test - CONTROLREGISTER = "));
        Serial.println(FRAMread8(CONTROLREGISTER));
        TakeTheBus();
            t = RTC.get();                    // Gets the current time
        GiveUpTheBus();
        currentHourlyPeriod = HOURLYPERIOD;   // Sets the hour period for when the count starts (see #defines)
        currentDailyPeriod = DAILYPERIOD;     // And the day  (see #defines)
        // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over
        time_t unixTime = FRAMread32(CURRENTCOUNTSTIME);
        breakTime(unixTime, tm);
        lastHour = tm.Hour;
        lastDate = tm.Day;
        Serial.println("Restoring Counts");
        dailyPersonCount = FRAMread16(CURRENTDAILYCOUNTADDR);  // Load Daily Count from memory
        hourlyPersonCount = FRAMread16(CURRENTHOURLYCOUNTADDR);  // Load Hourly Count from memory
        if (currentDailyPeriod != lastDate) {
            LogHourlyEvent(t);
            LogDailyEvent(t);
        }
        else if (currentHourlyPeriod != lastHour) {
            LogHourlyEvent(t);
        }
        TakeTheBus();
            source = readRegister(0x22);     // Reads the PULSE_SRC register to reset it
        GiveUpTheBus();
        attachInterrupt(digitalPinToInterrupt(INT2PIN), WakeUpNow, LOW);   // use interrupt and run wakeUpNow when pin 3 goes LOW
        Serial.println(F("Test Started"));
    }
    else {
        inTest = false;
        Serial.print(F("Stopping Test since CONTROLREGISTER is:"));
        Serial.println(FRAMread8(CONTROLREGISTER));
        TakeTheBus();
            source = readRegister(0x22);  // Reads the PULSE_SRC register to reset it
            detachInterrupt(digitalPinToInterrupt(INT2PIN));           // disables interrupt so the program can execute
            t = RTC.get();
        GiveUpTheBus();
        FRAMwrite16(CURRENTDAILYCOUNTADDR, dailyPersonCount);   // Load Daily Count to memory
        FRAMwrite16(CURRENTHOURLYCOUNTADDR, hourlyPersonCount);  // Load Hourly Count to memory
        FRAMwrite32(CURRENTCOUNTSTIME, t);   // Write to FRAM - this is so we know when the last counts were saved
        hourlyPersonCount = 0;        // Reset Person Count
        dailyPersonCount = 0;         // Reset Person Count
        Serial.println(F("Test Stopped"));
    }
}

void LogHourlyEvent(time_t LogTime) // Log Hourly Event()
{
    Serial.print("Logging an Hourly Event - ");
    unsigned int pointer = (HOURLYOFFSET + FRAMread16(HOURLYPOINTERADDR))*WORDSIZE;  // get the pointer from memory and add the offset
    tmElements_t timeElement;
    breakTime(LogTime, timeElement);
    LogTime = LogTime - 3600 -60*timeElement.Minute - timeElement.Second;       // So we need to back out the last hour to log the right one
    FRAMwrite32(pointer, LogTime);   // Write to FRAM - this is the end of the period
    FRAMwrite16(pointer+HOURLYCOUNTOFFSET,hourlyPersonCount);
    TakeTheBus();
        stateOfCharge = batteryMonitor.getSoC();
    GiveUpTheBus();
    FRAMwrite8(pointer+HOURLYBATTOFFSET,stateOfCharge);
    unsigned int newHourlyPointerAddr = (FRAMread16(HOURLYPOINTERADDR)+1) % HOURLYCOUNTNUMBER;  // This is where we "wrap" the count to stay in our memory space
    FRAMwrite16(HOURLYPOINTERADDR,newHourlyPointerAddr);
    hourlyPersonCount = 0;                    // Reset and increment the Person Count in the new period
    currentHourlyPeriod = HOURLYPERIOD;  // Change the time period
}


void LogDailyEvent(time_t LogTime) // Log Daily Event()
{
    Serial.print("Logging a Daily Event - ");
    int pointer = (DAILYOFFSET + FRAMread8(DAILYPOINTERADDR))*WORDSIZE;  // get the pointer from memory and add the offset
    tmElements_t timeElement;
    breakTime(LogTime, timeElement);
    LogTime = LogTime - 86400L- 3600*timeElement.Hour -60*timeElement.Minute - timeElement.Second;  // Logging for the previous day
    FRAMwrite8(pointer,month(LogTime)); // should be time.month
    FRAMwrite8(pointer+DAILYDATEOFFSET,day(LogTime));  // Write to FRAM - this is the end of the period  - should be time.date
    FRAMwrite16(pointer+DAILYCOUNTOFFSET,dailyPersonCount);
    TakeTheBus();
        stateOfCharge = batteryMonitor.getSoC();
    GiveUpTheBus();
    FRAMwrite8(pointer+DAILYBATTOFFSET,stateOfCharge);
    byte newDailyPointerAddr = (FRAMread8(DAILYPOINTERADDR)+1) % DAILYCOUNTNUMBER;  // This is where we "wrap" the count to stay in our memory space
    FRAMwrite8(DAILYPOINTERADDR,newDailyPointerAddr);
    dailyPersonCount = 0;    // Reset and increment the Person Count in the new period
    currentDailyPeriod = DAILYPERIOD;  // Change the time period
}


void SetTimeDate()  // Function to set the date and time from the terminal window
{
    tmElements_t tm;
    Serial.println(F("Enter Seconds (0-59): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Second = Serial.parseInt();
    Serial.println(F("Enter Minutes (0-59): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Minute = Serial.parseInt();
    Serial.println(F("Enter Hours (0-23): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Hour= Serial.parseInt();
    Serial.println(F("Enter Day of the Month (1-31): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Day = Serial.parseInt();
    Serial.println(F("Enter the Month (1-12): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Month = Serial.parseInt();
    Serial.println(F("Enter the Year (0-99): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Year = CalendarYrToTm(Serial.parseInt());
    
    t= makeTime(tm);

    
    TakeTheBus();
        RTC.set(t);             //use the time_t value to ensure correct weekday is set
        setTime(t);
    GiveUpTheBus();
    
}

void PrintTimeDate()  // Prints time and date to the console
{
    TakeTheBus();
        t = RTC.get();
    GiveUpTheBus();
    Serial.print(year(t), DEC);
    Serial.print('/');
    Serial.print(month(t), DEC);
    Serial.print('/');
    Serial.print(day(t), DEC);
    Serial.print(F(" "));
    Serial.print(hour(t), DEC);
    Serial.print(':');
    if (minute(t) < 10) Serial.print("0");
    Serial.print(minute(t), DEC);
    Serial.print(':');
    if (second(t) < 10) Serial.print("0");
    Serial.print(second(t), DEC);
    Serial.println();
}


void initMMA8452(byte fsr, byte dataRate)   // Initialize the MMA8452 registers
{
    // See the many application notes for more info on setting all of these registers:
    // http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8452Q
    // Feel free to modify any values, these are settings that work well for me.
    MMA8452Standby();  // Must be in standby to change registers
    
    // Set up the full scale range to 2, 4, or 8g.
    if ((fsr==2)||(fsr==4)||(fsr==8))
        writeRegister(0x0E, fsr >> 2);
    else
        writeRegister(0x0E, 0);
    
    // Setup the 3 data rate bits, from 0 to 7
    writeRegister(0x2A, readRegister(0x2A) & ~(0x38));
    if (dataRate <= 7)
        writeRegister(0x2A, readRegister(0x2A) | (dataRate << 3));
    
    /* Set up single and double tap - 5 steps:
     1. Set up single and/or double tap detection on each axis individually.
     2. Set the accelThreshold - minimum required acceleration to cause a tap.
     3. Set the time limit - the maximum time that a tap can be above the accelThreshold
     4. Set the pulse latency - the minimum required time between one pulse and the next
     5. Set the second pulse window - maximum allowed time between end of latency and start of second pulse
     for more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf */
    //writeRegister(0x21, 0x7F);  // 1. enable single/double taps on all axes
    writeRegister(0x21, 0x55);  // 1. single taps only on all axes
    // writeRegister(0x21, 0x6A);  // 1. double taps only on all axes
    writeRegister(0x23, accelSensitivity);  // 2. x thresh from 0 to 127, multiply the value by 0.0625g/LSB to get the accelThreshold
    writeRegister(0x24, accelSensitivity);  // 2. y thresh from 0 to 127, multiply the value by 0.0625g/LSB to get the accelThreshold
    writeRegister(0x25, accelSensitivity);  // 2. z thresh from 0 to 127, multiply the value by 0.0625g/LSB to get the accelThreshold
    writeRegister(0x26, 0xFF);  // 3. Max time limit at 100Hz odr, this is very dependent on data rate, see the app note
    writeRegister(0x27, 0x64);  // 4. 1000ms (at 100Hz odr) between taps min, this also depends on the data rate
    writeRegister(0x28, 0xFF);  // 5. 318ms (max value) between taps max
    
    // Set up interrupt 1 and 2
    writeRegister(0x2C, 0x02);  // Active high, push-pull interrupts
    writeRegister(0x2D, 0x19);  // DRDY, P/L and tap ints enabled
    writeRegister(0x2E, 0x01);  // DRDY on INT1, P/L and taps on INT2
    
    MMA8452Active();  // Set to active to start reading
}


void MMA8452Standby()   // Sets the MMA8452 to standby mode while we make register changes
{
    byte c = readRegister(0x2A);
    writeRegister(0x2A, c & ~(0x01));
}


void MMA8452Active()    // Sets the MMA8452 to active mode - once changes made
{
    byte c = readRegister(0x2A);
    writeRegister(0x2A, c | 0x01);
}


byte readRegister(uint8_t address)  // Read a single byte from address and return it as a byte
{
    byte data;
    
    i2cSendStart();
    i2cWaitForComplete();
    
    i2cSendByte((MMA8452_ADDRESS<<1)); // Write 0xB4
    i2cWaitForComplete();
    
    i2cSendByte(address);	// Write register address
    i2cWaitForComplete();
    
    i2cSendStart();
    
    i2cSendByte((MMA8452_ADDRESS<<1)|0x01); // Write 0xB5
    i2cWaitForComplete();
    i2cReceiveByte(TRUE);
    i2cWaitForComplete();
    
    data = i2cGetReceivedByte();	// Get MSB result
    i2cWaitForComplete();
    i2cSendStop();
    
    cbi(TWCR, TWEN);	// Disable TWI
    sbi(TWCR, TWEN);	// Enable TWI
    
    return data;
}


void writeRegister(unsigned char address, unsigned char data)   // Writes a single byte (data) into address
{
    i2cSendStart();
    i2cWaitForComplete();
    
    i2cSendByte((MMA8452_ADDRESS<<1)); // Write 0xB4
    i2cWaitForComplete();
    
    i2cSendByte(address);	// Write register address
    i2cWaitForComplete();
    
    i2cSendByte(data);
    i2cWaitForComplete();
    
    i2cSendStop();
}

void WakeUpNow()        // here the interrupt is handled after wakeup
{
    // execute code here after wake-up before returning to the loop() function
    // timers and code using timers (serial.print and more...) will not work here.
    // we don't really need to execute any special functions here, since we
    // just want the thing to wake up
    // If int2 goes high, either p/l has changed or there's been a single/double tap
    source = readRegister(0x0C);  // Read the interrupt source reg.
}

void sleepNow()         // here we put the arduino to sleep
{
    //   digitalWrite(BLUEFRUITPWR,LOW);   // Turn off the Bluefruit UART
    /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     * there is a list of sleep modes which explains which clocks and
     * wake up sources are available in which sleep mode.
     *
     * In the avr/sleep.h file, the call names of these sleep modes are to be found:
     *
     * The 5 different modes are:
     *     SLEEP_MODE_IDLE         -the least power savings
     *     SLEEP_MODE_ADC
     *     SLEEP_MODE_PWR_SAVE
     *     SLEEP_MODE_STANDBY
     *     SLEEP_MODE_PWR_DOWN     -the most power savings
     *
     * For now, we want as much power savings as possible, so we
     * choose the according
     * sleep mode: SLEEP_MODE_PWR_DOWN
     *
     */
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
    
    sleep_enable();          // enables the sleep bit in the mcucr register
    // so sleep is possible. just a safety pin
    
    /* Now it is time to enable an interrupt. We do it here so an
     * accidentally pushed interrupt button doesn't interrupt
     * our running program. if you want to be able to run
     * interrupt code besides the sleep function, place it in
     * setup() for example.
     *
     * In the function call attachInterrupt(A, B, C)
     * A   can be either 0 or 1 for interrupts on pin 2 or 3.
     *
     * B   Name of a function you want to execute at interrupt for A.
     *
     * C   Trigger mode of the interrupt pin. can be:
     *             LOW        a low level triggers
     *             CHANGE     a change in level triggers
     *             RISING     a rising edge of a level triggers
     *             FALLING    a falling edge of a level triggers
     *
     * In all but the IDLE sleep modes only LOW can be used.
     */
    
    attachInterrupt(digitalPinToInterrupt(INT2PIN),WakeUpNow, LOW); // use interrupt and run function
    
    sleep_mode();            // here the device is actually put to sleep!!
    // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
    
    sleep_disable();         // first thing after waking from sleep:
    // disable sleep...
    detachInterrupt(digitalPinToInterrupt(INT2PIN));      // disables interrupt
    // wakeUpNow code will not be executed
    // during normal running time.
    //    digitalWrite(BLUEFRUITPWR,HIGH);   // Turn on the Bluefruit UART
    
}

void toArduinoTime(time_t unixT) // Puts time in format for reporting
{
    tmElements_t timeElement;
    breakTime(unixT, timeElement);
    Serial.print(timeElement.Month);
    Serial.print(F("/"));
    Serial.print(timeElement.Day);
    Serial.print(F("/"));
    Serial.print(1970+timeElement.Year);
    Serial.print(F(" "));
    Serial.print(timeElement.Hour);
    Serial.print(F(":"));
    if(timeElement.Minute < 10) Serial.print(F("0"));
    Serial.print(timeElement.Minute);
    Serial.print(F(":"));
    if(timeElement.Second < 10) Serial.print(F("0"));
    Serial.print(timeElement.Second);
}


void BlinkForever() // When something goes badly wrong...
{
    Serial.println(F("Error - Reboot"));
    while(1) {
        digitalWrite(REDLED,HIGH);
        delay(200);
        digitalWrite(REDLED,LOW);
        delay(200);
    }
}



void enable32Khz(uint8_t enable)  // Need to turn on the 32k square wave for bus moderation
{
    Wire.beginTransmission(0x68);
    Wire.write(0x0F);
    Wire.endTransmission();
    
    // status register
    Wire.requestFrom(0x68, 1);
    
    uint8_t sreg = Wire.read();
    
    sreg &= ~0b00001000; // Set to 0
    if (enable == true)
        sreg |=  0b00001000; // Enable if required.
    
    Wire.beginTransmission(0x68);
    Wire.write(0x0F);
    Wire.write(sreg);
    Wire.endTransmission();
}

int freeRam ()  // Debugging code, to check usage of RAM
{
    // Example Call: Serial.println(freeRam());
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

