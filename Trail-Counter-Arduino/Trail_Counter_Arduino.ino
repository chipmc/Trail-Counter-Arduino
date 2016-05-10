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

//Time Period Deinifinitions - used for debugging
#define HOURLYPERIOD t.minute()   // Normally t.hour() but can use t.minute() for debugging
#define DAILYPERIOD t.hour() // Normally t.date() but can use t.minute() or t.hour() for debugging

//These defines let me change the memory map without hunting through the whole program
#define VERSIONNUMBER 4       // Increment this number each time the memory map is changed
#define WORDSIZE 8            // For the Word size
#define PAGESIZE 4096         // Memory size in bytes / word size - 256kb FRAM
// First Word - 8 bytes for setting global values
#define DAILYOFFSET 2        // First word of daily counts
#define HOURLYOFFSET 16        // First word of hourly counts (remember we start counts at 1)
#define DAILYCOUNTNUMBER 14    // used in modulo calculations - sets the # of days stored
#define HOURLYCOUNTNUMBER 4078 // used in modulo calculations - sets the # of hours stored - 256k (4096-14-2)
#define VERSIONADDR 0x0       // Memory Locations By Name not Number
#define SENSITIVITYADDR 0x1   // For the 1st Word locations
#define DEBOUNCEADDR 0x2
#define DAILYPOINTERADDR 0x3
#define HOURLYPOINTERADDR 0x4
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

// Pin Value Variables
#define INT1PIN 2         // Not used now but wired for future use
#define INT2PIN 3         // This is the interrupt pin that registers taps
#define REDLED 4          // led connected to digital pin 4
#define YELLOWLED 6       // The yellow LED
#define LEDPWR 7          // This pin turns on and off the LEDs

// Include application, user and local libraries
#include "i2c.h"                // not the wire library, can't use pull-ups
#include <avr/sleep.h>          // For Sleep Code
#include "MAX17043.h"           // Drives the LiPo Fuel Gauge
#include "Wire.h"               // For the LiPo Fuel Gauge
#include "RTClib.h"             // Adafruit's library which includes the DS3231
#include "Time.h"             // This library brings Unix Time capabilities
#include "Adafruit_FRAM_I2C.h"  // Library for FRAM functions


// Prototypes
// Prototypes From the included libraries
RTC_DS3231 rtc;                               // Init the DS3231
MAX17043 batteryMonitor;                      // Init the Fuel Gauge
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C(); // Init the FRAM

// Prototypes From my functions
// Prototypes for FRAM Functions
unsigned long FRAMread32(unsigned long address); // Reads a 32 bit word
void FRAMwrite32(int address, unsigned long value);  // Writes a 32-bit word
int FRAMread16(unsigned int address); // Reads a 16 bit word
void FRAMwrite16(unsigned int address, int value); //Writes a 32-bit word
uint8_t FRAMread8(unsigned int address);  // Reads a 8 bit word
void FRAMwrite8(unsigned int address, uint8_t value); //Writes a 32-bit word
void ResetFRAM();  // This will reset the FRAM - set the version and preserve delay and sensitivity

// Prototypes for i2c functions
boolean GiveUpTheBus(); // Give up the i2c bus
boolean TakeTheBus(); // Take the 12c bus
byte readRegister(uint8_t address); // Read an i2c device register
void writeRegister(unsigned char address, unsigned char data); // Writes to an i2c device register
void MMA8452Standby(); // Puts the i2c module into standby mode
void MMA8452Active();  // Bring the MMA8452 back on-line
void initMMA8452(byte fsr, byte dataRate);  // Initialize the MMA8452
void enable32Khz(uint8_t enable);  // Need to turn on the 32k square wave for bus moderation


// Prototypes for General Functions
void BlinkForever(); // Ends execution
void LogHourlyEvent(DateTime LogTime); // Log Hourly Event()
void LogDailyEvent(DateTime LogTime); // Log Daily Event()
void CheckForBump(); // Check for bump
void WakeUpNow();      // here the interrupt is handled after wakeup
void sleepNow();  // Puts the Arduino to Sleep


// Prototypes for Date and Time Functions
int SetTimeDate(); // Sets the RTC date and time
void PrintTimeDate(); // Prints to the console
unsigned long toUnixTime(DateTime ut); //Converts to Unix time for storage
void toArduinoTime(unsigned long unixT); //Converts to Arduino Time for use with the RTC and program


// Define variables and constants
// Pin Value Variables
const int TalkPin = A0;  // This is the open-drain line for signaling i2c mastery
const int The32kPin = A1;  // This is a 32k squarewave from the DS3231


// FRAM and Unix time variables
unsigned int  framAddr;
unsigned long unixTime;
// unsigned long time;
TimeElements currentTime;
DateTime t;
int lastHour = 0;  // For recording the startup values
int lastDate = 0;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
unsigned int hourlyPersonCount = 0;  // hourly counter
unsigned int dailyPersonCount = 0;   //  daily counter
byte currentHourlyPeriod;    // This is where we will know if the period changed
byte currentDailyPeriod;     // We will keep daily counts as well as period counts


// Accelerometer
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
boolean refreshMenu = 1;       //  Tells whether to write the menu
boolean inTest = 0;            // Are we in a test or not

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
    
    batteryMonitor.reset();               // Initialize the battery monitor
    batteryMonitor.quickStart();
    
    if (! rtc.begin()) {                    // Not sure if this is working
        Serial.println("Couldn't find RTC");
        BlinkForever();
    }
    
    enable32Khz(1); // turns on the 32k squarewave - need to test effect on power
    
    if (rtc.lostPower()) {
        Serial.println("RTC lost power, lets set the time!");
        // following line sets the RTC to the date & time this sketch was compiled
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        // This line sets the RTC with an explicit date & time, for example to set
        // January 21, 2014 at 3am you would call:
        // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }
    
    // Set up the interrupt pins, they're set as active low with an external pull-up
    pinMode(INT2PIN, INPUT);
    
    if (fram.begin()) {  // you can stick the new i2c addr in here, e.g. begin(0x51);
        Serial.println(F("Found I2C FRAM"));
    } else {
        Serial.println(F("No I2C FRAM found ... check your connections"));
        BlinkForever();
    }
    
    // Check to see if the memory map in the sketch matches the data on the chip
    if (fram.read8(VERSIONADDR) != VERSIONNUMBER) {
        Serial.print(F("FRAM Version Number: "));
        Serial.println(fram.read8(VERSIONADDR));
        Serial.read();
        Serial.println(F("Memory/Sketch mismatch! Erase FRAM? (Y/N)"));
        while (!Serial.available());
        switch (Serial.read()) {    // Give option to erase and reset memory
            case 'Y':
                ResetFRAM();
                break;
            case 'N':
                Serial.println(F("Cannot proceed"));
                BlinkForever();
                break;
            default:
                BlinkForever();
        }
    }
    
    // Import the accelSensitivity and Debounce values from memory
    Serial.print(F("Sensitivity set to: "));
    accelSensitivity = fram.read8(SENSITIVITYADDR);
    Serial.println(accelSensitivity);
    Serial.print(F("Debounce set to: "));
    debounce = fram.read8(DEBOUNCEADDR);
    Serial.println(debounce);
    
    
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
                PrintTimeDate();
                stateOfCharge = batteryMonitor.getSoC();
                Serial.print(F("State of charge: "));
                Serial.print(stateOfCharge);
                Serial.println("%");
                Serial.print(F("Sensitivity set to: "));
                Serial.println(fram.read8(SENSITIVITYADDR));
                Serial.print(F("Debounce set to: "));
                Serial.println(fram.read8(DEBOUNCEADDR));
                Serial.print(F("Hourly count: "));
                Serial.println(FRAMread16(CURRENTHOURLYCOUNTADDR));
                Serial.print(F("Daily count: "));
                Serial.println(FRAMread16(CURRENTDAILYCOUNTADDR));
                break;
            case '2':     // Set the clock
                SetTimeDate();
                PrintTimeDate();
                Serial.println(F("Date and Time Set"));
                break;
            case '3':  // Change the sensitivity
                Serial.println(F("Enter 0 (most) to 9 (least)"));
                while (Serial.available() == 0) {  // Look for char in serial queue and process if found
                    continue;
                }
                accelInputValue = (Serial.parseInt());
                accelSensitivity = byte(accelInputValue);
                Serial.print(F("accelSensitivity set to: "));
                Serial.println(accelInputValue);
                fram.write8(SENSITIVITYADDR, accelSensitivity);
                initMMA8452(accelFullScaleRange, dataRate);  // init the accelerometer if communication is OK
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
                fram.write8(DEBOUNCEADDR, debounce);
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
                    inTest = 1;
                    t = rtc.now();                    // Gets the current time
                    currentHourlyPeriod = HOURLYPERIOD;   // Sets the hour period for when the count starts (see #defines)
                    currentDailyPeriod = DAILYPERIOD;     // And the day  (see #defines)
                    // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over
                    unsigned long unixTime = FRAMread32(CURRENTCOUNTSTIME);
                    TimeElements timeElement;
                    breakTime(unixTime, timeElement);
                    lastHour = int(timeElement.Hour);
                    lastDate = int(timeElement.Day);
                    if (currentDailyPeriod == lastDate) {
                        Serial.println("Restoring Counts");
                        dailyPersonCount = FRAMread16(CURRENTDAILYCOUNTADDR);  // Load Daily Count from memory
                        if (currentHourlyPeriod == lastHour) {
                            hourlyPersonCount = FRAMread16(CURRENTHOURLYCOUNTADDR);  // Load Hourly Count from memory
                        }
                    }
                    source = readRegister(0x22);     // Reads the PULSE_SRC register to reset it
                    attachInterrupt(1, WakeUpNow, LOW);   // use interrupt 1 (pin 3) and run wakeUpNow when pin 3 goes LOW
                    Serial.println(F("Test Started"));
                }
                else {
                    inTest = 0;
                    source = readRegister(0x22);  // Reads the PULSE_SRC register to reset it
                    detachInterrupt(1);           // disables interrupt 1 on pin 3 so the program can execute
                    t = rtc.now();            // Gets the current time
                    FRAMwrite16(CURRENTDAILYCOUNTADDR, dailyPersonCount);   // Load Daily Count to memory
                    FRAMwrite16(CURRENTHOURLYCOUNTADDR, hourlyPersonCount);  // Load Hourly Count to memory
                    unsigned long unixTime = toUnixTime(t);  // Convert to UNIX Time
                    FRAMwrite32(CURRENTCOUNTSTIME, unixTime);   // Write to FRAM - this is so we know when the last counts were saved
                    hourlyPersonCount = 0;        // Reset Person Count
                    dailyPersonCount = 0;         // Reset Person Count
                    Serial.println(F("Test Stopped"));
                    refreshMenu = 1;
                }
                break;
            case '8':   // Dump the hourly data to the monitor
                Serial.println(F("Hour Ending -   Count  - Battery %"));
                for (int i=0; i < HOURLYCOUNTNUMBER; i++) {
                    if (fram.read8((HOURLYOFFSET + (i+FRAMread16(HOURLYPOINTERADDR)) % HOURLYCOUNTNUMBER)*WORDSIZE) != 0) {
                        unsigned long unixTime = FRAMread32((HOURLYOFFSET + (i+FRAMread16(HOURLYPOINTERADDR)) % HOURLYCOUNTNUMBER)*WORDSIZE);
                        toArduinoTime(unixTime);
                        Serial.print(" - ");
                        Serial.print(FRAMread16(((HOURLYOFFSET + (i+FRAMread16(HOURLYPOINTERADDR)) % HOURLYCOUNTNUMBER)*WORDSIZE)+HOURLYCOUNTOFFSET));
                        Serial.print("  -  ");
                        Serial.print(fram.read8(((HOURLYOFFSET + (i+FRAMread16(HOURLYPOINTERADDR)) % HOURLYCOUNTNUMBER)*WORDSIZE)+HOURLYBATTOFFSET));
                        Serial.println("%");
                    }
                }
                Serial.println(F("Done"));
                break;
            case '9':  // Download the daily counts
                Serial.println(F("Date - Count - Battery %"));
                for (int i=0; i < DAILYCOUNTNUMBER; i++) {
                    if (fram.read8((DAILYOFFSET + (i+fram.read8(DAILYPOINTERADDR)) % DAILYCOUNTNUMBER)*WORDSIZE+DAILYCOUNTOFFSET) != 0) {
                        Serial.print(fram.read8((DAILYOFFSET + (i+fram.read8(DAILYPOINTERADDR)) % DAILYCOUNTNUMBER)*WORDSIZE));
                        Serial.print("/");
                        Serial.print(fram.read8((DAILYOFFSET + (i+fram.read8(DAILYPOINTERADDR)) % DAILYCOUNTNUMBER)*WORDSIZE+DAILYDATEOFFSET));
                        Serial.print(" - ");
                        Serial.print(FRAMread16((DAILYOFFSET + (i+fram.read8(DAILYPOINTERADDR)) % DAILYCOUNTNUMBER)*WORDSIZE+DAILYCOUNTOFFSET));
                        Serial.print("  -  ");
                        Serial.print(fram.read8((DAILYOFFSET + (i+fram.read8(DAILYPOINTERADDR)) % DAILYCOUNTNUMBER)*WORDSIZE+DAILYBATTOFFSET));
                        Serial.println("%");
                    }
                }
                Serial.println(F("Done"));
                break;
            default:
                Serial.println(F("Invalid choice - try again"));
        }
        Serial.read();
    }
    if (inTest == 1) {
        CheckForBump();
        if (millis() >= lastBump + delaySleep) {
            Serial.println(F("Serial: Entering Sleep mode"));
            delay(100);     // this delay is needed, the sleep function will provoke a Serial error otherwise!!
            sleepNow();     // sleep function called here
        }
    }
}


void CheckForBump() // This is where we check to see if an interrupt is set when not asleep
{
    if (digitalRead(INT2PIN)==0)    // If int2 goes HIGH, either p/l has changed or there's been a single/double tap
    {
        if ((source & 0x08)==0x08) { // We are only interested in the TAP register so read that
            if (millis() >= lastBump + debounce) {
                t = rtc.now();
                if (int(HOURLYPERIOD) != currentHourlyPeriod) {
                    LogHourlyEvent(t);
                    Serial.print(F("Hour: "));
                    Serial.print(currentHourlyPeriod);
                    Serial.print(F(" - count: "));
                    Serial.println(hourlyPersonCount);
                    hourlyPersonCount = 0;                    // Reset and increment the Person Count in the new period
                    currentHourlyPeriod = int(HOURLYPERIOD);  // Change the time period
                    if (currentHourlyPeriod >= 19 ||  currentHourlyPeriod <= 6) {
                        // Turn things off at night
                    }
                    else {
                        // Turn things  on during the day
                    }
                }
                if (int(DAILYPERIOD) != currentDailyPeriod) {
                    LogDailyEvent(t);
                    Serial.print(F("Day: "));
                    Serial.print(currentDailyPeriod);
                    Serial.print(F(" - count: "));
                    Serial.println(dailyPersonCount);
                    dailyPersonCount = 0;    // Reset and increment the Person Count in the new period
                    currentDailyPeriod = int(DAILYPERIOD);  // Change the time period
                }
                hourlyPersonCount++;                    // Increment the PersonCount
                FRAMwrite16(CURRENTHOURLYCOUNTADDR, hourlyPersonCount);  // Load Hourly Count to memory
                dailyPersonCount++;                    // Increment the PersonCount
                FRAMwrite16(CURRENTDAILYCOUNTADDR, dailyPersonCount);   // Load Daily Count to memory
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


void LogHourlyEvent(DateTime LogTime) // Log Hourly Event()
{
    unsigned long pointer = (HOURLYOFFSET + FRAMread16(HOURLYPOINTERADDR))*WORDSIZE;  // get the pointer from memory and add the offset
    unsigned long unixTime = toUnixTime(LogTime);  // Convert to UNIX Time
    FRAMwrite32(pointer, unixTime);   // Write to FRAM - this is the end of the period
    FRAMwrite16(pointer+HOURLYCOUNTOFFSET,hourlyPersonCount);
    stateOfCharge = batteryMonitor.getSoC();
    fram.write8(pointer+HOURLYBATTOFFSET,stateOfCharge);
    unsigned int newHourlyPointerAddr = (FRAMread16(HOURLYPOINTERADDR)+1) % HOURLYCOUNTNUMBER;  // This is where we "wrap" the count to stay in our memory space
    FRAMwrite16(HOURLYPOINTERADDR,newHourlyPointerAddr);
}


void LogDailyEvent(DateTime LogTime) // Log Daily Event()
{
    int pointer = (DAILYOFFSET + fram.read8(DAILYPOINTERADDR))*WORDSIZE;  // get the pointer from memory and add the offset
    fram.write8(pointer,LogTime.month()); // should be time.month
    fram.write8(pointer+DAILYDATEOFFSET,LogTime.day());  // Write to FRAM - this is the end of the period  - should be time.date
    FRAMwrite16(pointer+DAILYCOUNTOFFSET,dailyPersonCount);
    stateOfCharge = batteryMonitor.getSoC();
    fram.write8(pointer+DAILYBATTOFFSET,stateOfCharge);
    byte newDailyPointerAddr = (fram.read8(DAILYPOINTERADDR)+1) % DAILYCOUNTNUMBER;  // This is where we "wrap" the count to stay in our memory space
    fram.write8(DAILYPOINTERADDR,newDailyPointerAddr);
}


int SetTimeDate() {
    Serial.println(F("Enter Seconds (0-59): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    int sec = Serial.parseInt();
    Serial.println(F("Enter Minutes (0-59): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    int minute = Serial.parseInt();
    Serial.println(F("Enter Hours (0-23): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    int hour = Serial.parseInt();
    Serial.println(F("Enter Day of the Month (1-31): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    int day = Serial.parseInt();
    Serial.println(F("Enter the Month (1-12): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    int month = Serial.parseInt();
    Serial.println(F("Enter the Year (0-99): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    int year = 2000 + Serial.parseInt();
    rtc.adjust(DateTime(year,month,day,hour, minute, sec));       // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    
}   // Function to set the date and time from the terminal window

void PrintTimeDate()  // Prints time and date to the console
{
    DateTime now = rtc.now();
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
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
    
    attachInterrupt(1,WakeUpNow, LOW); // use interrupt 1 (pin 3) and run function
    
    sleep_mode();            // here the device is actually put to sleep!!
    // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
    
    sleep_disable();         // first thing after waking from sleep:
    // disable sleep...
    detachInterrupt(1);      // disables interrupt 1 on pin 3 so the
    // wakeUpNow code will not be executed
    // during normal running time.
    //    digitalWrite(BLUEFRUITPWR,HIGH);   // Turn on the Bluefruit UART
    
}


unsigned long toUnixTime(DateTime ut)   // For efficiently storing time in memory
{
    TimeElements timeElement;
    timeElement.Month = ut.month();
    timeElement.Day = ut.day();
    timeElement.Year = (ut.year()-1970);
    timeElement.Hour = ut.hour();
    timeElement.Minute = ut.minute();
    timeElement.Second = ut.second();
    return makeTime(timeElement);
}

void toArduinoTime(unsigned long unixT) // Puts time in format for reporting
{
    TimeElements timeElement;
    breakTime(unixT, timeElement);
    Serial.print(timeElement.Month);
    Serial.print("/");
    Serial.print(timeElement.Day);
    Serial.print("/");
    Serial.print(1970+timeElement.Year);
    Serial.print(" ");
    Serial.print(timeElement.Hour);
    Serial.print(":");
    if(timeElement.Minute < 10) Serial.print("0");
    Serial.print(timeElement.Minute);
    Serial.print(":");
    if(timeElement.Second < 10) Serial.print("0");
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

boolean TakeTheBus()    // Claim the shared i2c bus
{
    int timeout = 10000;  // We will wait ten seconds then give up
    unsigned long startListening = millis();
    //Serial.println("Simblee has the Bus");
    while(!digitalRead(The32kPin)) {} // The Arduino will wait until the SQW pin goes low (Arduino needs high)
    while (!digitalRead(TalkPin))  { // Only proceed once the TalkPin is high or we timeout
        if (millis() >= timeout + startListening) return 0;  // timed out
    }
    pinMode(TalkPin,OUTPUT);  // Change to output
    digitalWrite(TalkPin,LOW);  // Claim the bus
    return 1;           // We have it
}

boolean GiveUpTheBus()  // Give up the shared i2c bus
{
    pinMode(TalkPin,INPUT);  // Start listening again
    //Serial.println("Simblee gave up the Bus");
    return 1;
}


uint8_t FRAMread8(unsigned int address) // Read 8 bits from FRAM
{
    uint8_t result;
    if (TakeTheBus()) {  // Request exclusive access to the bus
        result = fram.read8(address);
    }
    GiveUpTheBus();// Release exclusive access to the bus
    return result;
}

void FRAMwrite8(unsigned int address, uint8_t value)    // Write 8 bits to FRAM
{
    uint8_t result;
    if (TakeTheBus()) {  // Request exclusive access to the bus
        fram.write8(address,value);
    }
    GiveUpTheBus();// Release exclusive access to the bus
}



void FRAMwrite16(unsigned int address, int value)   // Write 16 bits to FRAM
{
    //This function will write a 2 byte (16bit) long to the eeprom at
    //the specified address to address + 1.
    //Decomposition from a long to 2 bytes by using bitshift.
    //One = Most significant -> Four = Least significant byte
    byte two = (value & 0xFF);
    byte one = ((value >> 8) & 0xFF);
    
    //Write the 2 bytes into the eeprom memory.
    if (TakeTheBus()) {  // Request exclusive access to the bus
        fram.write8(address, two);
        fram.write8(address + 1, one);
    }
    GiveUpTheBus();// Release exclusive access to the bus
}

int FRAMread16(unsigned int address)    // Read 16 bits from FRAM
{
    long two;
    long one;
    if(TakeTheBus()) {  // Request exclusive access to the bus
        //Read the 2 bytes from  memory.
        two = fram.read8(address);
        one = fram.read8(address + 1);
    }
    GiveUpTheBus();// Release exclusive access to the bus
    //Return the recomposed long by using bitshift.
    return ((two << 0) & 0xFF) + ((one << 8) & 0xFFFF);
}


void FRAMwrite32(int address, unsigned long value)  // Write 16 bits to FRAM
{
    //This function will write a 4 byte (32bit) long to the eeprom at
    //the specified address to address + 3.
    //Decomposition from a long to 4 bytes by using bitshift.
    //One = Most significant -> Four = Least significant byte
    
    byte four = (value & 0xFF);
    byte three = ((value >> 8) & 0xFF);
    byte two = ((value >> 16) & 0xFF);
    byte one = ((value >> 24) & 0xFF);
    
    //Write the 4 bytes into the eeprom memory.
    if (TakeTheBus()) {  // Request exclusive access to the bus
        fram.write8(address, four);
        fram.write8(address + 1, three);
        fram.write8(address + 2, two);
        fram.write8(address + 3, one);
    }
    GiveUpTheBus();// Release exclusive access to the bus
}

unsigned long FRAMread32(unsigned long address) // Read 32 bits from FRAM
{
    long four;
    long three;
    long two;
    long one;
    if(TakeTheBus()) {  // Request exclusive access to the bus
        //Read the 4 bytes from memory.
        four = fram.read8(address);
        three = fram.read8(address + 1);
        two = fram.read8(address + 2);
        one = fram.read8(address + 3);
    }
    GiveUpTheBus();// Release exclusive access to the bus
    //Return the recomposed long by using bitshift.
    return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}


void ResetFRAM()  // This will reset the FRAM - set the version and preserve delay and sensitivity
{
    // Note - have to hard code the size here due to this issue - http://www.microchip.com/forums/m501193.aspx
    Serial.println("Resetting Memory");
    for (unsigned long i=3; i < 32768; i++) {  // Start at 3 to not overwrite debounce and sensitivity
        fram.write8(i,0x0);
        //Serial.println(i);
        if (i==8192) Serial.println(F("25% done"));
        if (i==16384) Serial.println(F("50% done"));
        if (i==(24576)) Serial.println(F("75% done"));
        if (i==32767) Serial.println(F("Done"));
    }
    fram.write8(VERSIONADDR,VERSIONNUMBER);  // Reset version to match #define value for sketch
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
