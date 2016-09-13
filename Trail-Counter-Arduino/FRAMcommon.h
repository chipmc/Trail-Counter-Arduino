//
//  FRAMcommon.h
//  Trail-Counter-Simblee
//
//  Created by Charles McClelland on 7/14/16.
//  Copyright © 2016 Charles McClelland. All rights reserved.
//

#ifndef FRAMcommon_h
#define FRAMcommon_h

#include "Adafruit_FRAM_I2C.h"   // Note - had to comment out the Wire.begin() in this library

// Prototypes for FRAM Functions
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C(); // Init the FRAM
unsigned long FRAMread32(unsigned long address); // Reads a 32 bit word
void FRAMwrite32(int address, unsigned long value);  // Write 32 bits to FRAM
int FRAMread16(unsigned int address); // Reads a 16 bit word
void FRAMwrite16(unsigned int address, int value);   // Write 16 bits to FRAM
uint8_t FRAMread8(unsigned int address);  // Reads a 8 bit word
void FRAMwrite8(unsigned int address, uint8_t value);    // Write 8 bits to FRAM
void FRAMwrite8(unsigned int address, uint8_t value); //Writes a 32-bit word
void ResetFRAM();  // This will reset the FRAM - set the version and preserve delay and sensitivity

// Variables
boolean clockHighorLow = LOW;  // Set this as HIGH for for one Master and LOW for the other.

// Prototypes for i2c functions
boolean GiveUpTheBus(); // Give up the i2c bus
boolean TakeTheBus(); // Take the 12c bus
void enable32Khz(uint8_t enable);  // Need to turn on the 32k square wave for bus moderation

// Prototypes for General Functions
void NonBlockingDelay(int millisDelay);  // Used for a non-blocking delay

// Pin definitions
// Pin Value Variables
int TalkPin = A0;  // This is the open-drain line for signaling i2c mastery
int The32kPin = A1;  // This is a 32k squarewave from the DS3231


uint8_t FRAMread8(unsigned int address)
{
    uint8_t result;
    //Serial.println("In FRAMread8");
    if (TakeTheBus()) {   // Request exclusive access to the bus
        //Serial.println("got the bus");
        result = fram.read8(address);
    }
    GiveUpTheBus();       // Release exclusive access to the bus
    return result;
}

void FRAMwrite8(unsigned int address, uint8_t value)    // Write 8 bits to FRAM
{
    if (TakeTheBus()) {  // Request exclusive access to the bus
        fram.write8(address,value);
    }
    GiveUpTheBus();// Release exclusive access to the bus
}

int FRAMread16(unsigned int address)
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

unsigned long FRAMread32(unsigned long address)
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

void FRAMwrite32(int address, unsigned long value)  // Write 32 bits to FRAM
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


void ResetFRAM()  // This will reset the FRAM - set the version and preserve delay and sensitivity
{
    // Note - have to hard code the size here due to this issue - http://www.microchip.com/forums/m501193.aspx
    Serial.println("Resetting Memory");
    for (unsigned long i=4; i < 32768; i++) {  // Start at 4 to not overwrite debounce and sensitivity
        FRAMwrite8(i,0x0);
        if (i==8192) Serial.println(F("25% done"));
        if (i==16384) Serial.println(F("50% done"));
        if (i==(24576)) Serial.println(F("75% done"));
        if (i==32767) Serial.println(F("Done"));
    }
    FRAMwrite8(VERSIONADDR,VERSIONNUMBER);  // Reset version to match #define value for sketch
}

boolean TakeTheBus()
{
    //Serial.print(F(Asking for the bus..."));
    if (clockHighorLow) {
        while(digitalRead(The32kPin)) {} // The Simblee will only read the Talk line when SQW pin goes low
    }
    else while(!(digitalRead(The32kPin))) {} // The Simblee will only read the Talk line when SQW pin goes low

    while (!digitalRead(TalkPin)) { // Only proceed once the TalkPin is high
        NonBlockingDelay(50);
    }
    pinMode(TalkPin,OUTPUT);        // Change to output
    digitalWrite(TalkPin,LOW);      // Claim the bus by bringing the TalkPin LOW
    //Serial.println(F("..We have the bus"));
    return 1;                       // We have it
}

boolean GiveUpTheBus()
{
    digitalWrite(TalkPin,HIGH); // Not sure if this is needed - still for completeness.
    pinMode(TalkPin,INPUT_PULLUP);  // Start listening again
    //Serial.println("Simblee: We gave up the Bus");
    return 1;
}


void NonBlockingDelay(int millisDelay)  // Used for a non-blocking delay
{
    unsigned long commandTime = millis();
    while (millis() <= millisDelay + commandTime) { }
    return;
}


#endif /* FRAMcommon_h */

