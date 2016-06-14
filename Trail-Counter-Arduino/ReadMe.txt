
  Trail-Counter-Arduino
  Project
  ----------------------------------
  Developed with embedXcode

  Project Trail-Counter-Arduino
  Created by Charles McClelland on 5/10/16
  Copyright © 2016 Charles McClelland
  Licence GNU General Public Licence



  References
  ----------------------------------
/*
This code is for setting up and testing the clock, data logger and the sensor using a bluetooth serial
connection on the FTDI / UART port.

Acknolwedgemetns - I have benefitted greatly from the following contributions.
- MMA8452Q Example Code by: Jim Lindblom SparkFun Electronics November 17, 2011
- RTClib from Adafruit
- MAX1704 Library and code from Maxim
- Triembed.org - great group who answered my questions and showed me the path to this version

License - BSD Release 3

Hardware setup:
- Sparkfun Arduino Pro Mini - 3.3V / 8Mhz
- MMA8452 Breakout
- SDA - Pin A4 <- Pull ups are on the breakout board
- SCL - Pin A5 <- Ditto
- INT2 - D3
- INT1 - D2  - Not currently used
- TI FRAM Chip on i2c Bus
- DS3231 RTC Module on the i2c bus
- Indicator LED on pin 4
- accelSensitivity and debounce are set via terminal and stored in FRAM
- Inverter used to invert the interrupt from the accelerometer and send to INT2PIN
- Adafruit Bluetooth UART friend used to provide remote serial support

Memory Map - 256kb or 32kB divided into 4096 words - the  first one is reserved
Byte     Value
The first word is for system data
0        Memory Map Version (see what version the program expects in the first #define)
1        accelSensitivity
2-3       Delay
4        Daily Count Pointer
5-5      Current Hourly Count Pointer (16-bit number)
7        Control Register  (8 - 6 Reserved, 5-Erase Memory, 4-Reset Current Count, 3-Start / Stop Test, 2-Set Sensitivity, 1-Set Delay)
The second word is for storing the current count data
8-9      Current Hourly Count (16-bit)
10-11    Current Daily Count (16-bit)
12-15    EPOCH Time when last counts recorded (32-bits)
Words 3-16 are 14 days worth of daily counts - if this changes - need to change #offsets and DAILYCOUNTNUMBER
0        Month
1        Day
2 - 3    Daily Count (16-bit)
4        Daily Battery Level
5        Reserved
6        Reserved
7        Reserved
Words from 17 to the end of the memory store hourly data
0 - 3    EPOCH Time
4 - 5    Hourly count (16-bit)
6        Houlry Battery Level
7        Reserved

The part is open for an average of 12 hours per day so about 340 days of hourly data on a 256k chip

Note: The MMA8452 is an I2C sensor, however this code does
not make use of the Arduino Wire library. Because the sensor
is not 5V tolerant, we can't use the internal pull-ups used
by the Wire library. Instead use the included i2c.h, defs.h and types.h files.
*/




  embedXcode
  embedXcode+
  ----------------------------------
  Embedded Computing on Xcode
  Copyright © Rei VILO, 2010-2016
  All rights reserved
  http://embedXcode.weebly.com

