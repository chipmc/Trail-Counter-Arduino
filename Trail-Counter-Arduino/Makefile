#
# embedXcode
# ----------------------------------
# Embedded Computing on Xcode
#
# Copyright © Rei VILO, 2010-2016
# All rights reserved
# http://embedxcode.weebly.com
#
# Last update: Jul 31, 2015 release 300


# Libraries
# ----------------------------------
# Declare application Arduino/chipKIT/Digispark/Energia/Maple/Microduino/Teensy/Wiring
# and user's libraries used 
# Short-listing libraries speeds-up building
# Typing = 0 takes none (default for application and user)
# Otherwise, leaving empty considers all (default for local)

# List application Arduino/chipKIT/Digispark/Energia/Maple/Microduino/Teensy/Wiring libraries here
# default = 0 = none
#
# ARDUINO : AVR = EEPROM HID SoftwareSerial SPI Wire 
# ARDUINO = Bridge Esplora Ethernet Firmata GSM 
#	LiquidCrystal Robot_Control Robot_Motor RobotIRremote SD Servo 
#	SpacebrewYun Stepper Temboo TFT WiFi 
#
APP_LIBS_LIST = Wire Timelib

# List user's libraries here 
# default = 0 = none
#
# USER = Adafruit_ADS1X15 Adafruit_AM2315 Adafruit_BLE_UART 
#	Adafruit_BLEFirmata Adafruit_BluefruitLE_nRF51 Adafruit_CC3000 
#	Adafruit_DotStar Adafruit_FONA_OLD Adafruit_FONA_Teensy 
#	Adafruit_FRAM_I2C Adafruit_GFX_Library Adafruit_GPS Adafruit_INA219 
#	Adafruit_MAX31855 Adafruit_NeoPixel Adafruit_SSD1306 APA102 Bridge DHT 
#	DS1307RTC DS3234 DSRTCLib Ethernet FastLED Firmata fxos8700cq GPS 
#	HIH61XX HIH61XXCommander i2c_t3 Keyboard LGPRS LSM303 MAX17043 
#	MicroView Mouse NewPing Phant RadioHead RFM69 RTClib 
#	RunningAverage SdFat Servo SFE_CC3000 SFE_MicroOLED SoftI2CMaster 
#	SPIFlash Temboo Time TinyWireM Twitter U8glib Ubidots 
#
USER_LIBS_LIST = MAX17043 Adafruit_FRAM_I2C DS3232RTC Time

# List local libraries here
# default = empty = all
#
LOCAL_LIBS_LIST = i2c FRAMcommon

# List core, application and user's libraries to be excluded
# For example, WiFi may crash on Arduino 1.0.2, Esplora on Arduino 1.0.3, Firmata on Teensy 3.0,
#   OneWire on MPIDE 0023, HardwareSerial may conflict with MarlinSerial
#
#EXCLUDE_LIBS = Firmata WiFi Esplora OneWire Robot_Control Robot_Control/utility Robot_Motor


# Warning options
# ----------------------------------
# Contrary to errors, warnings don't stop compilation but they point at possible cause of errors.
# For example, unused variables with unused-variable.
#
# If 0, no warnings
# If empty, all warnings, same as WARNING_OPTIONS = all
# WARNING_OPTIONS = all no-missing-braces no-conversion-null no-pointer-arith
# WARNING_OPTIONS = unused-variable unused-function unused-label unused-value no-conversion-null no-pointer-arith
#
WARNING_OPTIONS = unused-variable unused-function unused-label unused-value


# Applications path folder
# ----------------------------------
# Path to the folder where the IDEs are located
# By default, = /Applications
#
#APPLICATIONS_PATH = /Applications


# Serial console for Fast target
# ----------------------------------
# = 0 or = empty or comment: open serial console
# = 1: don't open serial console
#
NO_SERIAL_CONSOLE =


# Teensy USB options (default)
# ----------------------------------
# TEENSY_USB = (USB_SERIAL) USB_HID USB_SERIAL_HID USB_DISK USB_DISK_SDFLASH USB_MIDI
#   USB_RAWHID USB_FLIGHTSIM
# TEENSY_LAYOUT = (LAYOUT_US_ENGLISH) LAYOUT_CANADIAN_FRENCH LAYOUT_CANADIAN_MULTILINGUAL
#   LAYOUT_DANISH LAYOUT_FINNISH LAYOUT_FRENCH LAYOUT_FRENCH_BELGIAN LAYOUT_FRENCH_SWISS
#   LAYOUT_GERMAN LAYOUT_GERMAN_MAC
#   LAYOUT_GERMAN_SWISS LAYOUT_ICELANDIC LAYOUT_IRISH LAYOUT_ITALIAN LAYOUT_NORWEGIAN
#   LAYOUT_PORTUGUESE LAYOUT_PORTUGUESE_BRAZILIAN LAYOUT_SPANISH LAYOUT_SPANISH_LATIN_AMERICA
#   LAYOUT_SWEDISH LAYOUT_TURKISH LAYOUT_UNITED_KINGDOM LAYOUT_US_INTERNATIONAL
#
#TEENSY_USB = USB_SERIAL
#TEENSY_LAYOUT = LAYOUT_US_ENGLISH


# Teensy CPU options (default)
# ----------------------------------
# TEENSY_F_CPU = (96000000) 72000000 48000000 24000000 for Teensy 3.1
# TEENSY_F_CPU = (96000000) 48000000 24000000 for Teensy 3.0
# TEENSY_F_CPU = (48000000) 24000000 for Teensy LC
# TEENSY_F_CPU = (16000000) 8000000 4000000 for Teensy 2.0
# TEENSY_OPTIMISATION = (-Os)
#
#TEENSY_F_CPU = 48000000
#TEENSY_OPTIMISATION = -Os


# Parameters
# ----------------------------------
# Xcode takes BOARD_TAG and BOARD_PORT from the .xcconfig file
# For other IDEs than Xcode,

# BOARD_TAG is required
# If not defined, error BOARD_TAG not defined
#
#BOARD_TAG  = mega2560

# BOARD_PORT is optional
# If not defined, BOARD_PORT = /dev/tty.usb* (default)
#
BOARD_PORT = /dev/cu.usbserial*

# Xcode takes SKETCHBOOK_DIR from preferences.txt
# If not defined, each IDE has its own SKETCHBOOK_DIR (default)
# To share the same SKETCHBOOK_DIR along all IDEs, define
#
#SKETCHBOOK_DIR =

# SERIAL_BAUDRATE for the serial console, 9600 by default
# Uncomment and specify another speed
#
#SERIAL_BAUDRATE = 9600


# Miscellaneous
# ----------------------------------
# Manage path with space in the name
#
CURRENT_DIR   := $(shell pwd)
CURRENT_DIR   := $(shell echo '$(CURRENT_DIR)' | sed 's/ /\\\ /g')

# C-compliant project name
#
PROJECT_NAME_AS_IDENTIFIER = Trail_Counter_Arduino
SKETCH_EXTENSION = ino

MAKEFILE_PATH  = $(CURRENT_DIR)/Makefiles
UTILITIES_PATH = $(CURRENT_DIR)/Utilities

include $(MAKEFILE_PATH)/Step1.mk




















































































































































































































































































































































