#
# embedXcode
# ----------------------------------
# Embedded Computing on Xcode
#
# Copyright © Rei VILO, 2010-2016
# http://embedxcode.weebly.com
# All rights reserved
#
#
# Last update: Oct 31, 2015 release 4.0.0

# Particle is the new name for Spark

#WARNING_MESSAGE = Support for Particle is put on hold.


# Particle specifics with pre-compiled library
# ----------------------------------
#
# lib_eXspark.a available under Spark/build folder
#
#
PLATFORM           := Spark
BUILD_CORE       = $(call PARSE_BOARD,$(BOARD_TAG),build.core)
PLATFORM_TAG        = SPARK=$(PLATFORM_VERSION) EMBEDXCODE=$(RELEASE_NOW) PLATFORM_ID=0
APPLICATION_PATH   := $(SPARK_PATH)/firmware
BOARDS_TXT         := $(SPARK_PATH)/boards.txt

ifeq ($(wildcard $(SPARK_APP)/*),) # */
    $(error Error: Particle framework not found)
endif

APP_TOOLS_PATH  = $(EMBEDXCODE_APP)/gcc-arm-none-eabi-4_8-2014q3/bin

# USER files
# Sketchbook/Libraries path
# wildcard required for ~ management
# ?ibraries required for libraries and Libraries
#
ifeq ($(SPARK_PATH)/preferences.txt,)
    $(error Error: define sketchbook.path in preferences.txt first)
endif

ifeq ($(wildcard $(SKETCHBOOK_DIR)),)
    SKETCHBOOK_DIR = $(shell grep sketchbook.path $(SPARK_PATH)/preferences.txt | cut -d = -f 2-)
endif

ifeq ($(wildcard $(SKETCHBOOK_DIR)),)
    $(error Error: sketchbook path not found ($(SKETCHBOOK_DIR)))
endif

USER_LIB_PATH  = $(wildcard $(SKETCHBOOK_DIR)/?ibraries)


# Uploaders
#
USB_VID         = 1d50
USB_PID         = 607f
UPLOADER_PATH   = /usr/local/bin

# WiFi option with spark
#
ifeq ($(UPLOADER),spark_wifi)
#    UPLOADER_EXEC   = $(UPLOADER_PATH)/node $(UPLOADER_PATH)/spark
    UPLOADER_EXEC   = $(UPLOADER_PATH)/node $(UPLOADER_PATH)/particle
    UPLOADER_OPTS   = flash $(SPARK_NAME)
    PREPARE_EXEC    =
    PREPARE_OPTS    =
    UPLOADER_RESET  =
    RESET_MESSAGE   = 0

# USB option with dfu utilities
#
else ifeq ($(UPLOADER),spark_usb)
    UPLOADER_EXEC   = $(UPLOADER_PATH)/dfu-util
    UPLOADER_OPTS   = -d $(USB_VID):$(USB_PID) -a 0 -i 0 -s 0x08005000:leave -D
    PREPARE_EXEC    = $(UPLOADER_PATH)/dfu-suffix
    PREPARE_OPTS    = -v $(USB_VID) -p $(USB_PID) -a
    UPLOADER_RESET  =
    RESET_MESSAGE   = 1

else
    $(error UPLOADER should be spark_usb or spark_wifi)
endif

# Rules for making a c++ file from the main sketch (.pde)
#
PDEHEADER      = \\\#include \"application.h\"


# Common and specific flags for gcc, g++, linker, objcopy
#
MCU_FLAG_NAME   = mcpu


# Tool-chain names
#
CC          = $(APP_TOOLS_PATH)/arm-none-eabi-gcc
CXX         = $(APP_TOOLS_PATH)/arm-none-eabi-g++
AR          = $(APP_TOOLS_PATH)/arm-none-eabi-ar
OBJDUMP     = $(APP_TOOLS_PATH)/arm-none-eabi-objdump
OBJCOPY     = $(APP_TOOLS_PATH)/arm-none-eabi-objcopy
SIZE        = $(APP_TOOLS_PATH)/arm-none-eabi-size
NM          = $(APP_TOOLS_PATH)/arm-none-eabi-nm
# ~
GDB         = $(APP_TOOLS_PATH)/arm-none-eabi-gdb
# ~~


# Optimisation
#
#ifeq ($(MAKECMDGOALS),debug)
#    OPTIMISATION   = -O0 -ggdb -DDEBUG
#else
    OPTIMISATION   ?= -Os -DNDEBUG
#endif

#SPARK_MAKE_OPTION := FULL


# Option for make
# . FAST with pre-compiled library
# . FULL build whole library
#
ifeq ($(wildcard $(SPARK_PATH)/build/$(LEVEL0)/$(TOOLCHAIN)/lib_eXspark.a),)
    SPARK_MAKE_OPTION := FULL
endif

ifeq ($(SPARK_MAKE_OPTION),FAST)
    include $(MAKEFILE_PATH)/Particle_fast.mk
else
    include $(MAKEFILE_PATH)/Particle_full.mk
endif


# Commands
# ----------------------------------
#
# Link command
#
COMMAND_LINK    = $(CXX) $(CPPFLAGS) $(OUT_PREPOSITION)$@ $(STARTUP_O) $(LOCAL_OBJS) $(LOCAL_ARCHIVES) -Wl,--start-group $(TARGET_A) $(SPARK_A) -Wl,--end-group $(LDFLAGS)


