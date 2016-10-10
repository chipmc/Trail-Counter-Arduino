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
# Last update: Oct 08, 2015 release 5.3.1



# RFduino specifics
# ----------------------------------
#
#
PLATFORM         := RFduino
BUILD_CORE        = $(call PARSE_BOARD,$(BOARD_TAG),build.core)
PLATFORM_TAG      = EMBEDXCODE=$(RELEASE_NOW) ARDUINO=10610 RFDUINO
APPLICATION_PATH := $(RFDUINO_PATH)
PLATFORM_VERSION := $(RFDUINO_RELEASE) for Arduino $(ARDUINO_CC_RELEASE)

HARDWARE_PATH     = $(APPLICATION_PATH)/hardware/RFduino/$(RFDUINO_RELEASE)
TOOL_CHAIN_PATH   = $(APPLICATION_PATH)/tools/arm-none-eabi-gcc/4.8.3-2014q1
OTHER_TOOLS_PATH  = $(APPLICATION_PATH)/tools/RFDLoader/1.5

UPLOADER          = RFDLoader
UPLOADER_PATH     = $(OTHER_TOOLS_PATH)
UPLOADER_EXEC     = $(UPLOADER_PATH)/RFDLoader_osx

APP_TOOLS_PATH   := $(TOOL_CHAIN_PATH)/bin
CORE_LIB_PATH    := $(HARDWARE_PATH)/cores/arduino
APP_LIB_PATH     := $(HARDWARE_PATH)/libraries
BOARDS_TXT       := $(HARDWARE_PATH)/boards.txt

# Sketchbook/Libraries path
# wildcard required for ~ management
# ?ibraries required for libraries and Libraries
#
ifeq ($(USER_LIBRARY_DIR)/Arduino15/preferences.txt,)
    $(error Error: run RFduino once and define the sketchbook path)
endif

ifeq ($(wildcard $(SKETCHBOOK_DIR)),)
    SKETCHBOOK_DIR = $(shell grep sketchbook.path $(USER_LIBRARY_DIR)/Arduino15/preferences.txt | cut -d = -f 2)
endif

ifeq ($(wildcard $(SKETCHBOOK_DIR)),)
    $(error Error: sketchbook path not found)
endif

USER_LIB_PATH  = $(wildcard $(SKETCHBOOK_DIR)/?ibraries)
CORE_AS_SRCS   = $(wildcard $(CORE_LIB_PATH)/*.S) # */


# Rules for making a c++ file from the main sketch (.pde)
#
PDEHEADER      = \\\#include \"Arduino.h\"  


# Tool-chain names
#
CC      = $(APP_TOOLS_PATH)/arm-none-eabi-gcc
CXX     = $(APP_TOOLS_PATH)/arm-none-eabi-g++
AR      = $(APP_TOOLS_PATH)/arm-none-eabi-ar
OBJDUMP = $(APP_TOOLS_PATH)/arm-none-eabi-objdump
OBJCOPY = $(APP_TOOLS_PATH)/arm-none-eabi-objcopy
SIZE    = $(APP_TOOLS_PATH)/arm-none-eabi-size
NM      = $(APP_TOOLS_PATH)/arm-none-eabi-nm

LDSCRIPT = $(call PARSE_BOARD,$(BOARD_TAG),build.ldscript)
VARIANT  = $(call PARSE_BOARD,$(BOARD_TAG),build.variant)
VARIANT_PATH = $(HARDWARE_PATH)/variants/$(VARIANT)


MCU             = $(call PARSE_BOARD,$(BOARD_TAG),build.mcu)
MCU_FLAG_NAME   = mcpu

BUILD_CORE_LIB_PATH  = $(VARIANT_PATH)
BUILD_CORE_LIBS_LIST = $(subst .cpp,,$(subst $(BUILD_CORE_LIB_PATH)/,,$(wildcard $(BUILD_CORE_LIB_PATH)/*.cpp))) # */
BUILD_CORE_CPP_SRCS = $(filter-out %program.cpp %main.cpp,$(wildcard $(BUILD_CORE_LIB_PATH)/*.cpp)) # */
BUILD_CORE_OBJ_FILES  = $(BUILD_CORE_C_SRCS:.c=.c.o) $(BUILD_CORE_CPP_SRCS:.cpp=.cpp.o)
BUILD_CORE_OBJS       = $(patsubst $(APPLICATION_PATH)/%,$(OBJDIR)/%,$(BUILD_CORE_OBJ_FILES))


EXTRA_CPPFLAGS  = -fno-builtin $(call PARSE_BOARD,$(BOARD_TAG),build.extra_flags)
EXTRA_CPPFLAGS += $(addprefix -D, $(PLATFORM_TAG))
EXTRA_CPPFLAGS += $(addprefix -I$(HARDWARE_PATH)/, variants/RFduino system/RFduino system/RFduino/include system/CMSIS/CMSIS/Include)

EXTRA_CXXFLAGS = -fno-rtti -fno-exceptions

LDFLAGS         = -Wl,--gc-sections --specs=nano.specs
LDFLAGS        += -Wl,--warn-common -Wl,--warn-section-align
LDFLAGS        += -Wl,-Map,$(OBJDIR)/embedXcode.cpp.map -Wl,--cref
LDFLAGS        += $(call PARSE_BOARD,$(BOARD_TAG),build.extra_flags) $(addprefix -D, $(PLATFORM_TAG))
LDFLAGS        += -$(MCU_FLAG_NAME)=$(MCU) -DF_CPU=$(F_CPU)
LDFLAGS        += -T$(VARIANT_PATH)/$(LDSCRIPT) -L$(OBJDIR)

INCLUDE_A       = $(wildcard $(VARIANT_PATH)/*.a) # */

# Specific OBJCOPYFLAGS for objcopy only
# objcopy uses OBJCOPYFLAGS only
#
OBJCOPYFLAGS  = -v -Oihex

# Target
#
TARGET_HEXBIN = $(TARGET_HEX)

#MAX_RAM_SIZE = $(call PARSE_BOARD,$(BOARD_TAG),upload.ram.maximum_size)


# Commands
# ----------------------------------
#
FIRST_O_IN_LD   = $$(find . -name syscalls.c.o) $$(find . -name variant.cpp.o)

COMMAND_LINK    = $(CXX) $(LDFLAGS) $(OUT_PREPOSITION)$@ -Wl,--start-group $(FIRST_O_IN_LD) $(LOCAL_OBJS) $(LOCAL_ARCHIVES) $(INCLUDE_A) $(TARGET_A) -Wl,--end-group

COMMAND_UPLOAD  = $(UPLOADER_EXEC) -q $(USED_SERIAL_PORT) $(TARGET_HEX)

