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
# Last update: Feb 27, 2016 release 4.3.6


include $(MAKEFILE_PATH)/About.mk

# LinkIt One specifics
# ----------------------------------
#
PLATFORM         := LinkIt
PLATFORM_TAG      = ARDUINO=10610 ARDUINO_ARCH_MTK EMBEDXCODE=$(RELEASE_NOW) __LINKIT_ONE__ LINKIT
APPLICATION_PATH := $(LINKIT_ARM_PATH)
PLATFORM_VERSION := One $(LINKIT_ONE_RELEASE) for Arduino $(ARDUINO_CC_RELEASE)

HARDWARE_PATH     = $(APPLICATION_PATH)/hardware/arm/$(LINKIT_ONE_RELEASE)
TOOL_CHAIN_PATH   = $(APPLICATION_PATH)/tools/arm-none-eabi-gcc/4.8.3-2014q1
OTHER_TOOLS_PATH  = $(APPLICATION_PATH)/tools/linkit_tools/$(LINKIT_ONE_RELEASE)

BOARDS_TXT      := $(HARDWARE_PATH)/boards.txt
BUILD_CORE       = $(call PARSE_BOARD,$(BOARD_TAG),build.core)
BUILD_BOARD      = ARDUINO_$(call PARSE_BOARD,$(BOARD_TAG),build.board)

ESP_POST_COMPILE   = $(OTHER_TOOLS_PATH)/PackTag
BUILD_FLASH_SIZE   = $(call PARSE_BOARD,$(BOARD_TAG),build.maximum_size)
BUILD_FLASH_FREQ   = $(call PARSE_BOARD,$(BOARD_TAG),build.f_cpu)


UPLOADER            = PushTool
UPLOADER_PATH       = $(OTHER_TOOLS_PATH)
UPLOADER_EXEC       = $(UPLOADER_PATH)/PushTool
UPLOADER_OPTS       = -d arduino

APP_TOOLS_PATH      := $(TOOL_CHAIN_PATH)/bin
CORE_LIB_PATH       := $(HARDWARE_PATH)/cores/arduino


# Take assembler file as first
#
APP_LIB_PATH        := $(HARDWARE_PATH)/libraries
CORE_AS_SRCS         = $(wildcard $(CORE_LIB_PATH)/*.S) # */
FIRST_O_IN_A         = $$(find . -name syscalls_mtk.c.o)


# Sketchbook/Libraries path
# wildcard required for ~ management
# ?ibraries required for libraries and Libraries
#
ifeq ($(USER_LIBRARY_DIR)/Arduino15/preferences.txt,)
    $(error Error: run Arduino once and define the sketchbook path)
endif

ifeq ($(wildcard $(SKETCHBOOK_DIR)),)
    SKETCHBOOK_DIR = $(shell grep sketchbook.path $(wildcard ~/Library/Arduino15/preferences.txt) | cut -d = -f 2)
endif

ifeq ($(wildcard $(SKETCHBOOK_DIR)),)
    $(error Error: sketchbook path not found)
endif

USER_LIB_PATH  = $(wildcard $(SKETCHBOOK_DIR)/?ibraries)

VARIANT      = $(call PARSE_BOARD,$(BOARD_TAG),build.variant)
VARIANT_PATH = $(HARDWARE_PATH)/variants/$(VARIANT)

VARIANT_CPP_SRCS  = $(wildcard $(VARIANT_PATH)/*.cpp) # */
VARIANT_OBJ_FILES = $(VARIANT_CPP_SRCS:.cpp=.cpp.o)
VARIANT_OBJS      = $(patsubst $(APPLICATION_PATH)/%,$(OBJDIR)/%,$(VARIANT_OBJ_FILES))

# Rules for making a c++ file from the main sketch (.pde)
#
PDEHEADER      = \\\#include \"WProgram.h\"  


# Tool-chain names
#
CC      = $(APP_TOOLS_PATH)/arm-none-eabi-gcc
CXX     = $(APP_TOOLS_PATH)/arm-none-eabi-g++
AR      = $(APP_TOOLS_PATH)/arm-none-eabi-ar
OBJDUMP = $(APP_TOOLS_PATH)/arm-none-eabi-objdump
# /Applications/LinkIT.app/Contents/Java/hardware/tools/mtk/PackTag
OBJCOPY = $(ESP_POST_COMPILE)
SIZE    = $(APP_TOOLS_PATH)/arm-none-eabi-size
NM      = $(APP_TOOLS_PATH)/arm-none-eabi-nm

MCU_FLAG_NAME    = mcpu
MCU              = $(call PARSE_BOARD,$(BOARD_TAG),build.mcu)
F_CPU            = $(call PARSE_BOARD,$(BOARD_TAG),build.f_cpu)

INCLUDE_PATH     = $(HARDWARE_PATH)/system/libmtk
INCLUDE_PATH    += $(HARDWARE_PATH)/system/libmtk/include
INCLUDE_PATH    += $(CORE_LIB_PATH)
INCLUDE_PATH    += $(VARIANT_PATH)

# /Applications/IDE/LinkIT.app/Contents/Java/hardware/arduino/mtk/variants/linkit_one/libmtk.a
CORE_A   = $(VARIANT_PATH)/$(call PARSE_BOARD,$(BOARD_TAG),build.variant_system_lib)

# USB PID VID
#
USB_VID     := $(call PARSE_BOARD,$(BOARD_TAG),build.vid)
USB_PID     := $(call PARSE_BOARD,$(BOARD_TAG),build.pid)
USB_PRODUCT := $(call PARSE_BOARD,$(BOARD_TAG),build.usb_product)

USB_FLAGS    = -DUSB_VID=$(USB_VID)
USB_FLAGS   += -DUSB_PID=$(USB_PID)
USB_FLAGS   += -DUSBCON
USB_FLAGS   += -DUSB_MANUFACTURER='Unknown'
USB_FLAGS   += -DUSB_PRODUCT='$(USB_PRODUCT)'

# ~
ifeq ($(MAKECMDGOALS),debug)
    OPTIMISATION   ?= -O0 -g
else
    OPTIMISATION   ?= -Os
endif
# ~~


# Flags for gcc, g++ and linker
# ----------------------------------
#
# Common CPPFLAGS for gcc, g++, assembler and linker
#
CPPFLAGS     = -g $(OPTIMISATION) $(WARNING_FLAGS)
CPPFLAGS    += -fvisibility=hidden -fpic -mlittle-endian -nostdlib
# Solution 1
# $(call PARSE_BOARD,$(BOARD_TAG),build.extra_flags)
# -D__COMPILER_GCC__ -D__LINKIT_ONE__ -D__LINKIT_ONE_RELEASE__ -mthumb {build.usb_flags}
#CPPFLAGS    += $(addprefix -D, printf=iprintf __LINKIT_ONE_RELEASE__ __COMPILER_GCC__ ARDUINO_MTK_ONE)
#CPPFLAGS    += $(addprefix -D, printf=iprintf ARDUINO_MTK_ONE) $(lko02)
# Solution 2
CPPFLAGS    += $(addprefix -D, printf=iprintf __LINKIT_ONE_RELEASE__ __COMPILER_GCC__ ARDUINO_MTK_ONE)
#CPPFLAGS    += $(USB_FLAGS)
CPPFLAGS    += -$(MCU_FLAG_NAME)=$(MCU) -DF_CPU=$(F_CPU)
CPPFLAGS    += $(addprefix -D, $(PLATFORM_TAG) $(BUILD_BOARD))
CPPFLAGS    += $(addprefix -I, $(INCLUDE_PATH))



# Specific CFLAGS for gcc only
# gcc uses CPPFLAGS and CFLAGS
#
CFLAGS       = -mthumb
# was -std=c99

# Specific CXXFLAGS for g++ only
# g++ uses CPPFLAGS and CXXFLAGS
#
CXXFLAGS     = -mthumb -fno-non-call-exceptions -fno-rtti -fno-exceptions

# Specific ASFLAGS for gcc assembler only
# gcc assembler uses CPPFLAGS and ASFLAGS
#
ASFLAGS      = -x assembler-with-cpp

# Specific LDFLAGS for linker only
# linker uses CPPFLAGS and LDFLAGS
#
LDFLAGS      = $(OPTIMISATION) $(WARNING_FLAGS)
LDFLAGS     += -$(MCU_FLAG_NAME)=$(MCU) -DF_CPU=$(F_CPU)
LDFLAGS     += -T $(VARIANT_PATH)/$(call PARSE_BOARD,$(BOARD_TAG),build.ldscript)

LDFLAGS     += -Wl,--gc-sections -Wl,--entry=gcc_entry -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-unresolved-symbols


# Specific OBJCOPYFLAGS for objcopy only
# objcopy uses OBJCOPYFLAGS only
#
OBJCOPYFLAGS  = $(call PARSE_BOARD,$(BOARD_TAG),build.flash_mode)

# Target
#
TARGET_HEXBIN = $(TARGET_VXP)


# Commands
# ----------------------------------
# Link command
#
COMMAND_LINK    = $(CXX) $(LDFLAGS) $(OUT_PREPOSITION)$@ -Wl,--start-group $(LOCAL_OBJS) $(LOCAL_ARCHIVES) $(CORE_A) $(TARGET_A) -Wl,--end-group -LBuilds -lm -fpic -pie

# Upload command
#
COMMAND_UPLOAD  = $(UPLOADER_EXEC) $(UPLOADER_OPTS) -b $(USED_SERIAL_PORT) -p $(TARGET_VXP)
