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
# Last update: Jun 22, 2016 release 5.0.5


include $(MAKEFILE_PATH)/About.mk

# RedBearLab Duo specifics
# ----------------------------------
#
PLATFORM         := RedBearLab
PLATFORM_TAG      = ARDUINO=10610 EMBEDXCODE=$(RELEASE_NOW) REDBEARLAB_DUO
APPLICATION_PATH := $(REDBEARLAB_DUO_PATH)
PLATFORM_VERSION := Duo $(REDBEARLAB_DUO_RELEASE) for Arduino $(ARDUINO_CC_RELEASE)

HARDWARE_PATH     = $(APPLICATION_PATH)/hardware/STM32F2/$(REDBEARLAB_DUO_RELEASE)

# Arduino IDE / RedBear Duo Conflict
# ----------------------------------
# http://discuss.redbear.cc/t/arduino-zero-redbear-duo-conflict/274
# http://forum.arduino.cc/index.php?topic=391429.msg2698429
#
# 1. Rename folder
#   ~/Library/Arduino15/packages/RedBear/tools/arm-none-eabi-gcc
# to
#   ~/Library/Arduino15/packages/RedBear/tools/arm-none-eabi-gcc-redbear
# 2.Edit
#   ~/Library/Arduino15/packages/RedBear/hardware/STM32F2/0.2.7/platform.txt line 8
# and change
#   compiler.path={runtime.tools.arm-none-eabi-gcc-4.9-2015-q3.path}/bin/
# to
#   compiler.path={runtime.tools.arm-none-eabi-gcc-redbear-4.9-2015-q3.path}/bin/
#
ifneq ($(wildcard $(APPLICATION_PATH)/tools/arm-none-eabi-gcc/*),) # */
    TOOL_CHAIN_PATH   = $(APPLICATION_PATH)/tools/arm-none-eabi-gcc/$(DUO_GCC_ARM_RELEASE)
else
    TOOL_CHAIN_PATH   = $(APPLICATION_PATH)/tools/arm-none-eabi-gcc-redbear/$(DUO_GCC_ARM_RELEASE)
endif
OTHER_TOOLS_PATH  = $(APPLICATION_PATH)/tools/bossac/1.3a-arduino

BUILD_CORE       = RedBear_Duo
BOARDS_TXT      := $(HARDWARE_PATH)/boards.txt
BUILD_CORE       = $(call PARSE_BOARD,$(BOARD_TAG),build.core)

# Uploader
#
ifeq ($(UPLOADER),openocd)
    UPLOADER         = openocd
    UPLOADER_PATH    = $(ARDUINO_SAMD_PATH)/tools/openocd/$(OPENOCD_RELEASE)
    UPLOADER_EXEC    = $(UPLOADER_PATH)/bin/openocd
    UPLOADER_OPTS    = -d1 -s $(UPLOADER_PATH)/share/openocd/scripts/
    UPLOADER_OPTS   += -f $(VARIANT_PATH)/$(call PARSE_BOARD,$(BOARD_TAG),build.openocdscript)
    UPLOADER_COMMAND = -c program {{$(TARGET_BIN)}} verify reset exit 0x80c0000
    COMMAND_UPLOAD   = $(UPLOADER_EXEC) $(UPLOADER_OPTS) "$(UPLOADER_COMMAND)"

    PREPARE_PATH     = $(HARDWARE_PATH)/tools/crc32/macosx
    PREPARE_EXEC     = $(PREPARE_PATH)/sh $(PREPARE_PATH)/crc32.sh
    PREPARE_OPTS     = $(TARGET_BIN) $(PREPARE_PATH) $(TOOL_CHAIN_PATH)/bin
    COMMAND_PREPARE  = $(PREPARE_EXEC) $(PREPARE_OPTS)

else
    UPLOADER             = avrdude
    AVRDUDE_PATH        := $(ARDUINO_PATH)/hardware/tools/avr
    AVRDUDE_EXEC        := $(AVRDUDE_PATH)/avrdude
    AVRDUDE_CONF         = $(HARDWARE_PATH)/avrdude_conf/avrdude.conf
    AVRDUDE_COM_OPTS     = -p$(AVRDUDE_MCU) -C$(AVRDUDE_CONF)

    PREPARE_PATH     = $(HARDWARE_PATH)/tools/crc32/macosx
    PREPARE_EXEC     = $(PREPARE_PATH)/sh $(PREPARE_PATH)/crc32.sh
    PREPARE_OPTS     = $(TARGET_BIN) $(PREPARE_PATH) $(TOOL_CHAIN_PATH)/bin
    COMMAND_PREPARE  = $(PREPARE_EXEC) $(PREPARE_OPTS)
endif

APP_TOOLS_PATH   := $(TOOL_CHAIN_PATH)/bin
CORE_LIB_PATH    := $(HARDWARE_PATH)/cores/RedBear_Duo
APP_LIB_PATH     := $(HARDWARE_PATH)/libraries

#BUILD_CORE_LIB_PATH  = $(HARDWARE_PATH)/cores/RBL_nRF51822
#BUILD_CORE_LIBS_LIST = $(subst .h,,$(subst $(BUILD_CORE_LIB_PATH)/,,$(wildcard $(BUILD_CORE_LIB_PATH)/*.h))) # */
#BUILD_CORE_C_SRCS    = $(wildcard $(BUILD_CORE_LIB_PATH)/*.c) # */

#BUILD_CORE_CPP_SRCS  = $(filter-out %program.cpp %main.cpp,$(wildcard $(BUILD_CORE_LIB_PATH)/*.cpp)) # */

#BUILD_CORE_OBJ_FILES = $(BUILD_CORE_C_SRCS:.c=.c.o) $(BUILD_CORE_CPP_SRCS:.cpp=.cpp.o)
#BUILD_CORE_OBJS      = $(patsubst $(APPLICATION_PATH)/%,$(OBJDIR)/%,$(BUILD_CORE_OBJ_FILES))


# Core files
# Crazy maze of sub-folders
#
CORE_C_SRCS          = $(shell find $(CORE_LIB_PATH) -name \*.c)
rbd1300              = $(filter-out %main.cpp, $(shell find $(CORE_LIB_PATH) -name \*.cpp))
CORE_CPP_SRCS        = $(filter-out %/$(EXCLUDE_LIST),$(rbd1300))
CORE_AS1_SRCS        = $(shell find $(CORE_LIB_PATH) -name \*.S)
CORE_AS1_SRCS_OBJ    = $(patsubst %.S,%.S.o,$(filter %.S, $(CORE_AS1_SRCS)))
CORE_AS2_SRCS        = $(shell find $(CORE_LIB_PATH) -name \*.s)
CORE_AS2_SRCS_OBJ    = $(patsubst %.s,%.s.o,$(filter %.s, $(CORE_AS_SRCS)))

CORE_OBJ_FILES       = $(CORE_C_SRCS:.c=.c.o) $(CORE_CPP_SRCS:.cpp=.cpp.o) $(CORE_AS1_SRCS_OBJ) $(CORE_AS2_SRCS_OBJ)
CORE_OBJS            = $(patsubst $(APPLICATION_PATH)/%,$(OBJDIR)/%,$(CORE_OBJ_FILES))

CORE_LIBS_LOCK       = 1

# Two locations for RedBearLabs libraries
#
APP_LIB_PATH     := $(HARDWARE_PATH)/libraries

rbd1000    = $(foreach dir,$(APP_LIB_PATH),$(patsubst %,$(dir)/%,$(APP_LIBS_LIST)))
rbd1000   += $(foreach dir,$(APP_LIB_PATH),$(patsubst %,$(dir)/%/utility,$(APP_LIBS_LIST)))
rbd1000   += $(foreach dir,$(APP_LIB_PATH),$(patsubst %,$(dir)/%/src,$(APP_LIBS_LIST)))
rbd1000   += $(foreach dir,$(APP_LIB_PATH),$(patsubst %,$(dir)/%/src/utility,$(APP_LIBS_LIST)))
rbd1000   += $(foreach dir,$(APP_LIB_PATH),$(patsubst %,$(dir)/%/src/arch/$(BUILD_CORE),$(APP_LIBS_LIST)))
rbd1000   += $(foreach dir,$(APP_LIB_PATH),$(patsubst %,$(dir)/%/src/$(BUILD_CORE),$(APP_LIBS_LIST)))
rbd1000   += $(HARDWARE_PATH)/libraries/RedBear_Duo/src
rbd1000   += $(HARDWARE_PATH)/libraries/RedBear_Duo/src/include
rbd1000   += $(HARDWARE_PATH)/libraries/RedBear_Duo/src/src
rbd1000   += $(HARDWARE_PATH)/libraries/RedBear_Duo/src/src/Internals
rbd1000   += $(HARDWARE_PATH)/libraries/RedBear_Duo/src/utility

APP_LIB_CPP_SRC = $(foreach dir,$(rbd1000),$(wildcard $(dir)/*.cpp)) # */
APP_LIB_C_SRC   = $(foreach dir,$(rbd1000),$(wildcard $(dir)/*.c)) # */
APP_LIB_S_SRC   = $(foreach dir,$(rbd1000),$(wildcard $(dir)/*.S)) # */
APP_LIB_IPP_SRC = $(foreach dir,$(rbd1000),$(wildcard $(dir)/*.ipp)) # */
APP_LIB_H_SRC   = $(foreach dir,$(rbd1000),$(wildcard $(dir)/*.h)) # */

APP_LIB_OBJS     = $(patsubst $(APPLICATION_PATH)/%.cpp,$(OBJDIR)/%.cpp.o,$(APP_LIB_CPP_SRC))
APP_LIB_OBJS    += $(patsubst $(APPLICATION_PATH)/%.c,$(OBJDIR)/%.c.o,$(APP_LIB_C_SRC))

BUILD_APP_LIBS_LIST = $(subst $(BUILD_APP_LIB_PATH)/, ,$(APP_LIB_CPP_SRC))

APP_LIBS_LOCK = 1


# Sketchbook/Libraries path
# wildcard required for ~ management
# ?ibraries required for libraries and Libraries
#
ifeq ($(USER_LIBRARY_DIR)/Arduino15/preferences.txt,)
    $(error Error: run Arduino or panStamp once and define the sketchbook path)
endif

ifeq ($(wildcard $(SKETCHBOOK_DIR)),)
    SKETCHBOOK_DIR = $(shell grep sketchbook.path $(wildcard ~/Library/Arduino15/preferences.txt) | cut -d = -f 2)
endif

ifeq ($(wildcard $(SKETCHBOOK_DIR)),)
    $(error Error: sketchbook path not found)
endif

USER_LIB_PATH   = $(wildcard $(SKETCHBOOK_DIR)/?ibraries)

VARIANT         = $(call PARSE_BOARD,$(BOARD_TAG),build.variant)
VARIANT_PATH    = $(HARDWARE_PATH)/variants/$(VARIANT)
LDSCRIPT_PATH   = $(VARIANT_PATH)
LDSCRIPT        = $(VARIANT_PATH)/$(call PARSE_BOARD,$(BOARD_TAG),build.ldscript)

# Rules for making a c++ file from the main sketch (.pde)
#
PDEHEADER      = \\\#include \"WProgram.h\"  


# Tool-chain names
#
CC      = $(APP_TOOLS_PATH)/arm-none-eabi-gcc
CXX     = $(APP_TOOLS_PATH)/arm-none-eabi-g++
AR      = $(APP_TOOLS_PATH)/arm-none-eabi-ar
OBJDUMP = $(APP_TOOLS_PATH)/arm-none-eabi-objdump
OBJCOPY = $(APP_TOOLS_PATH)/arm-none-eabi-objcopy
SIZE    = $(APP_TOOLS_PATH)/arm-none-eabi-size
NM      = $(APP_TOOLS_PATH)/arm-none-eabi-nm


MCU_FLAG_NAME    = mcpu
MCU              = $(call PARSE_BOARD,$(BOARD_TAG),build.mcu)
F_CPU            = $(call PARSE_BOARD,$(BOARD_TAG),build.f_cpu)
OPTIMISATION    ?= -Os -g3

INCLUDE_PATH     = $(CORE_LIB_PATH) $(APP_LIB_PATH) $(VARIANT_PATH) $(HARDWARE_PATH)
INCLUDE_PATH    += $(sort $(dir $(APP_LIB_CPP_SRC) $(APP_LIB_C_SRC) $(APP_LIB_H_SRC) $(APP_LIB_IPP_SRC)))
INCLUDE_PATH    += $(sort $(dir $(BUILD_APP_LIB_CPP_SRC) $(BUILD_APP_LIB_C_SRC) $(BUILD_APP_LIB_H_SRC) $(BUILD_APP_LIB_IPP_SRC)))
INCLUDE_PATH    += $(OBJDIR)

rbd1200            = $(call PARSE_BOARD,$(BOARD_TAG),build.ble_api_include)
rbd1200           += $(call PARSE_BOARD,$(BOARD_TAG),build.nRF51822_api_include)
rbd1200           += $(call PARSE_BOARD,$(BOARD_TAG),build.mbed_api_include)
rbd1210            = $(shell echo $(rbd1200) | sed 's/-I{runtime.platform.path}//g')
rbd1220            = $(addprefix $(HARDWARE_PATH),$(rbd1210))

INCLUDE_PATH    += $(rbd1220) .
INCLUDE_PATH    += $(sort $(shell find $(HARDWARE_PATH)/system -type d))
INCLUDE_PATH    += $(sort $(shell find $(CORE_LIB_PATH) -type d))
INCLUDE_PATH    += $(sort $(shell find $(APP_LIB_PATH) -type d | grep -v /examples))
INCLUDE_PATH    += $(sort $(shell find $(VARIANT_PATH) -type d))

#rbd2000         = $(call PARSE_BOARD,$(BOARD_TAG),variant_base_include)
#rbd2000        += $(call PARSE_BOARD,$(BOARD_TAG),build.variant_extra_include)
#rbd2010         = $(shell echo '$(rbd2000)' | sed 's/-I{build.core.path}//g' | sed 's/\"//g')
#rbd2020         = $(addprefix -I$(CORE_LIB_PATH),$(rbd2010))

D_FLAGS         = printf=iprintf
D_FLAGS        += STM32_DEVICE STM32F2XX PLATFORM_THREADING=1 PLATFORM_ID=88 PLATFORM_NAME=duo
D_FLAGS        += USBD_VID_SPARK=0x2B04 USBD_PID_DFU=0xD058 USBD_PID_CDC=0xC058
D_FLAGS        += START_DFU_FLASHER_SERIAL_SPEED=14400 START_YMODEM_FLASHER_SERIAL_SPEED=28800
D_FLAGS        += START_AVRDUDE_FLASHER_SERIAL_SPEED=19200 RELEASE_BUILD INCLUDE_PLATFORM=1
D_FLAGS        += USE_STDPERIPH_DRIVER DFU_BUILD_ENABLE USER_FIRMWARE_IMAGE_SIZE=0x40000
D_FLAGS        += USER_FIRMWARE_IMAGE_LOCATION=0x80C0000 SYSTEM_VERSION_STRING=0.2.4
D_FLAGS        += MODULAR_FIRMWARE=1 MODULE_FUNCTION=5 MODULE_INDEX=1 MODULE_VERSION=7
D_FLAGS        += MODULE_DEPENDENCY=4,2,7
D_FLAGS        += MBED_BUILD_TIMESTAMP=$(shell date +%s)


# Flags for gcc, g++ and linker
# ----------------------------------
#
# Common CPPFLAGS for gcc, g++, assembler and linker
#
CPPFLAGS     = $(OPTIMISATION) $(WARNING_FLAGS)  # -w
CPPFLAGS    += -gdwarf-2 -fno-common -fmessage-length=0 -Wall
CPPFLAGS    += -fno-exceptions -fno-builtin-malloc -fno-builtin-free -fno-builtin-realloc
CPPFLAGS    += -ffunction-sections -fdata-sections -fomit-frame-pointer -nostdlib
CPPFLAGS    += --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -mthumb
CPPFLAGS    += -$(MCU_FLAG_NAME)=$(MCU) -DF_CPU=$(F_CPU)
CPPFLAGS    += $(addprefix -D, $(PLATFORM_TAG) $(D_FLAGS))
CPPFLAGS    += $(addprefix -I, $(INCLUDE_PATH))

# Specific CFLAGS for gcc only
# gcc uses CPPFLAGS and CFLAGS
#
CFLAGS       = -std=gnu99

# Specific CXXFLAGS for g++ only
# g++ uses CPPFLAGS and CXXFLAGS
#
CXXFLAGS     = -std=gnu++11

# Specific ASFLAGS for gcc assembler only
# gcc assembler uses CPPFLAGS and ASFLAGS
#
ASFLAGS      = -x assembler-with-cpp

# Specific LDFLAGS for linker only
# linker uses CPPFLAGS and LDFLAGS
#
LDFLAGS      = $(OPTIMISATION) $(WARNING_FLAGS)  # -w
LDFLAGS     += -$(MCU_FLAG_NAME)=$(MCU) -DF_CPU=$(F_CPU)
LDFLAGS     += -gdwarf-2 -mthumb -fno-builtin -Werror
LDFLAGS     += -ffunction-sections -fdata-sections -Wall -Wno-switch
LDFLAGS     += -Wno-error=deprecated-declarations -fmessage-length=0 -fno-strict-aliasing
LDFLAGS     += -fno-builtin-malloc -fno-builtin-free -fno-builtin-realloc
LDFLAGS     += $(addprefix -D, $(PLATFORM_TAG) $(D_FLAGS)) -DSPARK=1
LDFLAGS     += $(addprefix -I, $(INCLUDE_PATH))

LDFLAGS2    += -Wl,--whole-archive $$(find $(CORE_LIB_PATH) -name STM32F2xx_Peripheral_Libraries.a) -Wl,--no-whole-archive
LDFLAGS2    += -nostartfiles -Xlinker --gc-sections
LDFLAGS2    += -L$(VARIANT_PATH)/linker_scripts/linker
LDFLAGS2    += -L$(VARIANT_PATH)/linker_scripts/linker/stm32f2xx
LDFLAGS2    += -LBuilds
LDFLAGS2    += -L$(CORE_LIB_PATH)/redbear_sdk/X_lib

LDFLAGS3    += -Wl,--whole-archive
# libcore.a = $(TARGET_A) so -lcore = $(TARGET_A)
LDFLAGS3    += $(TARGET_A) -lhal-dynalib -lservices-dynalib -lsystem-dynalib
LDFLAGS3    += -lrt-dynalib -lwiring -lcommunication-dynalib -lplatform
LDFLAGS3    += -lwiring_globals
LDFLAGS3    += -Wl,--no-whole-archive
LDFLAGS3    += -lnosys
#LDFLAGS3    += -L$(VARIANT_PATH)/linker_scripts/gcc
LDFLAGS3    += -L$(VARIANT_PATH)/linker_scripts/gcc/duo/system-part1
LDFLAGS3    += -L$(VARIANT_PATH)/linker_scripts/gcc/duo/system-part2
LDFLAGS3    += -L$(VARIANT_PATH)/linker_scripts/gcc/duo/user-part
LDFLAGS3    += -L$(VARIANT_PATH)/linker_scripts/gcc/shared/stm32f2xx
LDFLAGS3    += -L.
LDFLAGS3    += -T $(LDSCRIPT)
LDFLAGS3    += -Wl,--defsym,USER_FIRMWARE_IMAGE_SIZE=0x40000
LDFLAGS3    += -Wl,--defsym,USER_FIRMWARE_IMAGE_LOCATION=0x80C0000
LDFLAGS3    += -lstdc++_nano -lm
LDFLAGS3    += -Wl,--start-group -lgcc -lg_nano -lc_nano -Wl,--end-group
LDFLAGS3    += -Wl,--start-group -lgcc -lc_nano -Wl,--end-group

# Specific OBJCOPYFLAGS for objcopy only
# objcopy uses OBJCOPYFLAGS only
#
OBJCOPYFLAGS  = -v -Obinary

# Target
#
TARGET_HEXBIN = $(TARGET_BIN)

# Serial 1200 reset
#
USB_TOUCH := $(call PARSE_BOARD,$(BOARD_TAG),upload.use_1200bps_touch)
ifeq ($(USB_TOUCH),true)
    USB_RESET  = python $(UTILITIES_PATH)/reset_1200.py
endif


# Commands
# ----------------------------------
# Link command
#
COMMAND_LINK    = $(CXX) $(LDFLAGS) $(LOCAL_OBJS) $(LOCAL_ARCHIVES) -o $(TARGET_ELF) $(LDFLAGS2) $(LDFLAGS3)

#COMMAND_UPLOAD  = $(AVRDUDE_EXEC) $(AVRDUDE_COM_OPTS) $(AVRDUDE_OPTS) -P$(USED_SERIAL_PORT) -Uflash:w:$(TARGET_HEX):i
