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
# Last update: Aug 26, 2016 release 5.1.9
#
# Tested with Arduino/Genuino 101 CurieBLE Heart Rate Monitor
# https://www.arduino.cc/en/Tutorial/Genuino101CurieBLEHeartRateMonitor




include $(MAKEFILE_PATH)/About.mk

# Intel Curie specifics
# ----------------------------------
#
PLATFORM         := IntelArduino
BUILD_CORE       := Curie
PLATFORM_TAG      = ARDUINO=10610 __ARDUINO_ARC__ EMBEDXCODE=$(RELEASE_NOW)
APPLICATION_PATH := $(INTEL_PATH)
PLATFORM_VERSION := $(BUILD_CORE) $(INTEL_CURIE_RELEASE) for Arduino $(ARDUINO_CC_RELEASE)

HARDWARE_PATH     = $(APPLICATION_PATH)/hardware/arc32/$(INTEL_CURIE_RELEASE)
TOOL_CHAIN_PATH   = $(APPLICATION_PATH)/tools/arc-elf32/$(INTEL_ARC_RELEASE)/bin
OTHER_TOOLS_PATH  = $(APPLICATION_PATH)/tools/arduino101load/$(INTEL_CURIE_UPLOAD_RELEASE)

APP_TOOLS_PATH   := $(TOOL_CHAIN_PATH)
CORE_LIB_PATH    := $(HARDWARE_PATH)/cores/arduino
APP_LIB_PATH     := $(HARDWARE_PATH)/libraries
BOARDS_TXT       := $(HARDWARE_PATH)/boards.txt

# Version check
#
#w001 = $(APPLICATION_PATH)/lib/version.txt
#VERSION_CHECK = $(shell if [ -f $(w001) ] ; then cat $(w001) ; fi)
#ifneq ($(VERSION_CHECK),1.6.0+Intel)
#    $(error Intel Arduino IDE release 1.6.0 required.)
#endif

# Uploader
#
UPLOADER         = arduino101load
UPLOADER_PATH    = $(OTHER_TOOLS_PATH)/arduino101load
UPLOADER_EXEC    = $(UPLOADER_PATH)/arduino101load
UPLOADER_OPTS    = $(OTHER_TOOLS_PATH)/x86/bin

# Sketchbook/Libraries path
# wildcard required for ~ management
# ?ibraries required for libraries and Libraries
#
ifeq ($(USER_LIBRARY_DIR)/Arduino15/preferences.txt,)
    $(error Error: run Arduino once and define the sketchbook path)
endif

ifeq ($(wildcard $(SKETCHBOOK_DIR)),)
    SKETCHBOOK_DIR = $(shell grep sketchbook.path $(USER_LIBRARY_DIR)/Arduino15/preferences.txt | cut -d = -f 2)
endif

ifeq ($(wildcard $(SKETCHBOOK_DIR)),)
   $(error Error: sketchbook path not found)
endif

USER_LIB_PATH  = $(wildcard $(SKETCHBOOK_DIR)/?ibraries)

# Rules for making a c++ file from the main sketch (.pde)
#
PDEHEADER      = \\\#include \"Arduino.h\"

# Tool-chain names
#
CC      = $(APP_TOOLS_PATH)/arc-elf32-gcc
CXX     = $(APP_TOOLS_PATH)/arc-elf32-g++
AR      = $(APP_TOOLS_PATH)/arc-elf32-ar
OBJDUMP = $(APP_TOOLS_PATH)/arc-elf32-objdump
OBJCOPY = $(APP_TOOLS_PATH)/arc-elf32-objcopy
SIZE    = $(APP_TOOLS_PATH)/arc-elf32-size
NM      = $(APP_TOOLS_PATH)/arc-elf32-nm
STRIP   = $(APP_TOOLS_PATH)/arc-elf32-strip
# ~
GDB     = $(APP_TOOLS_PATH)/arc-elf32-gdb
# ~~

# Specific AVRDUDE location and options
#
#AVRDUDE_COM_OPTS  = -D -p$(MCU) -C$(AVRDUDE_CONF)

BOARD    = $(call PARSE_BOARD,$(BOARD_TAG),board)
LDSCRIPT = $(call PARSE_BOARD,$(BOARD_TAG),build.ldscript)
VARIANT  = $(call PARSE_BOARD,$(BOARD_TAG),build.variant)
VARIANT_PATH = $(HARDWARE_PATH)/variants/$(VARIANT)
VARIANT_CPP_SRCS  = $(wildcard $(VARIANT_PATH)/*.cpp) # */  $(VARIANT_PATH)/*/*.cpp #*/
VARIANT_OBJ_FILES = $(VARIANT_CPP_SRCS:.cpp=.cpp.o)
VARIANT_OBJS      = $(patsubst $(APPLICATION_PATH)/%,$(OBJDIR)/%,$(VARIANT_OBJ_FILES))

#SYSTEM_LIB  = $(call PARSE_BOARD,$(BOARD_TAG),build.variant_system_lib)
SYSTEM_PATH = $(VARIANT_PATH)
SYSTEM_OBJS = $(SYSTEM_PATH)/$(SYSTEM_LIB)


# Two locations for Arduino libraries
# The libraries USBHost WiFi SD Servo Ethernet are duplicated
# ../src/internal folders are requried for BLE
#
APP_LIB_PATH_1    = $(HARDWARE_PATH)/libraries
APP_LIB_PATH_2    = $(APPLICATION_PATH)/libraries
APP_LIB_PATH      = $(APP_LIB_PATH_1) $(APP_LIB_PATH_2)

APP_LIBS_LIST_1   = $(APP_LIBS_LIST)
APP_LIBS_LIST_2   = $(filter-out USBHost WiFi SD Servo Ethernet,$(APP_LIBS_LIST))

a1001    = $(foreach dir,$(APP_LIB_PATH_1),$(patsubst %,$(dir)/%,$(APP_LIBS_LIST_1)))
a1001   += $(foreach dir,$(APP_LIB_PATH_1),$(patsubst %,$(dir)/%/utility,$(APP_LIBS_LIST_1)))
a1001   += $(foreach dir,$(APP_LIB_PATH_1),$(patsubst %,$(dir)/%/src,$(APP_LIBS_LIST_1)))
a1001   += $(foreach dir,$(APP_LIB_PATH_1),$(patsubst %,$(dir)/%/src/utility,$(APP_LIBS_LIST_1)))
a1001   += $(foreach dir,$(APP_LIB_PATH_1),$(patsubst %,$(dir)/%/src/internal,$(APP_LIBS_LIST_1)))
a1001   += $(foreach dir,$(APP_LIB_PATH_1),$(patsubst %,$(dir)/%/src/arch/arc32,$(APP_LIBS_LIST_1)))

a1002    = $(foreach dir,$(APP_LIB_PATH_2),$(patsubst %,$(dir)/%,$(APP_LIBS_LIST_2)))
a1002   += $(foreach dir,$(APP_LIB_PATH_2),$(patsubst %,$(dir)/%/utility,$(APP_LIBS_LIST_2)))
a1002   += $(foreach dir,$(APP_LIB_PATH_2),$(patsubst %,$(dir)/%/src,$(APP_LIBS_LIST_2)))
a1002   += $(foreach dir,$(APP_LIB_PATH_2),$(patsubst %,$(dir)/%/src/utility,$(APP_LIBS_LIST_2)))
a1002   += $(foreach dir,$(APP_LIB_PATH_2),$(patsubst %,$(dir)/%/src/internal,$(APP_LIBS_LIST_2)))
a1002   += $(foreach dir,$(APP_LIB_PATH_2),$(patsubst %,$(dir)/%/src/arch/arc32,$(APP_LIBS_LIST_2)))

APP_LIB_CPP_SRC = $(foreach dir,$(a1001) $(a1002),$(wildcard $(dir)/*.cpp)) # */
APP_LIB_C_SRC   = $(foreach dir,$(a1001) $(a1002),$(wildcard $(dir)/*.c)) # */
APP_LIB_H_SRC   = $(foreach dir,$(a1001) $(a1002),$(wildcard $(dir)/*.h)) # */

APP_LIB_OBJS     = $(patsubst $(APPLICATION_PATH)/%.cpp,$(OBJDIR)/%.cpp.o,$(APP_LIB_CPP_SRC))
APP_LIB_OBJS    += $(patsubst $(APPLICATION_PATH)/%.c,$(OBJDIR)/%.c.o,$(APP_LIB_C_SRC))

BUILD_APP_LIBS_LIST = $(subst $(BUILD_APP_LIB_PATH)/, ,$(APP_LIB_CPP_SRC))

APP_LIBS_LOCK = 1


# MCU options
#
MCU_FLAG_NAME   = m
MCU             = $(call PARSE_BOARD,$(BOARD_TAG),build.mcu)

# Intel Curie / Arduino 101 USB PID VID
#
USB_VID     := $(call PARSE_BOARD,$(BOARD_TAG),build.vid)
USB_PID     := $(call PARSE_BOARD,$(BOARD_TAG),build.pid)
USB_PRODUCT := $(call PARSE_BOARD,$(BOARD_TAG),build.usb_product)

USB_FLAGS    = -DUSB_VID=$(USB_VID)
USB_FLAGS   += -DUSB_PID=$(USB_PID)
#USB_FLAGS   += -DUSBCON
USB_FLAGS   += -DUSB_MANUFACTURER=''
USB_FLAGS   += -DUSB_PRODUCT='$(USB_PRODUCT)'

# Intel Curie / Arduino 101 serial 1200 reset
#
USB_TOUCH := $(call PARSE_BOARD,$(BOARD_TAG),upload.protocol)
USB_RESET  = python $(UTILITIES_PATH)/reset_1200.py

# ~
ifeq ($(MAKECMDGOALS),debug)
    OPTIMISATION   ?= -O0 -g
else
    OPTIMISATION   ?= -Os
endif
# ~~

# Include paths
#
INCLUDE_PATH     = $(CORE_LIB_PATH) $(VARIANT_PATH)
INCLUDE_PATH    += $(sort $(dir $(APP_LIB_CPP_SRC) $(APP_LIB_C_SRC) $(APP_LIB_H_SRC)))
INCLUDE_PATH    += $(HARDWARE_PATH)/system/libarc32_arduino101/common
INCLUDE_PATH    += $(HARDWARE_PATH)/system/libarc32_arduino101/drivers
INCLUDE_PATH    += $(HARDWARE_PATH)/system/libarc32_arduino101/bootcode
INCLUDE_PATH    += $(HARDWARE_PATH)/system/libarc32_arduino101/framework/include

D_FLAGS = ARDUINO_ARC32_TOOLS ARDUINO_ARCH_ARC32 __CPU_ARC__ CLOCK_SPEED=32 CONFIG_SOC_GPIO_32 CONFIG_SOC_GPIO_AON INFRA_MULTI_CPU_SUPPORT CFW_MULTI_CPU_SUPPORT HAS_SHARED_MEM

# Flags for gcc, g++ and linker
# ----------------------------------
#
# Common CPPFLAGS for gcc, g++, assembler and linker
#
CPPFLAGS     = $(OPTIMISATION) $(WARNING_FLAGS)
CPPFLAGS    += -DF_CPU=$(call PARSE_BOARD,$(BOARD_TAG),build.f_cpu) -$(MCU_FLAG_NAME)$(MCU)
CPPFLAGS    += -mav2em -mlittle-endian
CPPFLAGS    += -Wall -fno-reorder-functions -fno-asynchronous-unwind-tables -fno-omit-frame-pointer -fno-defer-pop -Wno-unused-but-set-variable -Wno-main -ffreestanding -fno-stack-protector -mno-sdata -ffunction-sections -fdata-sections -fsigned-char
CPPFLAGS    += $(addprefix -D, $(PLATFORM_TAG) $(D_FLAGS))
CPPFLAGS    += $(addprefix -I, $(INCLUDE_PATH))

# Specific CFLAGS for gcc only
# gcc uses CPPFLAGS and CFLAGS
#
CFLAGS       =

# Specific CXXFLAGS for g++ only
# g++ uses CPPFLAGS and CXXFLAGS
#
CXXFLAGS     = -fno-rtti -fno-exceptions -std=c++11

# Specific ASFLAGS for gcc assembler only
# gcc assembler uses CPPFLAGS and ASFLAGS
#
ASFLAGS      = -Xassembler

# Specific LDFLAGS for linker only
# linker uses CPPFLAGS and LDFLAGS
#
LDFLAGS      = $(OPTIMISATION) $(WARNING_FLAGS)
LDFLAGS     += -DF_CPU=$(call PARSE_BOARD,$(BOARD_TAG),build.f_cpu) -$(MCU_FLAG_NAME)$(MCU)
LDFLAGS     += -nostartfiles -nodefaultlibs -nostdlib -static -Wl,-X -Wl,-N -Wl,-mARCv2EM -Wl,-marcelf -Wl,--gc-sections
LDFLAGS     += -T $(VARIANT_PATH)/$(LDSCRIPT)
LDFLAGS     += -L$(OBJDIR) -L$(VARIANT_PATH)
LDFLAGS     += -Wl,--whole-archive -larc32drv_arduino101 -Wl,--no-whole-archive -lnsim

LDOBJECTS    = -Wl,--start-group -lnsim -lc -lm -lgcc $(LOCAL_OBJS) $(TARGET_A) -Wl,--end-group

# Specific OBJCOPYFLAGS for objcopy only
# objcopy uses OBJCOPYFLAGS only
#
OBJCOPYFLAGS  = -v -Obinary

# Target
#
TARGET_HEXBIN = $(TARGET_BIN)


# Commands
# ----------------------------------
# Link command
#
COMMAND_LINK    = $(CC) $(LDFLAGS) $(OUT_PREPOSITION)$@ $(LDOBJECTS)

# Copy command
#
COMMAND_COPY    = $(OBJCOPY) -S -O binary -R .note -R .comment -R COMMON -R .eh_frame $< $@

# Upload command
#
COMMAND_UPLOAD  = $(UPLOADER_EXEC) $(UPLOADER_OPTS) $(TARGET_BIN) $(USED_SERIAL_PORT) verbose ; sleep 5
