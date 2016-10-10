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
# Last update: Dec 05, 2015 release 4.0.6



# Energia LaunchPad C2000 specifics
# ----------------------------------
#
APPLICATION_PATH := $(ENERGIA_PATH)
ENERGIA_RELEASE  := $(shell tail -c2 $(APPLICATION_PATH)/lib/version.txt)
ARDUINO_RELEASE  := $(shell head -c4 $(APPLICATION_PATH)/lib/version.txt | tail -c3)

ifeq ($(shell if [[ '$(ENERGIA_RELEASE)' -ge '17' ]] ; then echo 1 ; else echo 0 ; fi ),0)
    WARNING_MESSAGE = Energia 17 or later is required.
endif

PLATFORM         := Energia
BUILD_CORE       := c2000
PLATFORM_TAG      = ENERGIA=$(ENERGIA_RELEASE) ARDUINO=$(ARDUINO_RELEASE) EMBEDXCODE=$(RELEASE_NOW) $(filter __%__ ,$(GCC_PREPROCESSOR_DEFINITIONS))

#UPLOADER        = serial_loader2000
#UPLOADER_PATH   = $(APPLICATION_PATH)/hardware/c2000/serial_loader2000/macos
#UPLOADER_EXEC   = $(UPLOADER_PATH)/serial_loader2000
#ifneq ($(filter __TMS320F28027__,$(GCC_PREPROCESSOR_DEFINITIONS)),)
#    UPLOADER_OPTS   = -k $(APPLICATION_PATH)/hardware/c2000/F28027_flash_kernel/Debug/flash_kernel.txt -b 38400
#endif
#ifneq ($(filter __TMS320F28069__,$(GCC_PREPROCESSOR_DEFINITIONS)),)
#    UPLOADER_OPTS   = -k $(APPLICATION_PATH)/hardware/c2000/F28069_flash_kernel/Debug/2806_flash_kernel.txt -b 38400
#endif
UPLOADER          = DSLite
UPLOADER_PATH     = $(APPLICATION_PATH)/tools/common/DSLite
UPLOADER_EXEC     = $(UPLOADER_PATH)/DebugServer/bin/DSLite
ifneq ($(filter __TMS320F28027__,$(GCC_PREPROCESSOR_DEFINITIONS)),)
    UPLOADER_OPTS     = $(UPLOADER_PATH)/LAUNCHXL-F28027.ccxml
endif
ifneq ($(filter __TMS320F28069__,$(GCC_PREPROCESSOR_DEFINITIONS)),)
    UPLOADER_OPTS     = $(UPLOADER_PATH)/LAUNCHXL-F28069M.ccxml
endif
ifneq ($(filter __TMS320F28377S__,$(GCC_PREPROCESSOR_DEFINITIONS)),)
    UPLOADER_OPTS     = $(UPLOADER_PATH)/LAUNCHXL-F28377S.ccxml
endif

UPLOADER_RESET  =
#RESET_MESSAGE   = 1


APP_TOOLS_PATH   := $(APPLICATION_PATH)/hardware/tools/c2000/bin
CORE_LIB_PATH    := $(APPLICATION_PATH)/hardware/c2000/cores/c2000
APP_LIB_PATH     := $(APPLICATION_PATH)/hardware/c2000/libraries
BOARDS_TXT       := $(APPLICATION_PATH)/hardware/c2000/boards.txt

BUILD_CORE_LIB_PATH  = $(APPLICATION_PATH)/hardware/c2000/cores/c2000

# Horrible patch for core libraries
# ----------------------------------
#
# If driverlib/libdriverlib.a is available, exclude driverlib/
#
#CORE_LIB_PATH  = $(APPLICATION_PATH)/hardware/c2000/cores/c2000
#
#CORE_A   = $(CORE_LIB_PATH)/driverlib/libdriverlib.a

#BUILD_CORE_LIB_PATH = $(shell find $(CORE_LIB_PATH) -type d)
#ifneq ($(wildcard $(CORE_A)),)
#    BUILD_CORE_LIB_PATH := $(filter-out %/driverlib,$(BUILD_CORE_LIB_PATH))
#endif

e1200         = $(BUILD_CORE_LIB_PATH)

ifneq ($(filter __TMS320F28027__,$(GCC_PREPROCESSOR_DEFINITIONS)),)
    e1200    += $(BUILD_CORE_LIB_PATH)/f2802x_common/source
    e1200    += $(BUILD_CORE_LIB_PATH)/f2802x_headers/source
endif
ifneq ($(filter __TMS320F28069__,$(GCC_PREPROCESSOR_DEFINITIONS)),)
    e1200    += $(BUILD_CORE_LIB_PATH)/F2806x_common/source
    e1200    += $(BUILD_CORE_LIB_PATH)/F2806x_headers/source
endif
ifneq ($(filter __TMS320F28377S__,$(GCC_PREPROCESSOR_DEFINITIONS)),)
    e1200    += $(BUILD_CORE_LIB_PATH)/F2837xS_common/source
    e1200    += $(BUILD_CORE_LIB_PATH)/F2837xS_headers/source
endif

BUILD_CORE_CPP_SRCS = $(filter-out %program.cpp %main.cpp,$(foreach dir,$(e1200),$(wildcard $(dir)/*.cpp))) # */
BUILD_CORE_C_SRCS   = $(foreach dir,$(e1200),$(wildcard $(dir)/*.c)) # */
BUILD_CORE_S_SRCS   = $(foreach dir,$(e1200),$(wildcard $(dir)/*.S)) # */

BUILD_CORE_OBJ_FILES  = $(BUILD_CORE_C_SRCS:.c=.c.o) $(BUILD_CORE_CPP_SRCS:.cpp=.cpp.o) $(BUILD_CORE_S_SRCS:.S=.S.o)
BUILD_CORE_OBJS       = $(patsubst $(APPLICATION_PATH)/%,$(OBJDIR)/%,$(BUILD_CORE_OBJ_FILES))

CORE_LIBS_LOCK = 1
# ----------------------------------

ifneq ($(filter __TMS320F28027__,$(GCC_PREPROCESSOR_DEFINITIONS)),)
    FIRST_O_IN_A            = $(filter %/F2802x_asmfuncs.S.o,$(BUILD_CORE_OBJS))
endif
ifneq ($(filter __TMS320F28069__,$(GCC_PREPROCESSOR_DEFINITIONS)),)
    FIRST_O_IN_A            = $(filter %/F2806x_CodeStartBranch.S.o,$(BUILD_CORE_OBJS))
endif
ifneq ($(filter __TMS320F28377S__,$(GCC_PREPROCESSOR_DEFINITIONS)),)
    FIRST_O_IN_A            = $(filter %/F2806x_asmfuncs.S.o,$(BUILD_CORE_OBJS))
endif


# Sketchbook/Libraries path
# wildcard required for ~ management
# ?ibraries required for libraries and Libraries
#
ifeq ($(USER_LIBRARY_DIR)/Energia/preferences.txt,)
$(error Error: run Energia once and define the sketchbook path)
endif

ifeq ($(wildcard $(SKETCHBOOK_DIR)),)
    SKETCHBOOK_DIR = $(shell grep sketchbook.path $(wildcard ~/Library/Energia/preferences.txt) | cut -d = -f 2)
endif

ifeq ($(wildcard $(SKETCHBOOK_DIR)),)
    $(error Error: sketchbook path not found)
endif

USER_LIB_PATH  = $(wildcard $(SKETCHBOOK_DIR)/?ibraries)


# Rules for making a c++ file from the main sketch (.pde)
#
PDEHEADER      = \\\#include \"Energia.h\"


# Tool-chain names
#
CC      = $(APP_TOOLS_PATH)/cl2000
CXX     = $(APP_TOOLS_PATH)/cl2000
AR      = $(APP_TOOLS_PATH)/ar2000
#OBJDUMP = $(APP_TOOLS_PATH)/arm-none-eabi-objdump
OBJCOPY = $(APP_TOOLS_PATH)/hex2000
#SIZE    = $(APP_TOOLS_PATH)/arm-none-eabi-size
NM      = $(APP_TOOLS_PATH)/nm2000
# ~
#GDB     = $(APP_TOOLS_PATH)/arm-none-eabi-gdb
# ~~

BOARD    = $(call PARSE_BOARD,$(BOARD_TAG),board)
#LDSCRIPT = $(call PARSE_BOARD,$(BOARD_TAG),ldscript)
VARIANT  = $(call PARSE_BOARD,$(BOARD_TAG),build.variant)
VARIANT_PATH = $(APPLICATION_PATH)/hardware/c2000/variants/$(VARIANT)

LDSCRIPT = $(call PARSE_BOARD,$(BOARD_TAG),build.rts)
# Run-Time Support Library


# ~
ifeq ($(MAKECMDGOALS),debug)
    OPTIMISATION   ?= -o0 -g -mn
else
    OPTIMISATION   ?= -o3
endif
# ~~

OUT_PREPOSITION = --output_file=

INCLUDE_PATH    := $(CORE_LIB_PATH) $(VARIANT_PATH)
INCLUDE_PATH    += $(APPLICATION_PATH)/hardware/tools/c2000
INCLUDE_PATH    += $(APPLICATION_PATH)/hardware/tools/c2000/include
ifneq ($(filter __TMS320F28027__,$(GCC_PREPROCESSOR_DEFINITIONS)),)
    INCLUDE_PATH    += $(BUILD_CORE_LIB_PATH)/f2802x_common/include
    INCLUDE_PATH    += $(BUILD_CORE_LIB_PATH)/f2802x_headers/include
endif
ifneq ($(filter __TMS320F28069__,$(GCC_PREPROCESSOR_DEFINITIONS)),)
    INCLUDE_PATH    += $(BUILD_CORE_LIB_PATH)/F2806x_common/include
    INCLUDE_PATH    += $(BUILD_CORE_LIB_PATH)/F2806x_headers/include
endif
ifneq ($(filter __TMS320F28377S__,$(GCC_PREPROCESSOR_DEFINITIONS)),)
    INCLUDE_PATH    += $(BUILD_CORE_LIB_PATH)/F2837xS_common/include
    INCLUDE_PATH    += $(BUILD_CORE_LIB_PATH)/F2837xS_headers/include
endif

# Path for Energia 16
INCLUDE_PATH    += $(APPLICATION_PATH)/hardware/tools/c2000/lib
# Compatibility for Energia < 16
INCLUDE_PATH    += $(APPLICATION_PATH)/hardware/tools/lib


# Flags for gcc, g++ and linker
# ----------------------------------
#
# Common CPPFLAGS for gcc, g++, assembler and linker
#
CPPFLAGS     = -v28 -ml -mt -g
CPPFLAGS    += $(addprefix --define=,F_CPU=$(F_CPU) $(MCU))
CPPFLAGS    += --gcc --diag_warning=225 --gen_func_subsections=on
CPPFLAGS    += --display_error_number --diag_wrap=off --preproc_with_compile
# --preproc_dependency

# Specific CFLAGS for gcc only
# gcc uses CPPFLAGS and CFLAGS
#
CFLAGS       = $(addprefix --define=, $(PLATFORM_TAG))
CFLAGS      += $(addprefix --include_path=,$(INCLUDE_PATH))

# Specific CXXFLAGS for g++ only
# g++ uses CPPFLAGS and CXXFLAGS
#
CXXFLAGS     = $(addprefix --define=, $(PLATFORM_TAG))
CXXFLAGS    += $(addprefix --include_path=,$(INCLUDE_PATH))

# Specific ASFLAGS for gcc assembler only
# gcc assembler uses CPPFLAGS and ASFLAGS
#
ASFLAGS      = --asm_extension=S
ASFLAGS     += $(addprefix --include_path=,$(INCLUDE_PATH))

# Specific LDFLAGS for linker only
# linker uses CPPFLAGS and LDFLAGS
#
LDFLAGS      = $(addprefix --define=, $(PLATFORM_TAG))
LDFLAGS     += -z --stack_size=0x300 --warn_sections
LDFLAGS     += --map_file=Builds/embeddedcomputing.map
LDFLAGS     += $(addprefix -i,$(INCLUDE_PATH))
LDFLAGS     += --reread_libs --display_error_number --diag_wrap=off --entry_point=code_start --rom_model


ifneq ($(filter __TMS320F28027__,$(GCC_PREPROCESSOR_DEFINITIONS)),)
    COMMAND_FILES = $(CORE_LIB_PATH)/f2802x_common/cmd/F28027.cmd $(CORE_LIB_PATH)/f2802x_headers/cmd/F2802x_Headers_nonBIOS.cmd
endif
ifneq ($(filter __TMS320F28069__,$(GCC_PREPROCESSOR_DEFINITIONS)),)
    COMMAND_FILES = $(CORE_LIB_PATH)/f2806x_common/cmd/F28069.cmd $(CORE_LIB_PATH)/f2806x_headers/cmd/F2806x_Headers_nonBIOS.cmd
endif
ifneq ($(filter __TMS320F28377S__,$(GCC_PREPROCESSOR_DEFINITIONS)),)
    COMMAND_FILES = $(CORE_LIB_PATH)/F2837xS_common/cmd/2837xS_Generic_Flash_lnk.cmd $(CORE_LIB_PATH)/F2837xS_headers/cmd/F2837xS_Headers_nonBIOS.cmd
endif


# Specific OBJCOPYFLAGS for objcopy only
# objcopy uses OBJCOPYFLAGS only
#
OBJCOPYFLAGS  = #

# Target
#
#TARGET_HEXBIN = $(TARGET_TXT)
TARGET_HEXBIN = $(TARGET_OUT)


# Commands
# ----------------------------------
#
COMMAND_UPLOAD = $(UPLOADER_EXEC) load -c $(UPLOADER_OPTS) -f $(TARGET_OUT)

