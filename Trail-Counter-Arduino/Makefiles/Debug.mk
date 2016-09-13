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
# Last update: Aug 30, 2016 release 5.2.1





# Debug rules
# ----------------------------------
#
# ~
debug: 		make message_debug reset raw_upload serial launch_debug end_debug
# ~~

# ~
launch_debug:

# 1. GDB
ifneq ($(GDB),)
	@echo GDB debugger
	@if [ -f $(UTILITIES_PATH)/embedXcode_debug ]; then export STECK_EXTENSION=$(STECK_EXTENSION); $(UTILITIES_PATH)/embedXcode_debug; fi;

# 1.1 mspdebug
  ifeq ($(UPLOADER),mspdebug)
    ifeq ($(UPLOADER_PROTOCOL),tilib)
# . Step 1: Launch the server
		@echo "---- Launch server ----"
		$(call SHOW,"8.1-DEBUG",$(UPLOADER))
		osascript -e 'tell application "Terminal" to do script "cd $(UPLOADER_PATH); ./mspdebug $(UPLOADER_PROTOCOL) gdb"'

    else
		@echo "---- Launch server ----"
		$(call SHOW,"8.2-DEBUG",$(UPLOADER))
		@osascript -e 'tell application "Terminal" to do script "$(UPLOADER_EXEC) $(UPLOADER_PROTOCOL) gdb"'
    endif

# . Step 2: Launch the client
		@echo "---- Launch client ----"
		$(call SHOW,"8.3-DEBUG",$(notdir $(GDB)))
		@sleep 5
		@osascript -e 'tell application "Terminal" to do script "cd $(CURRENT_DIR_SPACE); $(GDB) -x \"$(UTILITIES_PATH_SPACE)/gdb.txt\""'

# . Step 3: Garbage collector
		@if [ -f libmsp430.so ]; then rm libmsp430.so; fi
		@if [ -f comm.log ]; then rm comm.log; fi

# 1.2 lm4flash
  else ifeq ($(UPLOADER),lm4flash)
# . Step 1: Launch the server
		@echo "---- Launch server ----"
		$(call SHOW,"8.4-DEBUG",$(UPLOADER))
		-killall openocd
		@osascript -e 'tell application "Terminal" to do script "openocd --file \"$(UTILITIES_PATH_SPACE)/debug_LM4F120XL.cfg\""'

# . Step 2: Launch the client
		@echo "---- Launch client ----"
		$(call SHOW,"8.5-DEBUG",$(notdir $(GDB)))
		@sleep 5
		@osascript -e 'tell application "Terminal" to do script "cd $(CURRENT_DIR_SPACE); $(GDB) -x \"$(UTILITIES_PATH_SPACE)/gdb.txt\""'

# 1.3 cc3200serial
  else ifeq ($(UPLOADER),cc3200serial)
# . Step 1: Launch the server
		@echo "---- Launch server ----"
		$(call SHOW,"8.6-DEBUG",$(UPLOADER))
		-killall openocd
		@osascript -e 'tell application "Terminal" to do script "openocd --file \"$(UTILITIES_PATH_SPACE)/debug_CC3200.cfg\""'

# . Step 2: Launch the client
		@echo "---- Launch client ----"
		$(call SHOW,"8.7-DEBUG",$(notdir $(GDB)))
		@sleep 5
		@osascript -e 'tell application "Terminal" to do script "cd $(CURRENT_DIR_SPACE); $(GDB) -x \"$(UTILITIES_PATH_SPACE)/gdb.txt\""'

# 1.4 DSLite
  else ifeq ($(UPLOADER),DSLite)
# . Step 1: Launch the server
		@echo "---- Launch server ----"
		$(call SHOW,"8.20-DEBUG",$(UPLOADER))
		-killall openocd
		@osascript -e 'tell application "Terminal" to do script "openocd --file \"$(UTILITIES_PATH_SPACE)/debug_MSP432P4.cfg\""'

# . Step 2: Launch the client
		@echo "---- Launch client ----"
		$(call SHOW,"8.21-DEBUG",$(notdir $(GDB)))
		@sleep 5
		@osascript -e 'tell application "Terminal" to do script "cd $(CURRENT_DIR_SPACE); $(GDB) -x \"$(UTILITIES_PATH_SPACE)/gdb.txt\""'

# 1.5 mbed
  else ifeq ($(PLATFORM),mbed)
# . Step 1: Launch the server
    ifeq ($(DEBUG_SERVER),stlink)
		@echo "---- Launch server ----"
		$(call SHOW,"8.8-DEBUG",$(UPLOADER))
		-killall st-util
		@osascript -e 'tell application "Terminal" to do script "st-util -p 3333"'
    else
		@echo "---- Launch server ----"
		$(call SHOW,"8.9-DEBUG",$(UPLOADER))
		-killall openocd
		@osascript -e 'tell application "Terminal" to do script "openocd --file \"$(UTILITIES_PATH_SPACE)/debug_$(BOARD_TAG).cfg\""'
    endif

# . Step 2: Launch the client
		@echo "---- Launch client ----"
		$(call SHOW,"8.10-DEBUG",$(UPLOADER))
		-killall $(GDB)
		@sleep 5
		@osascript -e 'tell application "Terminal" to do script "cd $(CURRENT_DIR_SPACE); $(GDB) -x \"$(UTILITIES_PATH_SPACE)/gdb.txt\""'

# 1.5 IntelYocto
  else ifeq ($(PLATFORM),IntelYocto)
# . Step 1: Launch the server
		@echo "---- Launch server ----"
		$(call SHOW,"8.11-DEBUG",$(UPLOADER))
		-killall $(GDB)

		osascript -e 'tell application "Terminal" to do script "cd $(CURRENT_DIR); $(UTILITIES_PATH)/uploader_ssh.sh $(SSH_ADDRESS) $(SSH_PASSWORD) $(REMOTE_FOLDER) $(TARGET) -debug"'
		@sleep 5

# . Step 2: Launch the client
		@echo "---- Launch client ----"
		$(call SHOW,"8.12-DEBUG",$(notdir $(GDB)))
		@sleep 5
		@osascript -e 'tell application "Terminal" to do script "cd $(CURRENT_DIR_SPACE); $(GDB) -x \"$(UTILITIES_PATH_SPACE)/gdb.txt\""'

# 1.6 Edison
  else ifeq ($(PLATFORM),Edison)
# . Step 1: Launch the server
		@echo "---- Launch server ----"
		$(call SHOW,"8.13-DEBUG",$(UPLOADER))
		-killall $(GDB)

		osascript -e 'tell application "Terminal" to do script "cd $(CURRENT_DIR); $(UTILITIES_PATH)/uploader_ssh.sh $(SSH_ADDRESS) $(SSH_PASSWORD) $(REMOTE_FOLDER) $(TARGET) -debug"'
		@sleep 5

# . Step 2: Launch the client
		@echo "---- Launch client ----"
		$(call SHOW,"8.14-DEBUG",$(notdir $(GDB)))
		-killall $(GDB)
		@sleep 5
		@osascript -e 'tell application "Terminal" to do script "cd $(CURRENT_DIR_SPACE); $(GDB) -x \"$(UTILITIES_PATH_SPACE)/gdb.txt\""'

# 1.7 openocd
  else ifeq ($(UPLOADER),openocd)
# . Step 1: Launch the server
		@echo "---- Launch server ----"
		$(call SHOW,"8.15-DEBUG",$(UPLOADER))
		-killall $(GDB)

		osascript -e 'tell application "Terminal" to do script "cd $(CURRENT_DIR); $(UPLOADER_EXEC) $(UPLOADER_OPTS)"'
		@sleep 5

# . Step 2: Launch the client
		@echo "---- Launch client ----"
		$(call SHOW,"8.16-DEBUG",$(notdir $(GDB)))
		@sleep 5
		@osascript -e 'tell application "Terminal" to do script "cd $(CURRENT_DIR_SPACE); $(GDB) -x \"$(UTILITIES_PATH_SPACE)/gdb.txt\""'

# 1.8 BeagleBoneDebian
  else ifeq ($(PLATFORM),BeagleBoneDebian)
# . Step 1: Launch the server
		@echo "---- Launch server ----"
		$(call SHOW,"8.17-DEBUG",$(UPLOADER))
		-killall $(GDB)

		osascript -e 'tell application "Terminal" to do script "cd $(CURRENT_DIR); $(UTILITIES_PATH)/uploader_ssh.sh $(SSH_ADDRESS) $(SSH_PASSWORD) $(REMOTE_FOLDER) $(TARGET) -debug"'
		@sleep 5

# . Step 2: Launch the client
		@echo "---- Launch client ----"
		$(call SHOW,"8.18-DEBUG",$(notdir $(GDB)))
		@sleep 5
		@osascript -e 'tell application "Terminal" to do script "cd $(CURRENT_DIR_SPACE); $(GDB) -x \"$(UTILITIES_PATH_SPACE)/gdb.txt\""'

# 1.9 Feather M0 with J-Link
else ifeq ($(UPLOADER),jlink)
# . Step 1: Launch the server
	@echo "---- Launch server ----"
	$(call SHOW,"8.22-DEBUG",$(notdir $(DEBUG_SERVER_EXEC)))
	-killall JLinkGDBServer

    ifneq ($(COMMAND_POWER),)
		@echo '. Board powered by J-Link'
		$(COMMAND_POWER)
    endif

	osascript -e 'tell application "Terminal" to do script "$(DEBUG_SERVER_EXEC) $(DEBUG_SERVER_OPTS)"'
	@sleep 5

# . Step 2: Launch the client
	@echo "---- Launch client ----"
	$(call SHOW,"8.23-DEBUG",$(notdir $(GDB)))
	-killall $(GDB)
	@sleep 5
	osascript -e 'tell application "Terminal" to do script "cd $(CURRENT_DIR_SPACE); $(GDB) -x \"$(UTILITIES_PATH_SPACE)/gdb.txt\""'

# 9.9 none
  else
		@echo "Board not supported" $(PLATFORM)
  endif

# 2. MDB
else ifneq ($(MDB),)
		@echo MDB debugger
		@if [ -f $(UTILITIES_PATH)/embedXcode_debug ]; then export STECK_EXTENSION=$(STECK_EXTENSION); $(UTILITIES_PATH)/embedXcode_debug; fi;

		@echo "---- Launch programmer-debugger ----"
		$(call SHOW,"8.19-DEBUG",$(UPLOADER))
		@osascript -e 'tell application "Terminal" to do script "$(MDB) \"$(UTILITIES_PATH_SPACE)/mdb.txt\""'

else
		@echo 'Unknown debugger'
endif

message_debug:
		@echo "==== Debug ===="

end_debug:
		@echo "==== Debug done ==== "
# ~~


.PHONY:	debug launch_debug message_debug end_debug

