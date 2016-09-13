#
# embedXcode
# ----------------------------------
# Embedded Computing on Xcode
#
# Copyright © Rei VILO, 2010-2016
# http://embedxcode.weebly.com
# All rights reserved
#
# Last update: Aug 20, 2016 release 5.1.6
#






# Doxygen parameters
# ----------------------------------
# Edit ./Utilities/doxyfile for options
#
# For more information, see
#   www.doxygen.org
# 	© 1997-2012 by Dimitri van Heesch
# 	GNU General Public License
# 	Documents produced by doxygen are derivative works derived from the input used in their production.
# 	They are not affected by this license.
#
#
DOXYGEN_PATH    = /Applications/Doxygen.app/Contents/Resources/doxygen
ifeq ($(wildcard $(DOXYGEN_PATH)),)
    $(error Error: application doxygen not found)
endif

DOCUMENTS_PATH  = $(CURRENT_DIR)/Builds
DOXYFILE        = $(CURRENT_DIR)/Utilities/doxyfile
ifeq ($(wildcard $(DOXYFILE)),)
    $(error Error: configuration file doxyfile not found)
endif


GENERATE_HTML   = $(strip $(shell grep 'GENERATE_HTML *=' $(DOXYFILE) | cut -d = -f 2- ))
GENERATE_PDF    = $(strip $(shell grep 'GENERATE_LATEX *=' $(DOXYFILE) | cut -d = -f 2- ))
GENERATE_DOCSET = $(strip $(shell grep 'GENERATE_DOCSET *=' $(DOXYFILE) | cut -d = -f 2- ))

BUNDLE_ID       = $(strip $(shell grep 'DOCSET_BUNDLE_ID *=' $(DOXYFILE) | cut -d = -f 2- ))
DOCSET_PATH     = $(USER_LIBRARY_DIR)/Developer/Shared/Documentation/DocSets/$(BUNDLE_ID).docset
LOAD_UTIL_PATH  = $(UTILITIES_PATH)/loadDocSet.scpt

PDF_PATH        = $(DOCUMENTS_PATH)/latex/refman.pdf
TEX_PATH        = $(DOCUMENTS_PATH)/latex/refman.tex

ifeq ($(GENERATE_HTML),YES)
    ifeq ($(wildcard $(DOXYGEN_PATH)),)
        $(error Error: application doxygen not found)
    endif
endif


# Style options
# ----------------------------------
#
STYLE_OPTIONS   = --options='$(CURRENT_DIR)/Utilities/style.conf'
STYLE_FLAG      = $(shell which astyle)


# Doxygen rules
# ----------------------------------
#
# ~
document0:
		@echo " ---- Doxygen options ----"
		@echo " .    html		" $(GENERATE_HTML)
		@echo " .    pdf		" $(GENERATE_PDF)
		@echo " .    docset	" $(GENERATE_DOCSET)
		@echo " .    bundle id	" $(BUNDLE_ID)

		@echo "--- Doxygen reset ---"
		@if [ -d $(DOXYFILE)/html ];  then $(REMOVE) $(DOXYFILE)/html; fi
		@if [ -d $(DOXYFILE)/latex ]; then $(REMOVE) $(DOXYFILE)/latex; fi
		@if [ -d $(DOXYFILE)/xml ];   then $(REMOVE) $(DOXYFILE)/xml; fi

		@if [ -f $(LOAD_UTIL_PATH) ]; then $(REMOVE) $(LOAD_UTIL_PATH); fi
		@if [ -f $(DOCSET_PATH) ]; then $(REMOVE) $(DOCSET_PATH); fi
		@echo "tell application \"Xcode\"" > $(LOAD_UTIL_PATH)
		@echo "load documentation set with path \"$(DOCSET_PATH)\"" >> $(LOAD_UTIL_PATH)
		@echo "end tell" >> $(LOAD_UTIL_PATH)

document1:
		@echo "--- Doxygen warnings ---"
		$(DOXYGEN_PATH) $(DOXYFILE)
		@echo "---- .docset generated ----"

document2:
ifeq ($(GENERATE_PDF),YES)
		@if [ -f $(TEX_PATH) ]; then Utilities/document.sh > Utilities/pdf.log; echo "---- PDF created ---- "; fi
		if [ -f $(PDF_PATH) ]; then cp $(PDF_PATH) "../$(PROJECT_NAME) - Reference Manual.pdf"; fi
endif

document3:
ifeq ($(GENERATE_DOCSET),YES)
		@if [ -f $(DOCSET_PATH) ]; then if [ $(shell osascript '$(LOAD_UTIL_PATH)') = true ]; then echo "---- .docset loaded ---- "; else echo "---- .docset not loaded ---- "; fi; fi;
endif

document4:
ifeq ($(GENERATE_DOCSET),YES)
		make -C $(DOCUMENTS_PATH)/html install > ./Builds/docset.log
		@echo "---- .docset installed ----"
endif

message_document:
		@echo "==== Document ===="

document:	message_document document0 document1 document2 document3 document4
		@echo "==== Document done ==== "
# ~~


# ~
distribute:
		@echo "==== Distribution ===="
		@Utilities/distribute.sh $(PROJECT_NAME)
		@echo "==== Distribution done ===="
# ~~

# ~
# Style rules
# ----------------------------------
#
style:
	@echo "==== Style ===="
ifeq ($(STYLE_FLAG),)
	@echo 'ERROR        astyle utility not available'
else
	@if [ $$(find . -name \*.ino -type f | wc -l) -gt 0 ] ; then astyle $(STYLE_OPTIONS) '$(CURRENT_DIR)/*.ino' ; fi
	@if [ $$(find . -name \*.c -type f | wc -l) -gt 0 ] ; then astyle $(STYLE_OPTIONS) '$(CURRENT_DIR)/*.c' ; fi
	@if [ $$(find . -name \*.cpp -type f | wc -l) -gt 0 ] ; then astyle $(STYLE_OPTIONS) '$(CURRENT_DIR)/*.cpp' ; fi
	@if [ $$(find . -name \*.h -type f | wc -l) -gt 0 ] ; then astyle $(STYLE_OPTIONS) '$(CURRENT_DIR)/*.h' ; fi
endif
	@echo "==== Style done ===="
# ~~

.PHONY:	document0 document1 document2 document3 document4 message_document document distribute style
