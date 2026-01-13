# OBSOLETE: This file may soon be removed.

#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2024 Lukas Bauer <lukas.bauer1@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    This is a base makefile, to be used in conjunction with the sysc-base.mk
#    to build c_soft_peripherals that have a qt gui.
#    It provides the necessary rules to build the moc and ui files.
#
#  --------------------- LICENSE -----------------------------------------------
#  Redistribution and use in source and binary forms, with or without modification,
#  are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice, this
#     list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation and/or
#     other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
#  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#  -----------------------------------------------------------------------------

# Set the default Shell
SHELL = /bin/bash

# include makefile containing PicoNut directory structure information
include $(PICONUT_HOME)/directory-base.mk

# include local configuration
# variables defined in the included config.mk overwrite those
# found in the global configuration
-include $(PNS_CONFIG_MK)

# include global configuration
include $(PN_CONFIG_MK)



################################################################################
#################                Configuration             #####################
################################################################################

# MOC_SRC						Files that need to be moced with qt
MOC_SRC ?=

# UI_SRC						UI files to convert to .h files
UI_SRC ?=


################ Target definitions #############################################

# Defines the Taget files for the moc compiler
MOC_SRC_H := $(filter %.h, $(MOC_SRC))
MOC_SRC_CPP := $(filter %.cpp, $(MOC_SRC))
MOC_FILES_H := $(MOC_SRC_H:%.h=%_moc.cpp)
MOC_FILES_CPP := $(MOC_SRC_CPP:%.cpp=%.moc)

# adds them to the src to be built
SIM_SRC += $(MOC_FILES_CPP) $(MOC_FILES_H)

# Defines the Target files for the ui compiler
UI_FILES_H := $(UI_SRC:%.ui=%_ui.h)


################# Qt dependencies ##############################################

##### To make Qt work, copy the following line in the .bashrc file

QT_HOME ?=

LD_LIBRARY_PATH ?= "$(QT_HOME)/lib:${LD_LIBRARY_PATH}"

################# General Compiler Configuration ###############################
MOC = $(QT_HOME)/libexec/moc
UI = $(QT_HOME)/libexec/uic

################# MOCS and UIs #################################################

# human usable target
.PHONY: build-moc
build-moc: $(MOC_FILES_H) $(MOC_FILES_CPP)

# rule to build the moc files from h files with Q_OBJECT
%_moc.cpp: %.h
	$(MOC) $(MODULE_INCLUDES) $< -o $@

# rule to build the moc files from cpp files with Q_OBJECT
%.moc: %.cpp
	$(MOC) $(MODULE_INCLUDES) $< -o $@

# human usable target
.PHONY: build-ui
build-ui: $(UI_FILES_H)

# rule to build the ui files
%_ui.h: %.ui
	$(UI) $< -o $@

# appends the moc and ui compilation to the compilation of the module
build-lib-pre:: $(MOC_FILES_H) $(MOC_FILES_CPP) $(UI_FILES_H)



################# Clean up #####################################################

.PHONY: clean-qt
clean-qt:
	@rm -f $(MOC_FILES_H) $(MOC_FILES_CPP) $(UI_FILES_H)

clean-custom: clean-qt

################# Help ##########################################################

help-append::

	@echo ""
	@echo "Makefile for Qt applications"
	@echo "This allows to build the moc and ui files"
	@echo ""
	@echo "Targets:"
	@echo "	make build-moc: 	build the moc files"
	@echo "	make build-ui: 		build the ui files"
	@echo "	make clean-qt: 		clean the moc and ui files"
