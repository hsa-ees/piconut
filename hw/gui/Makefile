#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2026 Johannes Hofmann <johannes.hofmann1@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
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





################################################################################
#                                                                              #
#   Configuration                                                              #
#                                                                              #
################################################################################


# Include PicoNut setup ...
include ../../piconut.mk





################################################################################
#                                                                              #
#   Source Files                                                               #
#                                                                              #
################################################################################



# Module name ...
#   By convention,
#   - the module name is equal to the directory name,
#   - the top-level entity has the same name,
#   - the top-level entity is defined in a file named $(MODULE).cpp,
#   - the testbench is defined in a file named $(MODULE)_tb.cpp.
#   The following lines can be copied to new modules with very little
#   modifications. However, if required and adequate for some module, other
#   variables may be used. Any variables not starting with "PN_" and which are
#   not special variables (like DEBUG, VERBOSE, ... - see `make help`) are local
#   and can be defined freely in this file.
MODULE := $(PN_MODULE_NAME)


# Module sources that are used for simulation and for synthesis ...
MODULE_SRC := $(MODULE).cpp mainwindow.cpp framebuffer_to_image.cpp

# Testbench and its source(s), containing `sc_main` ...
TESTBENCH := $(MODULE)_tb/$(MODULE)_tb
TESTBENCH_SRC := $(TESTBENCH).cpp


#
# Qt
#

# Module sources for Qt moc compiler ...
MOC_SRC ?= mainwindow.h

# Module sources for Qt ui files ...
UI_SRC ?= mainwindow.ui

# Tools
MOC = $(QT_HOME)/libexec/moc
UIC = $(QT_HOME)/libexec/uic





################################################################################
#                                                                              #
#   Simulation                                                                 #
#                                                                              #
################################################################################


# Define set of object files and output binaries ...
#   As many hardware modules, this section builds
#   a) a shared library with the simulation model of the module ('MODULE_*'),
#   b) a simulation binary based on a testbench using this library ('TESTBENCH_*').
MOC_SRC_H := $(filter %.h, $(MOC_SRC))
MOC_SRC_CPP := $(filter %.cpp, $(MOC_SRC))
MOC_FILES_H := $(MOC_SRC_H:%.h=$(PN_MODULE_BUILD_DIR)/sim/%_moc.cpp)
MOC_FILES_CPP := $(MOC_SRC_CPP:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.moc)
MODULE_SRC += $(MOC_FILES_CPP) $(MOC_FILES_H)

UI_FILES_H := $(UI_SRC:%.ui=$(PN_MODULE_BUILD_DIR)/sim/%_ui.h)


MODULE_OBJ := $(MODULE_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
MODULE_LIB := $(PN_MODULE_BUILD_DIR)/sim/lib$(MODULE).a

TESTBENCH_OBJ := $(TESTBENCH_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
TESTBENCH_BIN := $(PN_MODULE_BUILD_DIR)/sim/$(TESTBENCH)


# Automatic dependencies ...
#   The default compile command $(PN_HW_CC) generates a .d file for each .o file
#   containing the .h files it depends on. These files are included here.
-include $(MODULE_OBJ:%.o=%.d)
-include $(TESTBENCH_OBJ:%.o=%.d)


# Set object files to link ...
#   The PicoNut setup script ('piconut.mk') contains predefined (implicit) rules
#   for compiling and linking. If they are used (as in this file), an additional
#   linking rule must be supplied here to identify all object files the binary
#   depends on. The recipe must be left out, it is supplied by 'piconut.mk'.
#

#   dependencies of the library ...
$(MODULE_LIB): $(MOC_FILES_H) $(MOC_FILES_CPP) $(UI_FILES_H) $(MODULE_OBJ) 

#   dependencies of the executable ...
$(TESTBENCH_BIN): $(TESTBENCH_OBJ) $(MODULE_LIB)

QT_INCLUDES := \
	-I$(QT_HOME)/include/QtCore \
  -I$(QT_HOME)/include/QtGui \
  -I$(QT_HOME)/include/QtWidgets \
  -I$(QT_HOME)/include

# Note: Build dir has to be added as include dir because .ui files translated to
# to .h files have to be available for the module.
PN_HW_CFLAGS += $(QT_INCLUDES) -I$(PN_MODULE_BUILD_DIR)/sim/

PN_HW_LDFLAGS += \
	-fuse-ld=mold \
	-L$(QT_HOME)/lib \
	-lQt6Core -lQt6Gui -lQt6Widgets \
	-DQT_CORE_LIB -DQT_GUI_LIB -DQT_WIDGETS_LIB


# Qt moc build for header files
$(PN_MODULE_BUILD_DIR)/sim/%_moc.cpp: %.h
ifneq (0,$(VERBOSE))
	@mkdir -p $(dir $@)
	$(MOC) $(QT_INCLUDES) $< -o $@
else
	@echo $(PN_BUILD_PREFIX)HW-QT_MOC $(@:$(PN_MODULE_BUILD_DIR)/%=%); \
	mkdir -p $(dir $@) && \
	$(MOC) $(QT_INCLUDES) $< -o $@
endif

# Qt moc build for source files
$(PN_MODULE_BUILD_DIR)/sim/%.moc: %.cpp
ifneq (0,$(VERBOSE))
	@mkdir -p $(dir $@)
	$(MOC) $(QT_INCLUDES) $< -o $@
else
	@echo $(PN_BUILD_PREFIX)HW-QT_MOC $(@:$(PN_MODULE_BUILD_DIR)/%=%); \
	mkdir -p $(dir $@) && \
	$(MOC) $(QT_INCLUDES) $< -o $@
endif

# Qt moc build for ui files
$(PN_MODULE_BUILD_DIR)/sim/%_ui.h: %.ui
ifneq (0,$(VERBOSE))
	@mkdir -p $(dir $@)
	$(UIC) $< -o $@
else
	@echo $(PN_BUILD_PREFIX)HW-QT_UIC $(@:$(PN_MODULE_BUILD_DIR)/%=%); \
	mkdir -p $(dir $@) && \
	$(UIC) $< -o $@
endif



#################### Exported global targets ###################################


# Build ...
.PHONY: build-lib
build-lib: $(MODULE_LIB)

.PHONY: build-tb
build-tb: $(TESTBENCH_BIN)

build-sim: build-lib build-tb


# Verify ...
#   Run the main testbench.
.PHONY: verify-sim
verify-sim: $(TESTBENCH_BIN)
ifeq ($(strip $(DISPLAY)$(WAYLAND_DISPLAY)),)
		QT_QPA_PLATFORM=offscreen $(PN_HW_RUN) $<
else
		$(PN_HW_RUN) $<
endif


# Install ...
.PHONY: install-all
install-all:
	$(PN_INSTALL_HW_INCLUDE) $(MODULE).h

.PHONY: install-sim
install-sim:
	$(PN_INSTALL_HW_LIB) $(MODULE_LIB)

