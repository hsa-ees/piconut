#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#  Description:
#    Makefile of the module CPU which provides the PicoNut cores of
#    all currently available Capability levels
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
include ../../../../piconut.mk





################################################################################
#                                                                              #
#   Source Files                                                               #
#                                                                              #
################################################################################


# TBD: Eliminate/move ELF and soft peripheral code to hw/common
#      (In the test-bench there is a ELF loaded to test the elf_loader and the module.)



# MODULE_SRC:        Sources that are used for simulation and for synthesis.
MODULE := $(PN_MODULE_NAME)
MODULE_SRC := $(MODULE).cpp \
	elf_reader.cpp \
	c_soft_memory.cpp \
	c_soft_peripheral_container.cpp


# TESTBENCH_SRC:     Source that contains the testbench (and thus sc_main).
TESTBENCH := $(MODULE)_tb/$(MODULE)_tb
TESTBENCH_SRC := \
	$(TESTBENCH).cpp \
	$(MODULE)_tb/c_soft_fake_peripheral.cpp





################################################################################
#                                                                              #
#   Simulation                                                                 #
#                                                                              #
################################################################################


# Define set of object files and output binaries ...
#   As many hardware modules, this section builds
#   a) a shared library with the simulation model of the module ('MODULE_*'),
#   b) a simulation binary based on a testbench using this library ('TESTBENCH_*').
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
$(MODULE_LIB): $(MODULE_OBJ)

#   dependencies of the executable ...
$(TESTBENCH_BIN): $(TESTBENCH_OBJ) $(MODULE_LIB)





################################################################################
#                                                                              #
#   Global Targets                                                             #
#                                                                              #
################################################################################


# Build ...
.PHONY: build-lib
build-lib: $(MODULE_LIB)

.PHONY: build-tb
build-tb: $(TESTBENCH_BIN)

.PHONY: build-all
build-sim: build-lib build-tb


# Check ...
#   This is the normal way, running the main testbench
.PHONY: verify-sim
verify-sim: $(TESTBENCH_BIN)
	$(PN_HW_RUN) $<


# Install ...
.PHONY: install-all
install-all:
	$(PN_INSTALL_HW_INCLUDE) $(MODULE).h

.PHONY: install-sim^
install-sim:
	$(PN_INSTALL_HW_INCLUDE) $(MODULE).h c_soft_memory.h c_soft_peripheral_container.h
	$(PN_INSTALL_HW_LIB) $(MODULE_LIB)


# TBD: Reduce headers to install & adapt 'install-all' target
