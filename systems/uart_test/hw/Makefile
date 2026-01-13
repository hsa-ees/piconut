#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2024 Lukas Bauer <lukas.bauer1@tha.de>
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


# Declare that this is part of a system ...
PN_SYSTEM := ..


# Include PicoNut setup ...
include ../../../piconut.mk





################################################################################
#                                                                              #
#   Source Files                                                               #
#                                                                              #
################################################################################


# Module name ...
#   By convention,
#   - the module name is equal to the directory name,
#   - the top-level entity has the same name, prefixed with "m_",
#   - the top-level entity is defined in a file named $(MODULE).cpp,
#   - the testbench is defined in a file named $(MODULE)_tb.cpp.
#   The following lines can be copied to new modules with very little
#   modifications. However, if required and adequate for some module, other
#   variables may be used. Any variables not starting with "PN_" and which are
#   not special variables (like DEBUG, VERBOSE, ... - see `make help`) are local
#   and can be defined freely in this file.
MODULE := uart_test


# Module sources that are used for simulation and for synthesis ...
MODULE_SRC := \
	uart_test_top.cpp  \
	uart_echo.cpp


# Simulator and its source(s), containing `sc_main` - for simulation ...
SIMULATOR := $(MODULE)_tb
SIMULATOR_SRC :=  uart_test_top_tb.cpp


# Testbench and its source(s), containing `sc_main` - for synthesis ...
TESTBENCH := $(MODULE)_tb
TESTBENCH_SRC := uart_test_top_tb.cpp





################################################################################
#                                                                              #
#   Simulation                                                                 #
#                                                                              #
################################################################################


# Define set of object files and output binaries ...
#   As many hardware modules, this section builds
#   a) a shared library with the simulation model of the module ('MODULE_*'),
#   b) a simulation binary based on a testbench using this library ('SIMULATOR_*').
MODULE_OBJ := $(MODULE_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
MODULE_LIB := $(PN_MODULE_BUILD_DIR)/sim/lib$(MODULE).a

SIMULATOR_OBJ := $(SIMULATOR_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
SIMULATOR_BIN := $(PN_MODULE_BUILD_DIR)/sim/$(MODULE)_sim


# Automatic dependencies ...
#   The default compile command generates a .d file for each .o file
#   containing the .h files it depends on. These files are included here.
-include $(MODULE_OBJ:%.o=%.d)
-include $(SIMULATOR_OBJ:%.o=%.d)


# Set object files to link ...
#   The PicoNut setup script ('piconut.mk') contains predefined (implicit) rules
#   for compiling and linking. If they are used (as in this file), an additional
#   linking rule must be supplied here to identify all object files the binary
#   depends on. The recipe must be left out, it is supplied by 'piconut.mk'.
#

#   dependencies of the library ...
$(MODULE_LIB): $(MODULE_OBJ)

#   dependencies of the executable ...
$(SIMULATOR_BIN): $(SIMULATOR_OBJ) $(MODULE_LIB)


# Add libraries used ...
PN_HW_LDFLAGS += \
	-luart



#################### Exported global targets ###################################


#~ ifneq ($(PN_CFG_MEMBRANA),membrana_soft)
#~ $(warning "The simulator can presently only be built with PN_CFG_MEMBRANA = membrana_soft.")
#~ else


# Build ...
build-sim: $(MODULE_LIB) $(SIMULATOR_BIN)


# Install ...
.PHONY: install-sim
install-sim:
	$(PN_INSTALL_HW_BIN) $(SIMULATOR_BIN)


# TBD: Verify-sim





################################################################################
#                                                                              #
#   Synthesis                                                                  #
#                                                                              #
################################################################################

# At this level, (only) ICSC synthesis is performed for the complete hardware
# system excluding embedded memories. Gate-level and layout synthesis are performed
# later after the embedded memories are added.





#################### Phase 1: SystemC synthesis (build-systemc) ####################

# The following definitions direct the synthesis by the Intel Compiler for
# SystemC (ICSC). ICSC comes with its own build system an C++ compiler, and it
# may handle a complete tree of modules at once.


# Module sources for ICSC ...
#   Module source files including the testbench. Sources exported by an
#   'icsc-sources' recipe must be left out.
PN_SYSTEMC_SRC := $(MODULE_SRC) $(TESTBENCH_SRC)


# Submodules for ICSC ...
#   Define the submodules that ICSC must process together with this one.
PN_SYSTEMC_SUBMODULES := $(PN_SOURCE_DIR)/hw/cpu $(PN_SOURCE_DIR)/hw/peripherals/uart
#~ PN_SYSTEMC_SUBMODULES := $(PN_SOURCE_DIR)/hw/cpu $(PN_SOURCE_DIR)/hw/peripherals/uart





#################### Exported synthesis targets ################################


# Build ...
build-syn: build-systemc


# Verify ...
#~ verify-syn: build-netlist


# Install ...
install-syn: build-syn
	$(PN_INSTALL_HW_SYN) $(PN_SYSTEMC_OUTPUT)
