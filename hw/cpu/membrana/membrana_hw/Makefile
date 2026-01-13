#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#  Description:
#    Makefile of the reference Membrana.
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


# Note: This module is configured for pre-synthesis and stages Verilog models
#       - at RT level (membrana_hw.v)
#       - at gate level (membrana_hw.<board>.*)
#
# The installed pre-synthesized models contain blackbox 'm_membrana_hw_emem' to
# be supplied by the user. See the comment on EMEM_CONFIG on how to generate
# a compatible model. A compatible eMem module is also installed as
# $(EMEM_VERILOG_EMPTY).


# TBD+: Cleanup names





################################################################################
#                                                                              #
#   Configuration                                                              #
#                                                                              #
################################################################################


# Include PicoNut setup ...
include ../../../../piconut.mk


# Embedded memory configuration ...
#   Parameters to be passed to `pn-memgen` to generate a compatible embedded memory.
#   The Membrana implementation relies on these parameters. System designers
#   may copy these parameters to create there own compatible memories pre-initialized
#   with their software.
EMEM_CONFIG := -m m_membrana_hw_emem -p 2 -a 32 -d 4x8 -s $(PN_CFG_MEMBRANA_EMEM_SIZE)





################################################################################
#                                                                              #
#   Source Files                                                               #
#                                                                              #
################################################################################


# TBD+: Eliminate 'elf_reader' -> use the one from 'common'
# TBD+: 'membrana_hw_emem.cpp' makes decisions on hard-coded constants
#      PN_CFG_CPU_RESET_ADR (previously CFG_START_ADDRESS, the CPU reset vector)
#      and 0x0001FFFF. This does not make sense: Eliminate them and
#      use PN_CFG_MEMBRANA_EMEM_BASE and PN_CFG_MEMBRANA_EMEM_SIZE to determine
#      if an address goes to local memory.


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
MODULE_SRC := $(MODULE).cpp membrana_hw_emem.cpp elf_reader.cpp


# Testbench and its source(s), containing `sc_main` ...
TESTBENCH := $(MODULE)_tb/$(MODULE)_tb
TESTBENCH_SRC := $(TESTBENCH).cpp





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
	$(PN_HW_RUN) $<


# Install ...
.PHONY: install-all
install-all:
	$(PN_INSTALL_HW_INCLUDE) $(MODULE).h

.PHONY: install-sim
install-sim:
	$(PN_INSTALL_HW_LIB) $(MODULE_LIB)





################################################################################
#                                                                              #
#   Synthesis                                                                  #
#                                                                              #
################################################################################


# Embedded memory (eMem) Verilog modules ...
#   The blackbox module leaves the memory empty, allowing to supply the memory later.
#   It is used locally to pre-synthesize the Membrana module.
#
#   The empty module is a compatible module that can be used later, if no other
#   (i.e. preloaded) module is supplied. It is staged/installed for external use.
#
EMEM_VERILOG_BLACKBOX := $(PN_MODULE_BUILD_DIR)/syn/membrana_hw_emem-blackbox.v
EMEM_VERILOG_EMPTY := $(PN_MODULE_BUILD_DIR)/syn/membrana_hw_emem-empty.v

$(EMEM_VERILOG_BLACKBOX):
	@mkdir -p $(PN_MODULE_BUILD_DIR)/syn && \
	$(PN_TOOLS_BIN)/pn-memgen $(EMEM_CONFIG) -b $(EMEM_VERILOG_BLACKBOX)

$(EMEM_VERILOG_EMPTY):
	@mkdir -p $(PN_MODULE_BUILD_DIR)/syn && \
	$(PN_TOOLS_BIN)/pn-memgen $(EMEM_CONFIG) $(EMEM_VERILOG_EMPTY)


# Phony target to call the memory generation manually ...
.PHONY: build-emem
build-emem: $(EMEM_VERILOG_BLACKBOX) $(EMEM_VERILOG_EMPTY)





#################### Phase 1: SystemC synthesis (build-systemc) ################

# The following definitions direct the synthesis by the Intel Compiler for
# SystemC (ICSC). ICSC comes with its own build system an C++ compiler, and it
# may handle a complete tree of modules at once.


# Module sources for ICSC ...
#   Module source files including the testbench. Sources exported by an
#   'icsc-sources' recipe must be left out.
PN_SYSTEMC_SRC := $(MODULE_SRC) $(TESTBENCH_SRC)


# Submodules for ICSC ...
#   Define the submodules that ICSC must process together with this one.
PN_SYSTEMC_SUBMODULES := $(PN_SUBMODULES)





#################### Phase 2: Gate-level synthesis (build-netlist) #############

# The following rules define a *reusable module*: Local synthesis is performed
# up to a gate-level, board-specific netlist, and this netlist is installed by
# the 'install-syn' target. The 'verify-syn' target (only) checks if the netlist
# can be built successfully.
#
# The module contains a blackbox module 'm_membrana_hw_emem' for the embedded memory
# (RAM). A Verilog file for an empty RAM is provided as $(EMEM_VERILOG_EMPTY). The user
# may provide an alternative, pre-initialized memory variant instead, which
# must match the parameters given by $(EMEM_CONFIG) - see above.


# Synthesis for netlist ...
#   Name all RTL files (Verilog, VDHL) and presynthesized modules to be passed
#   to the synthesis tool. In most cases, (only) the SystemC synthesis result
#   (PN_SYSTEMC_OUTPUT) will be passed here.
PN_NETLIST_SRC := $(PN_SYSTEMC_OUTPUT) $(EMEM_VERILOG_BLACKBOX)


# Declare dependencies ...
#   Declare the source files on which the netlist generation step depends.
#   Usually, the following line can be left untouched.
$(PN_NETLIST_OUTPUT): $(PN_NETLIST_SRC)





#################### Exported global targets ###################################


# Build ...
build-syn: build-netlist $(EMEM_VERILOG_EMPTY)


# Verify ...
verify-syn: build-netlist


# Install ...
install-syn: build-syn
	$(PN_INSTALL_HW_SYN) $(PN_SYSTEMC_OUTPUT) $(PN_NETLIST_OUTPUT)
	$(PN_INSTALL_HW_SYN) $(EMEM_VERILOG_BLACKBOX) $(EMEM_VERILOG_EMPTY)
