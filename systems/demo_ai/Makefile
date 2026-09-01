#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#  Description:
#    Makefile for building the AI demo system for simulation and synthesis.
#    NOTE: Synthesis is not supported yet.
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





# Software to integrate ...
SW ?= sw/apps/ai/lenet5
#~ SW ?= sw/apps/hello_piconut


# Instruction trace level ...
ITRACE ?= 0


# VCD waveform level ...
VCD ?= 0





################################################################################
#                                                                              #
#   Configuration                                                              #
#                                                                              #
################################################################################


# Declare that this is the root of a system ...
PN_BUILD_INIT := 1
PN_SYSTEM := .


# Restrict TECHS to simulation (synthesis not supported yet) ...
export TECHS := sim


# Submodules ...
#   Define the submodules this module depends on.
#
#   The build system will automatically build them by the 'build' target before
#   executing the 'build-all' recipe defined below. Submodules are "installed"
#   into $PN_BUILD_DIR, so that no search paths for compiling/linking must be
#   set here.
#   Only PN_HW_LDFLAGS must be extended to load the submodules when linking.
PN_SUBMODULES := hw

# System-specific board constraints ...
PN_BOARD_CONSTRAINTS := boards/ulx3s/ulx3s.lpf


# Include PicoNut setup ...
include ../../piconut.mk


# Prebuild all required PicoNut modules ...
build-prepare:
	$(PN_PREBUILD) MODULES=hw/cpu:$(SW)
#	$(PN_PREBUILD) MODULES=hw/cpu:hw/peripherals/uart:hw/peripherals/uart_soft:$(SW)





################################################################################
#                                                                              #
#   Software                                                                   #
#                                                                              #
################################################################################


SW_NAME := $(notdir $(SW))
SW_BIN := $(PN_STAGE_DIR)/sw/bin/$(SW_NAME).rv32





################################################################################
#                                                                              #
#   Simulation                                                                 #
#                                                                              #
################################################################################


SIM_BIN := $(PN_MODULE_BUILD_DIR)/hw/sim/$(PN_MODULE_NAME)_sim


.PHONY: sim
sim: build
	$(SIM_BIN) -i $(ITRACE) -t $(VCD) $(SW_BIN)


.PHONY: debug-sim
debug-sim:
	gdbtui --args $(SIM_BIN) $(SW_BIN)


# Simulation with VCD generation ...
#   (adapt -t5 parameter if needed)
$(SIM_BIN).vcd: build
	$(SIM_BIN) -t5 $(SW_BIN) || true


.PHONY: view-vcd
view-vcd: $(SIM_BIN).vcd
	gtkwave $(SIM_BIN).vcd $(PN_MODULE_NAME)_sim.gtkw


.PHONY: view-sw
view-sw:
	riscv64-unknown-elf-objdump -M no-aliases,numeric -d $(SW_BIN)





################################################################################
#                                                                              #
#   Synthesis                                                                  #
#                                                                              #
################################################################################

# At this level, the RTL model of the complete system (from 'hw/') is
# combined with a memory pre-initialized with the software (from 'sw/').





##### Phase 2+3: Gate-level and layout synthesis ###############################

# (Nothing to be done here)

# Embedded memory (eMem) Verilog module ...
#   NOTE: The configuration (MEMBRANA_EMEM_CONFIG) must match the rules given by the
#         respective Membrana module (presently documented in `membrana_hw/Makefile`)
MEMBRANA_EMEM := $(PN_MODULE_BUILD_DIR)/syn/membrana_hw_emem.v
MEMBRANA_EMEM_CONFIG := -m m_membrana_hw_emem -p 2 -a 32 -d 4x8 -s $(PN_CFG_MEMBRANA_EMEM_SIZE)

$(MEMBRANA_EMEM): $(SW_BIN)
	@$(PN_TOOLS_BIN)/pn-memgen $(MEMBRANA_EMEM_CONFIG) -i $(SW_BIN) -ib $(PN_CFG_MEMBRANA_EMEM_BASE) $@

.PHONY: build-emem
build-emem: $(MEMBRANA_EMEM)


# Synthesis inputs ...
#   Name all RTL files (Verilog, VDHL) and presynthesized modules to be passed
#   to the synthesis tool. In most cases, (only) the SystemC synthesis result
#   (PN_SYSTEMC_OUTPUT) will be passed here.
#
PN_NETLIST_SRC := $(PN_MODULE_BUILD_DIR)/hw/syn/hw.v \
  $(PN_STAGE_DIR)/hw/syn/cpu.$(PN_CFG_BOARD).v \
  $(PN_STAGE_DIR)/hw/syn/uart.$(PN_CFG_BOARD).v \
  $(MEMBRANA_EMEM)


# Name of the top-level module ...
PN_TOPLEVEL := m_$(MODULE_NAME)



# Declare dependencies ...
#   Declare the source files on which the netlist generation step depends.
#   Usually, the following line can be left untouched.
$(PN_NETLIST_OUTPUT): $(PN_NETLIST_SRC)





################################################################################
#                                                                              #
#   Help                                                                       #
#                                                                              #
################################################################################


define HELP_TARGETS_LOCAL
	@echo
	@echo "  sim   : Build and run the simulator with software SW for the system."
endef


define HELP_PARAMETERS_LOCAL
	@echo "  SW    : Select the software module to run in the system."
	@echo "          This either means the software to simulate (sim target) or"
	@echo "          the software to include in the flashable bitfile."
	@echo "          (Default = $(subst $() $(),:,$(SW)))"
	@echo
endef





################################################################################
#                                                                              #
#   Exported global targets                                                    #
#                                                                              #
################################################################################


# Build ...
build-syn: build-layout


# Verify ...
verify-syn: build-layout


# Install ...
install-syn: build-syn
	$(PN_INSTALL_HW_SYN) $(PN_LAYOUT_OUTPUT)
