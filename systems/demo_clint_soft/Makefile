#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#  Description:
#    Makefile for building the reference system for simulation and synthesis.
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


# Declare that this is the root of a system ...
PN_BUILD_INIT := 1
PN_SYSTEM := .


# Submodules ...
#   Define the submodules this module depends on.
#
#   The build system will automatically build them by the 'build' target before
#   executing the 'build-all' recipe defined below. Submodules are "installed"
#   into $PN_BUILD_DIR, so that no search paths for compiling/linking must be
#   set here.
#   Only PN_HW_LDFLAGS must be extended to load the submodules when linking.
PN_SUBMODULES := hw


# Include PicoNut setup ...
include ../../piconut.mk



# Software to integrate ...
SW ?= sw/verification/interrupt_test


# Prebuild all required PicoNut modules ...
build-prepare:
	$(PN_PREBUILD) MODULES=hw/cpu:hw/peripherals/uart_soft:hw/cpu_peripherals/clint_soft:hw/cpu_peripherals/clint # for clint_defs.h
	$(PN_PREBUILD) MODULES=$(SW)





################################################################################
#                                                                              #
#   Software                                                                   #
#                                                                              #
################################################################################


SW_NAME := $(notdir $(SW))
SW_ELF := $(PN_STAGE_DIR)/sw/bin/$(SW_NAME).rv32





################################################################################
#                                                                              #
#   Simulation                                                                 #
#                                                                              #
################################################################################

.PHONY: sim
sim: build
	$(PN_MODULE_BUILD_DIR)/hw/sim/clint_soft_top_sim $(PN_STAGE_DIR)/sw/bin/$(SW_NAME).rv32



verify: build
	@output="$$( $(PN_MODULE_BUILD_DIR)/hw/sim/clint_soft_top_sim $(PN_STAGE_DIR)/sw/bin/$(SW_NAME).rv32 2>&1 )" || exit 1; \
	echo "$$output"; \
	if ! echo "$$output" | grep -q 'Interrupt demonstration completed successfully!'; then \
		exit 1; \
	fi





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

$(MEMBRANA_EMEM): $(SW_ELF)
	@$(PN_TOOLS_BIN)/pn-memgen $(MEMBRANA_EMEM_CONFIG) -i $(SW_ELF) -ib $(PN_CFG_MEMBRANA_EMEM_BASE) $@

.PHONY: build-emem
build-emem: $(MEMBRANA_EMEM)


# Synthesis inputs ...
#   Name all RTL files (Verilog, VDHL) and presynthesized modules to be passed
#   to the synthesis tool. In most cases, (only) the SystemC synthesis result
#   (PN_SYSTEMC_OUTPUT) will be passed here.
#
PN_NETLIST_SRC := $(PN_MODULE_BUILD_DIR)/hw/syn/hw.v \
  $(MEMBRANA_EMEM)




# Declare dependencies ...
#   Declare the source files on which the netlist and layout generation steps
#   depend. Usually, the following lines can be left untouched. Unused
#   Variables can be removed, additional relevant sources can be added.
$(PN_NETLIST_OUTPUT): $(PN_NETLIST_SRC)
$(PN_LAYOUT_OUTPUT): $(PN_LAYOUT_SRC)





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
	@echo "  SW    : Select the software to run in the refdesign system."
	@echo "          This either means the software to simulate (sim target) or"
	@echo "           the software to to include in the flashable bit-file."
	@echo "          Input in module form so from repository root to the directory of the Software Makefile"
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
