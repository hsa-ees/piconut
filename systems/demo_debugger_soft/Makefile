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
SW ?= sw/apps/hello_piconut


# Prebuild all required PicoNut modules ...
build-prepare:
	$(PN_PREBUILD) MODULES=hw/cpu:hw/cpu_peripherals/debugger_soft:hw/peripherals/uart_soft:$(SW)





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
	$(PN_MODULE_BUILD_DIR)/hw/sim/demo_debugger_soft_sim $(PN_STAGE_DIR)/sw/bin/$(SW_NAME).rv32


.PHONY: run-hello
run-hello: sim








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



