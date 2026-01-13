#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#  Description:
#    Makefile for the "common", which provides basic functionality for all other
#    modules.
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


# Don't build common ...
PN_BUILD_COMMON := 0

PN_SUBMODULES := riscv_disasm


# Include PicoNut setup ...
include ../../piconut.mk


# Module dependency ...
#   This is a layer-1 module: (Only) config is required.
build-prepare: prepare-config





################################################################################
#                                                                              #
#   Source Files                                                               #
#                                                                              #
################################################################################


# MODULE_SRC:        Sources that are used for simulation and for synthesis.
MODULE := pn_common
MODULE_SRC := \
	pn_base.cpp \
	pn_elf_reader.cpp \
	uart/pn_uart.cpp \
	c_soft_peripheral.cpp





################################################################################
#                                                                              #
#   Simulation                                                                 #
#                                                                              #
################################################################################


# Make sure that <piconut_base.h> is found from subdirectories ...
PN_HW_CFLAGS += -I$(PN_MODULE_SOURCE_DIR)


# Define set of object files and output binaries ...
MODULE_OBJ := $(MODULE_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
MODULE_LIB := $(PN_MODULE_BUILD_DIR)/sim/lib$(MODULE).a

SUBMOD_OBJ := \
	$(PN_MODULE_BUILD_DIR)/riscv_disasm/sim/libpn_riscv_disasm.a


# Automatic dependencies ...
-include $(MODULE_OBJ:%.o=%.d)


# Set object files to link ...
#   dependencies of the library ...
$(MODULE_LIB): $(MODULE_OBJ) $(SUBMOD_OBJ)





################################################################################
#                                                                              #
#   Global Targets                                                             #
#                                                                              #
################################################################################


# Build ...
build-sim: $(MODULE_LIB)


# Verify ...
# TBD


# Synthesis ...
.PHONY: icsc-sources
icsc-sources:
	@echo pn_base.cpp


# Install ...
.PHONY: install-all
install-all:
	$(PN_INSTALL_HW_INCLUDE) piconut.h piconut_base.h pn_base.h pn_module.h pn_elf_reader.h
	$(PN_INSTALL_HW_INCLUDE) pn_riscv_defs.h riscv_encoding.h
	$(PN_INSTALL_SW_INCLUDE) pn_riscv_defs.h riscv_encoding.h
	$(PN_INSTALL_HW_INCLUDE) uart/pn_uart.h
	$(PN_INSTALL_HW_INCLUDE) c_soft_peripheral.h

.PHONY: install-sim
install-sim:
	$(PN_INSTALL_HW_LIB) $(MODULE_LIB)
