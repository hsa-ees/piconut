#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2023 Felix Wagner <felix.wagner1@hs-augsburg.de
#                2024 Lukas Bauer <lukas.bauer1.tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#  Description:
#    Makefile for the bluetooth implant demo
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

##### EDIT HERE
################# Software Configuration #########################################

# SOFTWARE_SRC:               Sources that are used for Simulation as well as
#                            synthesis. (including the testbench/sc_main file)
SOFTWARE_SRC ?= bootloader.c

# SOFTWARE_HEADERS:           Headers provided by the library that shall be copied
#                            to include
SOFTWARE_HEADERS ?=

# LIBRARY_DEPENDENCIES:      Paranut specific libraries, this software
#                            depends on. Mainly used to make shure the respective
#                            files are built
LIBRARY_DEPENDENCIES ?= libuzlib

# SOFTWARE_CFLAGS:            Additional compiler flags this software needs
SOFTWARE_CFLAGS ?=

# SOFTWARE_LDFLAGS:           Additional linker flags this software needs
SOFTWARE_LDFLAGS ?=

# SYSTEM:					  Defines the system the software is run on
#                             in simulation as well as on hardware
PN_SYSTEM ?= refdesign


################################################################################
##### DO NOT EDIT FROM HERE ON #################################################
################################################################################

################# Module Makefile ##############################################
APPLICATION_DIR:= $(abspath $(CURDIR))
PICONUT_HOME ?= $(CURDIR)/../../..

################# Automatic Name Generation ####################################
# Get Module name from folder name
SOFTWARE_NAME = $(lastword $(subst /, ,$(APPLICATION_DIR)))


################# Directory Makefile Include ###################################
include $(PICONUT_HOME)/directory-base.mk

################# Master Makefile Include ######################################
include $(APPLICATION_BASE_MK)

################################################################################
##### CUSTOM TARGETS ###########################################################
################################################################################
##### EDIT HERE

CFLAGS = $(DEBUGFLAGS) -march=$(CFG_MARCH) -mabi=ilp32 -I$(RISCV_COMMON_DIR) -I$(PNS_HAL_INC_DIR) $(SOFTWARE_CFLAGS)
LDFLAGS = $(CFLAGS) -static -nostartfiles $(SOFTWARE_LDFLAGS) -L$(PNS_HAL_LIB_DIR) $(LIBRARY_DEPENDENCY_CFLAGS)

MEM_DEST ?=

ifeq ($(BOOT_EXT), 1)
	CFLAGS += -DUART_EXTERNAL
	LDFLAGS += -lc startup/startup_external.S startup/syscalls.c -T startup/paranut_boot_external.ld
else
	LDFLAGS += -lc startup/startup.S startup/syscalls.c -T startup/paranut_boot.ld
endif

mem: $(ELF)
	$(TOOLS_BIN_DIR)/pn-extract-bin $(ELF) .text
	$(TOOLS_BIN_DIR)/pn-extract-bin $(ELF) .data

.PHONY:copy-mem
copy-mem: mem
	@echo $(MEM_DEST)
	cp -t $(MEM_DEST) *.mem

.PHONY:clean-mem
clean-mem:
	rm -f *.mem

clean:: clean-mem

