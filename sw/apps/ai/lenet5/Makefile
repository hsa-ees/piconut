#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025-2026 Gundolf Kiefer <gundolf.kiefer@tha.de>
#     Technische Hochschule Augsburg / Technical University of Applied Sciences
#
#  Description:
#    Makefile for the LeNet5 RISC-V AI demonstration code.
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


# Base ISA (for compiling) ...
RV_ISA := rv32im_zve32x

# Base ISA (for simulation) ...
RV_ISA_VEXT := rv32imv_zvl1024b_zicntr_zihpm



################################################################################
#                                                                              #
#   Configuration                                                              #
#                                                                              #
################################################################################


# Environment ...
PN_SW_OS = bare
#~ PN_SW_OS = std

PN_CFG_SYS_CODE_SIZE ?= 0x00080000
  # Size of the code (ROM) section in the linker script.
PN_CFG_SYS_RAM_SIZE ?= 0x00180000
  # Size of the RAM (data) section in the linker script.
#~ PN_CFG_SYS_STACK_SIZE := 0x00004000
  # Size of the stack section in the linker script.
PN_CFG_SYS_HEAP_SIZE ?= 0x00040000
  # Size of the heap section in the linker script.


# Include PicoNut setup ...
include ../../../../piconut.mk





################################################################################
#                                                                              #
#   Building                                                                   #
#                                                                              #
################################################################################


APP := $(PN_MODULE_NAME)
APP_SRC = main.c $(APP).c model_int16f8.c pn_support.c



######################### Building for RISC-V ##################################


RV_APP_SRC = $(APP_SRC) lenet5_asm.S test_images.c

RV_APP_SRC_VIRT = $(RV_APP_SRC:.S=.c)
RV_APP_OBJ = $(RV_APP_SRC_VIRT:%.c=$(PN_MODULE_BUILD_DIR)/%.o)

RV_APP_BIN = $(PN_MODULE_BUILD_DIR)/$(APP).rv32
RV_APP_SPIKE_BIN = $(PN_MODULE_BUILD_DIR)/$(APP)-spike.rv32


# Special CFLAGS ...
PN_SW_CFLAGS += -O2 -march=$(RV_ISA) -DARCH_RV32=1


# Linking ...
$(RV_APP_BIN): $(RV_APP_OBJ)


# Linking (for Spike) ...
$(RV_APP_SPIKE_BIN): $(RV_APP_OBJ)
	@mkdir -p $(dir $@)
ifneq (0,$(VERBOSE))
	$(PN_SW_LD) -o $@ $^ -march=rv32i -mabi=ilp32
else
	@echo $(PN_BUILD_PREFIX)SW-LD $(@:$(PN_MODULE_BUILD_DIR)/%=%); \
	$(PN_SW_LD) -o $@ $^ -march=rv32i -mabi=ilp32
endif


# Automatic dependencies ...
-include $(RV_APP_OBJ:.o=.d)



######################### Building for PC ######################################


OPENMP_FLAGS := -fopenmp

PC_CC := gcc
PC_CFLAGS := -Wall -g -MMD $(OPENMP_FLAGS) -DARCH_RV32=0 -O2
PC_LDFLAGS := $(OPENMP_FLAGS) -lm

PC_APP_SRC := $(APP_SRC) model_float.c test_images_10000.c # test_images.c

PC_APP_OBJ := $(PC_APP_SRC:%.c=$(PN_MODULE_BUILD_DIR)/%.x86.o)

PC_APP_BIN := $(PN_MODULE_BUILD_DIR)/$(APP).x86


# Compiling (PC) ...
$(PN_MODULE_BUILD_DIR)/%.x86.o: %.c
ifneq (0,$(VERBOSE))
	@mkdir -p $(dir $@)
	$(PC_CC) -c $(PC_CFLAGS) -o $@ $<
else
	@mkdir -p $(dir $@); \
	echo $(PN_BUILD_PREFIX)SW-CC-PC $<; \
	$(PC_CC) -c $(PC_CFLAGS) -o $@ $<
endif


# Linking (PC) ...
$(PC_APP_BIN): $(PC_APP_OBJ)
	@mkdir -p $(dir $@) && if test "$^" = ""; then echo "$(@:$(PN_BUILD_DIR)/%=%): Missing object files in makefile rule!"; exit 3; fi
ifneq (0,$(VERBOSE))
	$(PC_CC) -o $@ $^ $(PC_LDFLAGS)
else
	@echo $(PN_BUILD_PREFIX)SW-LD-PC $(@:$(PN_MODULE_BUILD_DIR)/%=%); \
	$(PC_CC) -o $@ $^ $(PC_LDFLAGS)
endif


# Automatic dependencies ...
-include $(PC_APP_OBJ:.o=.d)





################################################################################
#                                                                              #
#   Global & Exported Targets                                                  #
#                                                                              #
################################################################################


##### Exported #####

build-all: $(PC_APP_BIN) $(RV_APP_BIN)

install-all:
	$(PN_INSTALL_SW_BIN) $(RV_APP_BIN)


##### Additional Main Targets #####

.PHONY: run
run: $(PC_APP_BIN)
	$(PC_APP_BIN)

.PHONY: sim
sim: build-prepare .WAIT $(RV_APP_SPIKE_BIN)
	$(RISCV)/bin/spike --isa=$(RV_ISA_VEXT) pk $(RV_APP_SPIKE_BIN)

.PHONY: view
view: build-prepare .WAIT $(RV_APP_SPIKE_BIN)
	riscv64-unknown-elf-objdump -S $(RV_APP_SPIKE_BIN) | less $(ARGS)

.PHONY: debug
debug: build-prepare .WAIT $(RV_APP_SPIKE_BIN)
	$(RISCV)/bin/spike -d --isa=(RV_ISA_VEXT) pk $(RV_APP_BIN)
