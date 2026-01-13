#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#  Description:
#    Makefile for the common software parts.
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


# Environment ...
PN_BUILD_COMMON := 0
PN_SW_OS = bare


# Include PicoNut setup ...
include ../../piconut.mk


# Module dependency ...
#   This is a layer-1 module: Request config (only).
build-prepare: prepare-config





################################################################################
#                                                                              #
#   Building                                                                   #
#                                                                              #
################################################################################


# Define the library ...
MODULE := piconut

SRC_C := syscalls.c
SRC_ASM := startup.S


# Custom CFLAGS and LDFLAGS (optional) ...
#   Adding additional compilation flags:
#     PN_SW_CFLAGS += -I/some/extra/include
#   Adding additional linker flags:
#     PN_SW_LDFLAGS += -lsome_extra_library


# Define set of object files and output binary ...
#   The names OBJ and BIN can be chosen randomly. In more complex setups
#   (e.g. with multiple apps or app versions), you may want to call them
#   like TETRIS_OBJ/TETRIS_BIN, PACMAN_OBJ/PACMAN_BIN.
OBJ := $(SRC_C:%.c=$(PN_MODULE_BUILD_DIR)/%.o) $(SRC_ASM:%.S=$(PN_MODULE_BUILD_DIR)/%.o)
LIB := $(PN_MODULE_BUILD_DIR)/lib$(MODULE).a


# Automatic dependencies ...
#   The default compile command $(PN_SW_CC) generates a .d file for each .o file
#   containing the .h files it depends on. These files are included here.
-include $(OBJ:%.o=%.d)


# Set object files to link ...
#   The PicoNut setup script ('piconut.mk') contains predefined rules for compiling
#   and linking. If they are used (as in this file), an additional linking rule
#   must be supplied here to identify all object files the binary depends on.
#   The recipe must be left out, it is supplied by 'piconut.mk'.
$(LIB): $(OBJ)





################################################################################
#                                                                              #
#   Exported Targets                                                           #
#                                                                              #
################################################################################


# Exported targets ...
.PHONY: build-all
build-all: $(LIB)


.PHONY: install-all
install-all:
	$(PN_INSTALL_SW_INCLUDE) piconut.h pn_encoding.h
	$(PN_INSTALL_SW_LIB) $(LIB) piconut.ld
