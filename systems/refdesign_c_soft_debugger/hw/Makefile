#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2023 Felix Wagner <felix.wagner1@hs-augsburg.de
#                2024 Lukas Bauer <lukas.bauer1.tha.de>
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

##### EDIT HERE
################# Module Configuration #########################################

# MODULE_SRC:               Sources that are used for Simulation as well as
#                           synthesis. (including the testbench/sc_main file)
MODULE_SRC =

# TESTBENCH_SRC:            Source that contains tb (and thus sc_main)
TESTBENCH_SRC = top_tb.cpp

# SIM_SRC:                  Sources that are only used in simulationon
SIM_SRC = top.cpp

# MEMU_VERSION:             Version of the MEMU module to use in the piconut
MEMU_VERSION = simonlymemu

#NUCLEUS_VERSION:             Version of the CORE module to use in the piconut
NUCLEUS_VERSION = minimalnucleus

# PHERIPHERAL_MODULES:      List of peripheral modules to include into the system
PERIPHERAL_MODULES = \
	c_soft_uart \
	c_soft_debugger \
	c_remote_bitbang

# MODULE_INCLUDES:          Include directives for the compiler (e.g. -I..)
MODULE_INCLUDES =

# MODULE_CFLAGS:            Additional compiler flags this module needs
MODULE_CFLAGS =

# MODULE_LDFLAGS:           Additional linker flags this module needs
MODULE_LDFLAGS =

# SYNTHESIS_TOPMODULE:      Defines the Top Module for synthesis as defined inside
#							the test bench cpp
SYNTHESIS_TOP_MODULE =

# PICONUT_MEMU_NUCLEUS_SELECT: 	Selects the version of the MEMU and NUCLEUS module
#								used for the build of the piconut library (don't touch)
PICONUT_MEMU_NUCLEUS_SELECT = -D__$(shell echo ${NUCLEUS_VERSION} | tr '[:lower:]' '[:upper:]')__ \
							  -D__$(shell echo ${MEMU_VERSION} | tr '[:lower:]' '[:upper:]')__

# name the sim target different from the module name for
# historical reasons
SIM_TARGET = pn-sim

# BOARD: 					The default target board for the synthesis
BOARD ?= ulx3s

# CONSTRAINT_FILE: 			The constraint file name for the target board
#							Located in systems/<system-name>/boards/<board-name>
CONSTRAINT_FILE_NAME ?= ulx3s.lpf

# VERILOG_SRC: 				Path to addtitonal verilog sources that are needed
VERILOG_SRC = $(PNS_V_DIR)/pll.v

# TOP_MODULE: 				The top verilog module name used for the board specific
#							synthesis
TOP_MODULE = m_top

# CUSTOM TARGETS OPTIONS
# INPUT_CLK_FEQ:            The frequency of the input clock for the PLL
INPUT_CLK_FEQ ?= 25

# PLL_CLK_FEQ:              The frequency of the output clock for the PLL
PLL_CLK_FEQ ?= 5

################################################################################
##### DO NOT EDIT FROM HERE ON #################################################
################################################################################

################# Module Makefile ##############################################
SYSC_MODULE_DIR:= $(abspath $(CURDIR))
PICONUT_HOME ?= $(CURDIR)/../../..

################# Automatic Name Generation ####################################
# Get Module name from folder name
MODULE_NAME = $(lastword $(subst /, ,$(abspath $(SYSC_MODULE_DIR)/..)))
PNS_DIR = $(abspath $(SYSC_MODULE_DIR)/..)
PN_SYSTEM = $(MODULE_NAME)

################# System Configuration  #########################################

USE_PICONUT_MODULE = 1


################# Directory Makefile Include #####################################
include $(PICONUT_HOME)/directory-base.mk

################# Master Makefile Include #####################################
include $(SYSC_BASE_MK)
include $(SYN_BASE_MK)

# CUSTOM TARGETS

$(PNS_V_DIR)/pll.v: $(PNS_V_DIR)
	ecppll -n pll --clkin_name clk_in -i $(INPUT_CLK_FEQ) --clkout0_name clk_out -o $(PLL_CLK_FEQ) -f $@