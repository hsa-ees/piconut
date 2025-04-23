#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2023 Felix Wagner <felix.wagner1@hs-augsburg.de
#                2024 Lukas Bauer <lukas.bauer1.tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#  Description:
#    Makefile for the Module Common, which provides configuration and helpter
#    functions for other modules. Usually the only dependency of low level modules
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

################ MAKEFILE FOR: common module ONLY ##############################

# MODULE_SRC:               Sources that are used for Simulation as well as
#                           synthesis. (including the testbench/sc_main file)
MODULE_SRC = base.cpp

# TESTBENCH_SRC:            Source that contains tb (and thus sc_main)
TESTBENCH_SRC =

# SIM_SRC:                  Sources that are only used in simulationon
SIM_SRC = c_soft_peripheral.cpp

# MODULE_INCLUDES:          Include directives for the compiler (e.g. -I..)
MODULE_INCLUDES =

# MODULE_CFLAGS:            Additional compiler flags this module needs
MODULE_CFLAGS =

# MODULE_LDFLAGS:           Additional linker flags this module needs
MODULE_LDFLAGS =



################################################################################
##### DO NOT EDIT FROM HERE ON #################################################
################################################################################

################# Module Makefile ##############################################
SYSC_MODULE_DIR:= $(abspath $(CURDIR))
PICONUT_HOME ?= $(CURDIR)/../../

################# Automatic Name Generation ####################################
# Get Module name from folder name
MODULE_NAME = $(subst module_,,$(lastword $(subst /, ,$(SYSC_MODULE_DIR))))


################# Directory Makefile Include ###################################
# disable original config target to supress warnings
DISABLE_ORIGINAL_CONFIG_TARGET = 1
include $(PICONUT_HOME)/directory-base.mk

################# Master Makefile Include ######################################
include $(SYSC_BASE_MK)

#################### CONFIG_H Parsing ##########################################
# The config module deviates slightly from all other module makefiles,
# as it additionaly has the task of creating the piconut-config.h file by
# parsing $(PN_CONFIG_MK) [defined in include $(MASTER_MAKEFILE)] and setting the
# according variables in $(CONFIG_H_TEMPLATE)

.PHONY: build-config
.ONESHELL:  build-config
build-config: $(CONFIG_H)
# Update local piconut-config.h
$(CONFIG_H): $(VERSION_ENV) $(PNS_CONFIG_MK) $(PN_CONFIG_MK) $(CONFIG_H_TEMPLATE)
	@echo $(PNS_CONFIG_MK)
	@$(eval CFG_NUT_MIMPID= $(shell grep CFG_NUT_MIMPID $(VERSION_ENV) | sed 's#^.*=##' ))
	@mkdir --parents $(@D)
	@echo "### Updating $(CONFIG_H)..."
	@sed \
	-e 's#{CFG_MEMU_BRAM_SIZE}#$(CFG_MEMU_BRAM_SIZE)#g' \
	-e 's#{CFG_REGFILE_SIZE}#$(CFG_REGFILE_SIZE)#g' \
	-e 's#{CFG_START_ADDRESS}#$(CFG_START_ADDRESS)#g' \
	-e 's#{CFG_CSR_BUS_ADR_WIDTH}#$(CFG_CSR_BUS_ADR_WIDTH)#g' \
	-e 's#{CFG_CSR_BUS_DATA_WIDTH}#$(CFG_CSR_BUS_DATA_WIDTH)#g' \
	-e 's#{CFG_DEBUG_DEBUGGER_BASE_ADDRESS}#$(CFG_DEBUG_DEBUGGER_BASE_ADDRESS)#g' \
	-e 's#{CFG_DEBUG_DEBUG_HANDLER_START_ADDRESS}#$(CFG_DEBUG_DEBUG_HANDLER_START_ADDRESS)#g' \
	-e 's#{CFG_DEBUG_OPENOCD_PORT}#$(CFG_DEBUG_OPENOCD_PORT)#g' \
	-e 's#{CFG_DEBUG_DMI_BUS_ADR_WIDTH}#$(CFG_DEBUG_DMI_BUS_ADR_WIDTH)#g' \
	-e 's#{CFG_WB_UART_BASE_ADDRESS}#$(CFG_WB_UART_BASE_ADDRESS)#g' \
	-e "s#{CFG_WB_UART_DISABLE_FIFO}#$(CFG_WB_UART_DISABLE_FIFO)#g" \
	$(CONFIG_H_TEMPLATE) > $(CONFIG_H)

.PHONY: clean-custom
clean-custom:
	@rm -f $(CONFIG_H)


