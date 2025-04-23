#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2024 Lukas Bauer <lukas.bauer1@hs-augsburg.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#  Description:
#    This is a base makefile, that can be included in system makefiles
#    in oder to clean up their contents. It contains targets for synthesising
#    the verilog code for a specific board and flashing it to the board.
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

# Set the default Shell
SHELL = /bin/bash

# # include makefile containing PicoNut directory structure information
include $(PICONUT_HOME)/directory-base.mk

# include local configuration
# variables defined in the included config.mk overwrite those
# found in the global configuration
-include $(PNS_CONFIG_MK)

# include global configuration
include $(PN_CONFIG_MK)

################################################################################
#################                Configuration             #####################
################################################################################

# BOARD: 					The default target board for the synthesis
BOARD ?= ulx3s

# CONSTRAINT_FILE: 			The constraint file name for the target board
#							Located in systems/<system-name>/boards/<board-name>
CONSTRAINT_FILE_NAME ?=

# VERILOG_SRC: 				Path to addtitonal verilog sources that are needed
VERILOG_SRC ?=

# TOP_MODULE: 				The top verilog module name used for the board specific
#							synthesis
TOP_MODULE ?=


################# Target definitions ###########################################

# Predefinding the target names for the Board selection
ADDITIONAL_PREREQUISITES ?=
SYNTH_TARGET ?=
PNR_TARGET ?=
BIT_TARGET ?= $(PNS_SYNTH_DIR)/$(MODULE_NAME).bit

################# Boad commands ##################################################

# Predefinding the command strings for the Board selection
SYNTH_CMD ?=
PNR_CMD ?=
BIT_CMD ?=
FLASH_CMD ?=

################ Verilog sources ################################################

# Adding the ICSC syntheis result to the verilog sources
VERILOG_SRC += $(V_TARGET)

################ Constraint file #################################################

CONSTRAINT_FILE ?= $(PNS_BOARDS_DIR)/$(BOARD)/$(CONSTRAINT_FILE_NAME)


################ Boad selection ##################################################

# Defining the commands and targets for the ULX3S Board synthesis

ifeq ($(BOARD), ulx3s)
	# Synthesis
	SYNTH_TARGET := $(PNS_SYNTH_DIR)/$(MODULE_NAME).json
	SYNTH_CMD := yosys -p "read_verilog $(VERILOG_SRC); hierarchy -top $(TOP_MODULE); \
				 synth_ecp5 -json $(SYNTH_TARGET)"

	# Place and Route
	PNR_TARGET := $(PNS_SYNTH_DIR)/$(MODULE_NAME).cfg
	PNR_CMD := nextpnr-ecp5 --json $(SYNTH_TARGET) --textcfg $(PNR_TARGET)\
				--85k --package CABGA381 --lpf $(CONSTRAINT_FILE) --lpf-allow-unconstrained

	# Bit file generation
	BIT_CMD := ecppack --compress --input $(PNR_TARGET) --bit $(BIT_TARGET)

	# Flash
	FLASH_CMD := fujprog $(BIT_TARGET)
endif

ifeq ($(BOARD), orangecrab)
	# Synthesis
	SYNTH_TARGET := $(PNS_SYNTH_DIR)/$(MODULE_NAME).json
	SYNTH_CMD := yosys -p "read_verilog $(VERILOG_SRC); hierarchy -top $(TOP_MODULE); \
				 synth_ecp5 -json $(SYNTH_TARGET)"

	# Place and Route
	PNR_TARGET := $(PNS_SYNTH_DIR)/$(MODULE_NAME).cfg
	PNR_CMD := nextpnr-ecp5 --json $(SYNTH_TARGET) --textcfg $(PNR_TARGET)\
			   --85k --package CSFBGA285 --lpf $(CONSTRAINT_FILE)\
			   --freq 38.8

	# Bit file generation
	BIT_TARGET := $(PNS_SYNTH_DIR)/$(MODULE_NAME).dfu
	BIT_TARGET_TEMP := $(PNS_SYNTH_DIR)/$(MODULE_NAME).bit
	BIT_CMD := ecppack --compress --freq 38.8 --input $(PNR_TARGET) --bit $(BIT_TARGET_TEMP)\
			   && cp $(BIT_TARGET_TEMP) $(BIT_TARGET)\
			   && dfu-suffix -v 1209 -p 5af0 -a $(BIT_TARGET)

	# Flash
	FLASH_CMD := -dfu-util -a 0 -D $(BIT_TARGET)
endif

ifeq ($(BOARD), zybo-z7020)
	#Synthesis
	SYNTH_TARGET := $(PNS_SYNTH_DIR)/viv-syn/$(MODULE_NAME)-synth.dcp
	SYNTH_CMD := mkdir -p $(PNS_SYNTH_DIR)/viv-syn && cd $(PNS_SYNTH_DIR)/viv-syn && \
		sed \
		-e 's|{VERILOG_SRC}|$(VERILOG_SRC)|g' \
		-e 's|{TOP_MODULE}|$(TOP_MODULE)|g' \
		-e 's|{CONSTRAINT_FILE}|$(CONSTRAINT_FILE)|g' \
		-e 's|{SYNTH_TARGET}|$(SYNTH_TARGET)|g' \
		$(PNS_BOARDS_DIR)/$(BOARD)/synth-template.tcl > $(PNS_SYNTH_DIR)/viv-syn/synth.tcl && \
		$(TOOLS_BIN_DIR)/pn-vivado $(PNS_SYNTH_DIR)/viv-syn/synth.tcl \
		&& rm $(PNS_SYNTH_DIR)/viv-syn/synth.tcl

	# Place and Route
	PNR_TARGET := $(PNS_SYNTH_DIR)/viv-impl/$(MODULE_NAME)-impl.dcp
	PNR_CMD := mkdir -p $(PNS_SYNTH_DIR)/viv-impl && cd $(PNS_SYNTH_DIR)/viv-impl && \
		sed \
		-e 's|{SYNTH_TARGET}|$(SYNTH_TARGET)|g' \
		-e 's|{PNR_TARGET}|$(PNR_TARGET)|g' \
		$(PNS_BOARDS_DIR)/$(BOARD)/pnr-template.tcl > $(PNS_SYNTH_DIR)/viv-impl/pnr.tcl && \
		$(TOOLS_BIN_DIR)/pn-vivado $(PNS_SYNTH_DIR)/viv-impl/pnr.tcl \
		&& rm $(PNS_SYNTH_DIR)/viv-impl/pnr.tcl

	# Bit file generation
	BIT_TARGET := $(PNS_SYNTH_DIR)/$(MODULE_NAME).bit
	BIT_CMD := mkdir -p $(PNS_SYNTH_DIR)/viv-bit && cd $(PNS_SYNTH_DIR)/viv-bit && \
		sed \
		-e 's|{PNR_TARGET}|$(PNR_TARGET)|g' \
		-e 's|{BIT_TARGET}|$(BIT_TARGET)|g' \
		$(PNS_BOARDS_DIR)/$(BOARD)/bit-template.tcl > $(PNS_SYNTH_DIR)/viv-bit/bit.tcl && \
		$(TOOLS_BIN_DIR)/pn-vivado $(PNS_SYNTH_DIR)/viv-bit/bit.tcl \
		&& rm $(PNS_SYNTH_DIR)/viv-bit/bit.tcl && ln -sf $(PNS_SYNTH_DIR)/viv-bit/$(MODULE_NAME).bit $(BIT_TARGET)

	# Flash
	FLASH_CMD := cd $(PNS_SYNTH_DIR)/vivado && \
		source /opt/Xilinx/Vivado/2019.1/settings64.sh && \
		$(TOOLS_BIN_DIR)/pn-xil-program $(BIT_TARGET)


endif

ifeq ($(BOARD), olimex-gma1)
	# Synthesis
	SYNTH_TARGET := $(PNS_SYNTH_DIR)/$(MODULE_NAME).v
	SYNTH_CMD := yosys -p "read_verilog $(VERILOG_SRC); \
				 synth_gatemate -top $(TOP_MODULE) -nomx8 -vlog $(SYNTH_TARGET)"

	# Place and Route
	PNR_TARGET := $(PNS_SYNTH_DIR)/$(MODULE_NAME).cfg
	PNR_CMD := p_r -i $(SYNTH_TARGET) -o $(PNR_TARGET) -ccf $(CONSTRAINT_FILE) -om 3 +uCIO && \
		mv $(PNS_SYNTH_DIR)/$(MODULE_NAME)_00.cfg $(PNS_SYNTH_DIR)/$(MODULE_NAME).cfg

	# Bit file generation
	BIT_TARGET := $(PNS_SYNTH_DIR)/$(MODULE_NAME)_bit.cfg
	BIT_CMD := cp $(PNR_TARGET) $(BIT_TARGET)

	# Flash
	FLASH_CMD := openFPGALoader -c dirtyJtag $(BIT_TARGET)
endif


################################################################################
#################                Make Targets              #####################
################################################################################

################################################################################
######             Synthesis Targets    ########################################
################################################################################

# Running the synthesis command specified for the target board
$(SYNTH_TARGET): $(VERILOG_SRC) $(PNS_SYNTH_DIR) $(ADDITIONAL_PREREQUISITES)
	$(SYNTH_CMD)

# Running the place and route command specified for the target board
$(PNR_TARGET): $(SYNTH_TARGET)
	$(PNR_CMD)

# Running the bit file generation command specified for the target board
$(BIT_TARGET): $(PNR_TARGET)
	$(BIT_CMD)

# Create the synthesis directory
$(PNS_SYNTH_DIR):
	mkdir -p $(PNS_SYNTH_DIR)

# Creating a phony targets for the synthesis
.PHONY: build-synth build-pnr build-bit
build-synth: $(SYNTH_TARGET)

build-pnr: $(PNR_TARGET)

build-bit: $(BIT_TARGET)


################################################################################
######             Flash Targets    ########################################
################################################################################

# Running the simulation command specified for the target board
.PHONY: flash
flash: $(BIT_TARGET)
	$(FLASH_CMD)


################################################################################
######             Clean Targets        #######################################
################################################################################

.PHONY: clean-synth

# Adding cleaning target for the synthesis directory
clean-synth:
	@rm -rf $(PNS_SYNTH_DIR)

# Adding cleaning target for the synthesis results to default clean target
clean-all:: clean-synth

################################################################################
######             HEEEEELP!!!      ############################################
################################################################################
help::
	@echo "Makefile Synthesis and Flashing"
	@echo "This makefile allows the user to build the libuart needed for the test benches"
	@echo "and the hls with icsc"
	@echo
	@echo "Usage: make [<target>] [<parameter>=<value> ...]"
	@echo
	@echo "Targets:"
	@echo
	@echo "  build-synth    : synthesises the verilog sources with the tool specefied for the"
	@echo "                   selected board."
	@echo
	@echo "  build-pnr      : runs the place and route tool for the selected board"
	@echo "				      a different constraint file can be selected by setting"
	@echo "					  the CONSTRAINT_FILE_NAME variable"
	@echo
	@echo "  build-bit      : generates the bitstream for the selected board"
	@echo
	@echo "  flash		    : flashes the generated bitstream to the selected board"
	@echo
	@echo "  clean          : Cleanup files created by all build targets"
	@echo "  clean-synth    : Cleanup files created by build-synth, build-pnr and build-bit"
	@echo
	@echo "  Parameters:"
	@echo
	@echo "  BOARD:                The target board for the synthesis. Default is given in the Makefile"
	@echo "  CONSTRAINT_FILE_NAME: The name of the constraint file for the selected board"
	@echo "                        Default is given in the Makefile"

