#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
#                2024 Lukas Bauer <lukas.bauer1@hs-augsburg.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#  Description:
#    This is a master file covering multiple board definitions.
#    In the future, this file may be divided into multiple files, each located
#    in the respective board subdirectory.
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

# TBD: After all TBDs are resolved: Consider splitting this file into separate
#      ones per board, located in the respective board directory.


# The board file must define:
#
#   PN_BOARD_CONSTRAINTS (default), unless already set
#
#   PN_NETLIST_OUTPUT, based on PN_NETLIST_OUTPUT_BASE
#     - file name of the synthesis output (the netlist)
#
#   PN_NETLIST_CMD
#     - rtl to gate-level synthesis command
#
#   PN_NETLIST_REPORT
#     - command for a short, 2-line report (one line area utilization, one line timing)
#
#   PN_LAYOUT_OUTPUT, based on PN_LAYOUT_OUTPUT_BASE and PN_BOARD_CONSTRAINTS
#     - file name of the layout output (bitfile for FPGAs)
#
#   PN_LAYOUT_CMD
#     - command to produce the final layout from the netlist
#
#   PN_LAYOUT_REPORT
#     - command for a short, 2-line report (one line area utilization, one line timing)
#
#   PN_PROGRAM_CMD
#     - command for FPGA programming
#
# The board file may define additional targets, e.g. for synthesis scripts.
# Such scripts should have a name with the prefix PN_NETLIST_OUTPUT_BASE or
# PN_LAYOUT_OUTPUT_BASE, respectively. And PN_NETLIST_OUTPUT or PN_LAYOUT_OUTPUT,
# respectively, must depend on them.





################################################################################
#                                                                              #
#   Generic (Yosys)                                                            #
#                                                                              #
################################################################################


# Special "board" that supports synthesis and circuit visualization for a generic
# gate-level target. Layout and programming are unsupported.

ifeq ($(PN_CFG_BOARD),generic)
  # TBD+: Implement "generic" board;
  #       Netlist should be the generic netlist (see `ees-synthesize.oss -g`);
  #       Layout should lead to a "not available" message
  $(error Board 'generic' not defined yet.)
endif





################################################################################
#                                                                              #
#   ULX3S (Yosys, ECP5 85k)                                                    #
#                                                                              #
################################################################################

# TBD+: Test the board!
# TBD+: Check the constraints file: It should enable all I/Os in a failsafe way for which a failsafe activation is possible.


ifeq ($(PN_CFG_BOARD),ulx3s)


# Default constraints ...
PN_BOARD_CONSTRAINTS ?= $(PN_BOARD_DIR)/ulx3s.lpf


# Synthesis ...
PN_NETLIST_OUTPUT := $(PN_NETLIST_OUTPUT_BASE).v
PN_NETLIST_CMD := yosys -m ghdl -s $(PN_NETLIST_OUTPUT_BASE).yosys
PN_NETLIST_REPORT := \
  sed '0,/Count including submodules/d' < $(PN_NETLIST_OUTPUT).log \
  | sort -rk 2 | awk '/^ *[0-9]+ *(DP16KD|LUT4|TRELLIS_FF)/ { printf "  %s:%s", $$2, $$1 }' && echo

PN_LAYOUT_SRC :=  $(PN_NETLIST_OUTPUT_BASE).json
  # Usually, PN_LAYOUT_SRC and PN_NETLIST_OUTPUT should be identical.
  # However, in this case, we use Verilog (.v) as the primary output format
  # (e.g. for pre-synthesized sub-modules) and .json as a secondary output,
  # which is used and required for place and route!
  #
  # NOTE[2025-11-06]:
  #   The .json format causes problems with modular synthesis.
  #   If two .json files of submodules (uart.ulx3s.json, membrana_hw.ulx3s.json)
  #   are read, Yosys quits with an error message saying that module "USRMCLK" is
  #   redefined.
  #   Using the Verilog file for layout generation causes problems, too.
  #
$(PN_LAYOUT_SRC): $(PN_NETLIST_OUTPUT)

# Synthesis script ...
$(PN_NETLIST_OUTPUT_BASE).yosys: $(PN_MODULE_BUILD_DIR)/syn/read_sources.yosys
	@cp $(PN_MODULE_BUILD_DIR)/syn/read_sources.yosys $@
	@echo "hierarchy -top $(PN_TOPLEVEL)" >> $@
	@echo "synth_ecp5" >> $@
	@echo "write_json $(PN_NETLIST_OUTPUT_BASE).json" >> $@
	@echo "write_verilog $(PN_NETLIST_OUTPUT)" >> $@
#~ 	@echo "sta" >> $@

$(PN_NETLIST_OUTPUT): $(PN_NETLIST_OUTPUT_BASE).yosys


# Place and Route ...
#
#   Converting Verilog to json first (2025-11-07: Such .json files do not work!):
#     $ yosys -r $(PN_TOPLEVEL) -o $(PN_NETLIST_OUTPUT_BASE).json $(PN_NETLIST_OUTPUT) && \
#
PN_PNR_OUTPUT := $(PN_LAYOUT_OUTPUT_BASE).cfg
PN_LAYOUT_OUTPUT := $(PN_LAYOUT_OUTPUT_BASE).bit
PN_LAYOUT_CMD := \
  nextpnr-ecp5 \
    --json $(PN_LAYOUT_SRC) --textcfg $(PN_PNR_OUTPUT) \
    --85k --package CABGA381 --lpf $(PN_BOARD_CONSTRAINTS) \
    --lpf-allow-unconstrained && \
  ecppack \
    --compress --input $(PN_PNR_OUTPUT) --bit $(PN_LAYOUT_OUTPUT)

PN_LAYOUT_REPORT := \
  echo " Device utilisation:" && \
  sed '0,/Device utilisation/d' < $(PN_LAYOUT_OUTPUT).log \
    | grep -E '(TRELLIS_(IO|FF|COMB|RAMW)|DP16KD|MULT18X18D|ALU54B):' \
    | sed -e 's/^Info: \t    //' -e 's/^Info://' && \
  grep "Max frequency for clock" $(PN_LAYOUT_OUTPUT).log | tail -1 | sed 's/^Info: */ /'


# Program ...
PN_PROGRAM_CMD = fujprog $(PN_LAYOUT_OUTPUT)


endif





################################################################################
#                                                                              #
#   OrangeCrab (Yosys, ECP5 85k)                                               #
#                                                                              #
################################################################################

# TBD+: Test the board!
# TBD+: Check the constraints file: It should enable all I/Os in a failsafe way for which a failsafe activation is possible.


ifeq ($(PN_CFG_BOARD),orangecrab)


# Default constraints ...
PN_BOARD_CONSTRAINTS ?= $(PN_BOARD_DIR)/orangecrab.lpf


# Synthesis ...
PN_NETLIST_OUTPUT := $(PN_NETLIST_OUTPUT_BASE).v
PN_NETLIST_CMD := yosys -s $(PN_NETLIST_OUTPUT_BASE).yosys

PN_LAYOUT_SRC :=  $(PN_NETLIST_OUTPUT_BASE).json
  # See comment on PN_LAYOUT_SRC for the ULX3S board.
  # The primary output format is Verilog (.v). As a secondary output, a .json
  # file is written as well. The latter is used and required for place and route!
$(PN_LAYOUT_SRC): $(PN_NETLIST_OUTPUT)

# Synthesis script ...
$(PN_NETLIST_OUTPUT_BASE).yosys: $(PN_MODULE_BUILD_DIR)/syn/read_sources.yosys
	@cp $(PN_MODULE_BUILD_DIR)/syn/read_sources.yosys $@
	@echo "hierarchy -top $(PN_TOPLEVEL)" >> $@
	@echo "synth_ecp5" >> $@
	@echo "write_json $(PN_NETLIST_OUTPUT_BASE).json" >> $@
	@echo "write_verilog $(PN_NETLIST_OUTPUT)" >> $@

$(PN_NETLIST_OUTPUT): $(PN_NETLIST_OUTPUT_BASE).yosys


# Place and Route ...
PN_PNR_OUTPUT := $(PN_LAYOUT_OUTPUT_BASE).cfg
PN_LAYOUT_OUTPUT := $(PN_LAYOUT_OUTPUT_BASE).dfu
PN_LAYOUT_CMD := \
  nextpnr-ecp5 \
    --json $(PN_LAYOUT_SRC) --textcfg $(PN_PNR_OUTPUT) \
    --85k --package CSFBGA285 --lpf $(PN_BOARD_CONSTRAINTS) \
    --freq 38.8 && \
  ecppack \
    --compress --freq 38.8 --input $(PN_PNR_OUTPUT) \
    --bit $(PN_LAYOUT_OUTPUT) && \
  dfu-suffix -v 1209 -p 5af0 -a $(PN_LAYOUT_OUTPUT)

PN_LAYOUT_REPORT := \
  echo " Device utilisation:" && \
  sed '0,/Device utilisation/d' < $(PN_LAYOUT_OUTPUT).log \
    | grep -E '(TRELLIS_(IO|FF|COMB|RAMW)|DP16KD|MULT18X18D|ALU54B):' \
    | sed -e 's/^Info: \t    //' -e 's/^Info://' && \
  grep "Max frequency for clock" $(PN_LAYOUT_OUTPUT).log | tail -1 | sed 's/^Info: */ /'


# Program ...
PN_PROGRAM_CMD = -dfu-util -a 0 -D $(PN_LAYOUT_OUTPUT)


endif





################################################################################
#                                                                              #
#   ZYBO Z7-20 (Xilinx, Zynq 7020)                                             #
#                                                                              #
################################################################################

# TBD+: Eliminate hard-coded path: /opt/Xilinx/Vivado/2019.1/settings64.sh
# TBD+: Test the board!
# TBD+: Check the constraints file: It should enable all I/Os in a failsafe way for which a failsafe activation is possible.
# TBD+: Support VHDL and presynthesized modules (check non-Verilog sources)


ifeq ($(PN_CFG_BOARD),zybo-z7020)


# Default constraints ...
PN_BOARD_CONSTRAINTS ?= $(PN_BOARD_DIR)/zybo-z7020.xdc


# Synthesis ...
PN_NETLIST_OUTPUT := $(PN_NETLIST_OUTPUT_BASE)-synth.dcp
PN_NETLIST_CMD = \
  mkdir -p $(PN_MODULE_BUILD_DIR)/viv-syn && \
  cd $(PN_MODULE_BUILD_DIR)/viv-syn && \
  sed \
    -e 's|{VERILOG_SRC}|$(PN_NETLIST_SRC)|g' \
    -e 's|{TOP_MODULE}|$(PN_TOPLEVEL)|g' \
    -e 's|{CONSTRAINT_FILE}|$(PN_BOARD_CONSTRAINTS)|g' \
    -e 's|{PN_NETLIST_OUTPUT}|$(PN_NETLIST_OUTPUT)|g' \
    $(PN_BOARD_DIR)/synth-template.tcl \
    > $(PN_MODULE_BUILD_DIR)/viv-syn/synth.tcl && \
  $(PN_TOOLS_BIN)/pn-vivado $(PN_MODULE_BUILD_DIR)/viv-syn/synth.tcl

# Place and Route ...
PN_PNR_OUTPUT := $(PN_MODULE_BUILD_DIR)/viv-impl/$(MODULE_NAME)-impl.dcp
PN_LAYOUT_OUTPUT := $(PN_LAYOUT_OUTPUT_BASE).bit
PN_LAYOUT_CMD = \
  mkdir -p $(PN_MODULE_BUILD_DIR)/viv-impl && cd $(PN_MODULE_BUILD_DIR)/viv-impl && \
  sed \
    -e 's|{PN_NETLIST_OUTPUT}|$(PN_NETLIST_OUTPUT)|g' \
    -e 's|{PNR_TARGET}|$(PN_PNR_OUTPUT)|g' \
    $(PNS_BOARD_DIR)/pnr-template.tcl \
    > $(PN_MODULE_BUILD_DIR)/viv-impl/pnr.tcl && \
  $(PN_TOOLS_BIN)/pn-vivado $(PN_MODULE_BUILD_DIR)/viv-impl/pnr.tcl && \
  mkdir -p $(PN_MODULE_BUILD_DIR)/viv-bit && \
  cd $(PN_MODULE_BUILD_DIR)/viv-bit && \
  sed \
    -e 's|{PNR_TARGET}|$(PN_PNR_OUTPUT)|g' \
    -e 's|{PN_LAYOUT_OUTPUT}|$(PN_LAYOUT_OUTPUT)|g' \
    $(PN_BOARD_DIR)/bit-template.tcl \
    > $(PN_MODULE_BUILD_DIR)/viv-bit/bit.tcl && \
  $(PN_TOOLS_BIN)/pn-vivado $(PN_MODULE_BUILD_DIR)/viv-bit/bit.tcl && \
  mv $(PN_MODULE_BUILD_DIR)/viv-bit/$(MODULE_NAME).bit $(PN_LAYOUT_OUTPUT)


# Program ...
PN_PROGRAM_CMD = \
  cd $(PN_MODULE_BUILD_DIR)/vivado && \
  source /opt/Xilinx/Vivado/2019.1/settings64.sh && \
  $(PN_TOOLS_BIN)/pn-xil-program $(PN_LAYOUT_OUTPUT)


endif





################################################################################
#                                                                              #
#   Olimex GateMate A1 (Yosys, GateMate CCGM1A1)                               #
#                                                                              #
################################################################################

# TBD+: Test the board!
# TBD+: Check the constraints file: It should enable all I/Os in a failsafe way for which a failsafe activation is possible.


ifeq ($(PN_CFG_BOARD),olimex-gma1)


# Default constraints ...
PN_BOARD_CONSTRAINTS ?= $(PN_BOARD_DIR)/zybo-z7020.xdc


# Synthesis ...
PN_NETLIST_OUTPUT := $(PN_NETLIST_OUTPUT_BASE).v
PN_NETLIST_CMD := yosys -s $(PN_NETLIST_OUTPUT_BASE).yosys


# Synthesis script ...
$(PN_NETLIST_OUTPUT_BASE).yosys: $(PN_MODULE_BUILD_DIR)/syn/read_sources.yosys
	@cp $(PN_MODULE_BUILD_DIR)/syn/read_sources.yosys $@
	@echo "synth_gatemate -top $(PN_TOPLEVEL) -nomx8 -vlog $(PN_NETLIST_OUTPUT)" >> $@

$(PN_NETLIST_OUTPUT): $(PN_NETLIST_OUTPUT_BASE).yosys


# Place and Route ...
PN_PNR_OUTPUT := $(PN_LAYOUT_OUTPUT_BASE).cfg
PN_LAYOUT_OUTPUT := $(PN_LAYOUT_OUTPUT_BASE)_bit.cfg
PN_LAYOUT_CMD = \
  p_r -i $(PN_NETLIST_OUTPUT) -o $(PN_PNR_OUTPUT) \
    -ccf $(PN_BOARD_CONSTRAINTS) -om 3 +uCIO && \
  mv $(PN_LAYOUT_OUTPUT_BASE)_00.cfg $(PN_LAYOUT_OUTPUT_BASE)_bit.cfg


# Program ...
PN_PROGRAM_CMD = openFPGALoader -c dirtyJtag $(PN_LAYOUT_OUTPUT)


endif
