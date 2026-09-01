#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025 Niklas Sirch <niklas.sirch1@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
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


# Include PicoNut setup ...
include ../../../piconut.mk





################################################################################
#                                                                              #
#   Source Files                                                               #
#                                                                              #
################################################################################



# Module name ...
#   By convention,
#   - the module name is equal to the directory name,
#   - the top-level entity has the same name,
#   - the top-level entity is defined in a file named $(MODULE).cpp,
#   - the testbench is defined in a file named $(MODULE)_tb.cpp.
#   The following lines can be copied to new modules with very little
#   modifications. However, if required and adequate for some module, other
#   variables may be used. Any variables not starting with "PN_" and which are
#   not special variables (like DEBUG, VERBOSE, ... - see `make help`) are local
#   and can be defined freely in this file.
MODULE := $(PN_MODULE_NAME)


# Module sources that are used for simulation and for synthesis ...
MODULE_SRC := $(MODULE).cpp video_wb.cpp \
	color_translator.cpp framebuffer_ram.cpp \
	framebuffer_source.cpp vga_color_generator.cpp \
	vga_sync_generator.cpp vga_timings.cpp \
	line_column_counter.cpp color_palettes.cpp


# Testbench and its source(s), containing `sc_main` ...
TESTBENCHES :=\
	framebuffer_source_tb/framebuffer_source_tb \
	vga_sync_generator_tb/vga_sync_generator_tb color_translator_tb/color_translator_tb \
	line_column_counter_tb/line_column_counter_tb vga_timings_tb/vga_timings_tb \
	video_wb_tb/video_wb_tb video_tb/video_tb

TESTBENCH_SRC := $(addsuffix .cpp,$(TESTBENCHES))


##########
# Driver


# Define the library ...
DRIVER_NAME := $(PN_MODULE_NAME)_driver
DRIVER_SRC := $(DRIVER_NAME).c


DRIVER_OBJ := $(DRIVER_SRC:%.c=$(PN_MODULE_BUILD_DIR)/%.o)
DRIVER_LIB := $(PN_MODULE_BUILD_DIR)/lib$(MODULE)_driver.a


-include $(DRIVER_OBJ:%.o=%.d)

$(DRIVER_LIB): $(DRIVER_OBJ)





################################################################################
#                                                                              #
#   Simulation                                                                 #
#                                                                              #
################################################################################


# Define set of object files and output binaries ...
#   As many hardware modules, this section builds
#   a) a shared library with the simulation model of the module ('MODULE_*'),
#   b) a simulation binary based on a testbench using this library ('TESTBENCH_*').
MODULE_OBJ := $(MODULE_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
MODULE_LIB := $(PN_MODULE_BUILD_DIR)/sim/lib$(MODULE).a

TESTBENCH_OBJ := $(TESTBENCH_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
TESTBENCH_BIN := $(TESTBENCHES:%=$(PN_MODULE_BUILD_DIR)/sim/%)


# Automatic dependencies ...
#   The default compile command $(PN_HW_CC) generates a .d file for each .o file
#   containing the .h files it depends on. These files are included here.
-include $(MODULE_OBJ:%.o=%.d)
-include $(TESTBENCH_OBJ:%.o=%.d)


# Set object files to link ...
#   The PicoNut setup script ('piconut.mk') contains predefined (implicit) rules
#   for compiling and linking. If they are used (as in this file), an additional
#   linking rule must be supplied here to identify all object files the binary
#   depends on. The recipe must be left out, it is supplied by 'piconut.mk'.
#

#   dependencies of the library ...
$(MODULE_LIB): $(MODULE_OBJ)

#   dependencies of the executable ...
# One rule per testbench binary
define TESTBENCH_template
$(PN_MODULE_BUILD_DIR)/sim/$(1): \
	$(PN_MODULE_BUILD_DIR)/sim/$(1).o \
	$(MODULE_LIB)
endef

$(foreach tb,$(TESTBENCHES),$(eval $(call TESTBENCH_template,$(tb))))






#################### Exported global targets ###################################


# Build ...
.PHONY: build-lib
build-lib: $(MODULE_LIB)

.PHONY: build-tb
build-tb: $(TESTBENCH_BIN)

.PHONY: build-driver
build-driver: $(DRIVER_LIB)

build-sim: build-lib build-tb build-driver

# Shell version that does not contain make instructions
ifneq (0,$(VERBOSE))
  PN_HW_RUN_SH :=
else
  PN_HW_RUN_SH := f() { cmd="$$@"; echo "$(PN_BUILD_PREFIX)HW-RUN $$cmd"; out=`$$cmd 2>&1` || (echo "$$out" && exit 1) }; f
endif

# Verify ...
#   Run the main testbench.
.PHONY: verify-sim
verify-sim: $(TESTBENCH_BIN)
ifneq (0,$(VERBOSE))
	set -e; \
	for tb in $(TESTBENCH_BIN); do \
		$(PN_HW_RUN_SH) $$tb; \
	done
else
	@set -e; \
	for tb in $(TESTBENCH_BIN); do \
		$(PN_HW_RUN_SH) $$tb; \
	done
endif


# Install ...
.PHONY: install-all
install-all:
	$(PN_INSTALL_HW_INCLUDE) $(MODULE).h $(MODULE)_defs.h $(MODULE)_defs_hw.h
	$(PN_INSTALL_SW_INCLUDE) $(MODULE)_defs.h
	$(PN_INSTALL_SW_INCLUDE) $(DRIVER_NAME).h
	$(PN_INSTALL_SW_LIB) $(DRIVER_LIB)

.PHONY: install-sim
install-sim:
	$(PN_INSTALL_HW_LIB) $(MODULE_LIB)





################################################################################
#                                                                              #
#   Synthesis                                                                  #
#                                                                              #
################################################################################


FB_VERILOG_TEMPLATE := verilog/framebuffer_ram_template.v
FB_VERILOG := $(PN_MODULE_BUILD_DIR)/syn/framebuffer_ram.v

$(FB_VERILOG): $(FB_VERILOG_TEMPLATE) $(PN_HW_CONFIG_H)
	@mkdir -p $(dir $@)
	@sed -e 's/{RAM_SIZE}/$(PN_CFG_VIDEO_FB_SIZE)/g' \
	    verilog/framebuffer_ram_template.v > $(FB_VERILOG)



#################### Phase 1: SystemC synthesis (build-systemc) ####################


# The following definitions direct the synthesis by the Intel Compiler for
# SystemC (ICSC). ICSC comes with its own build system an C++ compiler, and it
# may handle a complete tree of modules at once.


# Module sources for ICSC ...
#   Module source files including the testbench.
#   Sources exported by an 'icsc-sources' recipe must be left out.
PN_SYSTEMC_SRC := $(MODULE_SRC) $(TESTBENCH_SRC)

# Exported module sources for ICSC ...
#   If this module does not provide a pre-synthesized RTL model, the following
#   recipe must output all source files to be exported to super modules that
#   use this one.
#   If this module exports a pre-synthesized netlist, the rule can be omitted.
.PHONY: icsc-sources
icsc-sources:
	@echo $(MODULE_SRC)

# Submodules for ICSC ...
#   Define the submodules that ICSC must process together with this one.
PN_SYSTEMC_SUBMODULES := $(PN_SUBMODULES)





#################### Phase 2: Gate-level synthesis (build-netlist) #############

# The following rules define a *reusable module*: Local synthesis is performed
# up to a gate-level, board-specific netlist, and this netlist is installed by
# the 'install-syn' target. The 'verify-syn' target (only) checks if the netlist
# can be built successfully.
#
# For a *simple module*, which is synthesized together with its super-module,
# the 'build-syn' and 'install-syn' targets can be left empty to save overall
# build time. The 'verify-syn' target, however, should be the same as for a
# reusable module to check for synthesis issues due to this current module.


# Synthesis for netlist ...
#   Name all RTL files (Verilog, VDHL) and presynthesized modules to be passed
#   to the synthesis tool. In most cases, (only) the SystemC synthesis result
#   (PN_SYSTEMC_OUTPUT) will be passed here.
PN_NETLIST_SRC := $(PN_SYSTEMC_OUTPUT)


# Declare dependencies ...
#   Declare the source files on which the netlist generation step depends.
#   Usually, the following line can be left untouched.
$(PN_NETLIST_OUTPUT): $(PN_NETLIST_SRC)





#################### Exported global targets ###################################


# Build ...
build-syn: build-driver


# Verify ...
verify-syn:


# Install ...
install-syn: $(FB_VERILOG) 
	$(PN_INSTALL_HW_SYN) $(FB_VERILOG)
