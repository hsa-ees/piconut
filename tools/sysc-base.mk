#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2023 Felix Wagner <felix.wagner1@hs-augsburg.de
#                2024 Lukas Bauer <lukas.bauer1@hs-augsburg.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#  Description:
#    This is a base makefile, that can be included in hardware module makefiles
#    in oder to clean up their contents. It contains targets for creating
#    systemc simulation binaries and synthesizing the respective modules1
#    See Makefile in the folder containing this file for an example usage.
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

# include makefile containing PicoNut directory structure information
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

################# Common Library ###############################################
CONFIG_H_TEMPLATE = $(HW_COMMON_DIR)/piconut-config.template.h

################# Module configuration #########################################
# MODULE_SRC:               Sources that are used for Simulation as well as
#                           synthesis. (including the testbench/sc_main file)
MODULE_SRC ?=

# TESTBENCH_SRC:            Source that contains tb (and thus sc_main)
TESTBENCH_SRC ?=

# SIM_SRC:                  Sources that are only used in simulationon
SIM_SRC ?=

# MEMU_VERSION:             Version of the MEMU module to use in the piconut
MEMU_VERSION ?=

#NUCLEUS_VERSION:             Version of the CORE module to use in the piconut
NUCLEUS_VERSION ?=

# PHERIPHERAL_MODULES:      List of peripheral modules to include into the system
PERIPHERAL_MODULES ?=

# MODULE_INCLUDES:          Include directives for the compiler (e.g. -I..)
MODULE_INCLUDES ?=

# MODULE_CFLAGS:            Additional compiler flags this module needs
MODULE_CFLAGS ?=

# MODULE_LDFLAGS:           Additional linker flags this module needs
MODULE_LDFLAGS ?=

# SYNTHESIS_TOPMODULE:      Defines the Top Module for synthesis as defined inside
#							the test bench cpp
SYNTHESIS_TOP_MODULE ?=

# USE_PICONUT_MODULE: 	 	If set to 1, the piconut module will be used
#							Used in systems to add the piconut module to the system
USE_PICONUT_MODULE ?= 0


# PICONUT_MEMU_NUCLEUS_SELECT: 	Selects the version of the MEMU and NUCLEUS module
#								used for the build of the piconut library
PICONUT_MEMU_NUCLEUS_SELECT ?=


################# Target definitions ###########################################
SIM_TARGET ?= $(MODULE_NAME).elf
V_TARGET ?= $(PNS_V_DIR)/$(MODULE_NAME).v
RTL_TARGET ?= $(CURDIR)/$(MODULE_NAME)_rtl.svg
TECH_ECP5_TARGET ?= $(CURDIR)/$(MODULE_NAME)_tech_ecp5.svg
SYSC_MODULE_LIB ?= $(PNS_LIB_DIR)/lib$(MODULE_NAME).a
SYSC_MODULE_HEADERS ?= $(wildcard *.h)
SYSC_MODULE_INC := $(addprefix $(PNS_INC_DIR)/, $(SYSC_MODULE_HEADERS))


################# Module Interdependency #######################################
# these variables ensure including and linking external modules works
# ALL MODULE FOLDERS MUST BE NAMED: $(MODULE_NAME)
# otherwise this will fail horribly

PICONUT_MODULE ?=

MODULE_DEPENDENCY_DIRS = $(addprefix $(SYSC_DIR)/, $(MODULE_DEPENDENCIES))
MODULE_DEPENDENCY_DIRS += $(addprefix $(SYSC_DIR)/piconut/memus/, $(MEMU_VERSION))
MODULE_DEPENDENCY_DIRS += $(addprefix $(SYSC_DIR)/piconut/nuclei/, $(NUCLEUS_VERSION))
MODULE_DEPENDENCY_DIRS += $(addprefix $(SYSC_DIR)/peripherals/, $(PERIPHERAL_MODULES))


ifeq ($(USE_PICONUT_MODULE), 1)
	PICONUT_MODULE = piconut
	MODULE_DEPENDENCY_DIRS += $(SYSC_DIR)/$(PICONUT_MODULE)
endif

# add the common module to the list of dependencies for simulation to prevent infinit recursion in synthesis
# common is added to synthesis in copy-syn-files-master
SIM_MODULE_DEPENDENCY_DIRS = $(MODULE_DEPENDENCY_DIRS) $(addprefix $(SYSC_DIR)/, common)
SIM_MODULE_DEPENDENCY_LIBS = $(addprefix $(PNS_LIB_DIR)/lib, $(addsuffix .a, common $(MEMU_VERSION) $(NUCLEUS_VERSION) $(PERIPHERAL_MODULES) $(PICONUT_MODULE)))
SIM_MODULE_DEPENDENCY_FILE = $(PNS_BUILD_DIR)/$(MODULE_NAME)_Makefile.deps
# SIM_SYSC_MODULE_HEADERS += $(SYSC_MODULE_HEADERS) $(SYSC_MODULE_HEADERS2)


############################ SystemC Defines ###################################

################# SystemC dependencies #########################################
# Overwrite the options in this section from SystemC config file
include $(SYSTEMC_CONFIG_MK)

SYSTEMC_HOME ?=
# SYSTEMC_LIB: location of libsystem.so
SYSTEMC_LIB ?=
# SYSTEMC_INCLUDE: location of  systemc.h
SYSTEMC_INCLUDE ?=
# SYSTEMC_CPP_VER: CPP Standard Version must match that of the SystemC library
SYSTEMC_CPP_VER ?=

################# General Compiler Configuration ###############################
CXX = g++
AR = ar
DEBUG_FLAGS ?= -O0 -g
CFLAGS = -MMD $(DEBUG_FLAGS) -I$(PNS_INC_DIR) $(SYSTEMC_INCLUDE) -pthread $(SYSTEMC_CPP_VER) $(MODULE_INCLUDES) $(MODULE_CFLAGS) $(PICONUT_MEMU_NUCLEUS_SELECT)
LDFLAGS = $(SYSTEMC_LIB) -lsystemc $(MODULE_LDFLAGS)


################# Simulation Setup  ############################################
# SIMBUILD will supress the __SYNTHESIS__ is set info
SIM_FLAGS ?= -DSIMBUILD
SYSC_MODULE_OBJ = $(addprefix $(PNS_BUILD_DIR)/,$(MODULE_SRC:.cpp=.o))
SYSC_TESTBENCH_OBJ = $(addprefix $(PNS_BUILD_DIR)/,$(TESTBENCH_SRC:.cpp=.o))
SIM_OBJ = $(addprefix $(PNS_BUILD_DIR)/,$(SIM_SRC:.cpp=.o))

# Default level for vcd tracing
TRACE_LVL ?= 2

# Programm options for the simulation or testbench
PROG_OPTS ?=


################# Synthesis Setup ##############################################
# SYNTH_LIB: The path derived from SYSTEMC_LIB will be correct
#    if SYSTEMC_HOME is set to the root of an icsc installation
# subst removes -L from $(SYSTEMC_LIB) to make it usable for linking

SYNTH_DIR ?= $(SYSC_MODULE_DIR)/sc_build
SYNTH_BUILD_DIR ?= $(SYSC_MODULE_DIR)/sc_build-ees
SYNTH_LIB ?= $(subst -L,,$(SYSTEMC_LIB))/libSCTool.so
SYNTH_SCTOOL_TEMPLATE ?= $(SYSC_DIR)/sctool.template.cpp
SYNTH_SCTOOL_SRC ?= $(SYSC_MODULE_DIR)/sctool.cpp
SYNTH_SCTOOL_OBJ ?= $(SYNTH_SCTOOL_SRC:.cpp=.o)
SYNTH_SCTOOL_INC := $(addprefix $(SYNTH_DIR)/, $(SYSC_MODULE_HEADERS))
SYNTH_CONFIG_H = $(SYNTH_DIR)/$(notdir $(CONFIG_H))

SYNTH_FLAGS ?= -D__SC_TOOL__

################################################################################
#################                Make Targets              #####################
################################################################################

################################################################################
######             General Targets       #######################################
################################################################################

#################  Default Target = help #######################################
.PHONY: all
all: help


#################  Version #####################################################
$(VERSION_ENV):
	@$(MAKE) -C $(PICONUT_HOME) update-version

#################  Config File #################################################
# target is to be overwritten in module_common
# DISABLE_ORIGINAL_CONFIG_TARGET is used to silence target already exists warning
ifndef DISABLE_ORIGINAL_CONFIG_TARGET
$(CONFIG_H): $(PN_CONFIG_MK) $(PNS_CONFIG_MK)
	@$(MAKE) -C $(HW_COMMON_DIR) build-config
endif
################################################################################
######             Simulation Targets    #######################################
################################################################################

#### Automatic dependencies ####################################################
# watch header files for changes by including dependency files
# created with -MMD at compilation
-include $(SIM_OBJ:%.o=%.d)

# general compilation setup
# dependency on $(SIM_MODULE_DEPENDENCY_LIBS) mainly ensures the presence
# of the module's .h files in $(PNS_INC_DIR)
$(PNS_BUILD_DIR)/%.o: %.cpp $(CONFIG_H) $(SYSC_MODULE_INC) $(SIM_MODULE_DEPENDENCY_LIBS)
	$(CXX) -c $(CFLAGS) $(SIM_FLAGS) $< -o $@


# phony target for human invocation
.PHONY: build-sim
build-sim: $(SIM_TARGET)

# phony target for human invocation
.PHONY: build-tb
build-tb: build-sim

$(SIM_TARGET): $(SYSC_MODULE_LIB)
	$(CXX) $(CFLAGS) $(LDFLAGS) $(TESTBENCH_SRC) -o $@ $(SYSC_MODULE_LIB) -L$(PNS_LIB_DIR) $(addprefix -l, $(PICONUT_MODULE) $(PERIPHERAL_MODULES) $(MEMU_VERSION) $(NUCLEUS_VERSION) common)
#         Note on linking done here. Linking the pn_common lib must be listed after the
#         target files themselves, as it is otherwise not searched in the nessecary order

# install all .h files present in the module root directory in the include folder  $(PNS_INC_DIR)
# additionally creates a folder to hold built object file
$(SYSC_MODULE_INC): $(SYSC_MODULE_HEADERS)
	@mkdir --parents $(PNS_BUILD_DIR)
	@ln -sf $(SYSC_MODULE_DIR)/$(@F) $@

# used to create missing library files from external makefile invocations
.PHONY: build-lib
build-lib: $(SYSC_MODULE_LIB) $(SYSC_MODULE_INC)
	@echo "   --> built library for Module $(MODULE_NAME)"

# create module library and place it in $(PNS_LIB_DIR)
# this should additionally be dependent on $(SYSTEMC_LIB)/libsystemc.so,
# but that complicates the install process
$(SYSC_MODULE_LIB): $(SYSC_MODULE_OBJ) $(SIM_OBJ) $(SIM_MODULE_DEPENDENCY_FILE)
	@mkdir --parents $(PNS_LIB_DIR)
	$(AR)  cr $@ $(SYSC_MODULE_OBJ) $(SIM_OBJ)

# helps to create the library file of external modules
# automatically calls make in the folder named like the missing module
# this seems to block all parallelism in the makefile.
# annoying but the only solution i could find
# technically you could leave this out, some things would then be created
# multiple times, but it would work and be a bit faster, however... ugly.
.NOTPARALLEL: $(SIM_MODULE_DEPENDENCY_LIBS)
# .SECONDEXPANSION is needed to enable shell execution in target dependency
.SECONDEXPANSION:
$(PNS_LIB_DIR)/lib%.a: $$(shell cat $$(PNS_BUILD_DIR)/$$*_Makefile.deps 2> /dev/null) $(PN_CONFIG_MK) $(PNS_CONFIG_MK)
	+$(MAKE) -C $(filter %$*, $(SIM_MODULE_DEPENDENCY_DIRS)) PICONUT_MEMU_NUCLEUS_SELECT="$(PICONUT_MEMU_NUCLEUS_SELECT)" build-lib

#simple target to run the created executable without knowing it's name
.PHONY: run-sim run-sim-opts
run-sim: $(SIM_TARGET)
	./$(SIM_TARGET)

run-sim-opts: $(SIM_TARGET)
	./$(SIM_TARGET) $(PROG_OPTS)

.PHONY: run-tb run-tb-trace run-tb-opts
run-tb: run-sim

run-tb-trace: $(SIM_TARGET)
	./$(SIM_TARGET) -t$(TRACE_LVL)

run-tb-opts: $(SIM_TARGET)
	./$(SIM_TARGET) $(PROG_OPTS)

################################################################################
######             Synthesis Targets    ########################################
################################################################################

# generate lists of relevant module files
# namely the cpp files mentioned in $(MODULE_SRC)
# and all .h files present in the root of the module dir
SYNTH_MODULE_SRC_CPP := $(addprefix $(SYNTH_DIR)/, $(notdir $(MODULE_SRC)))

#check if module name ends with _tb and if it is the case add all .h files in the parent directory to the list
SYNTH_MODULE_H ?=
ifeq ($(findstring _tb,$(MODULE_NAME)),_tb)
	SYNTH_MODULE_H := $(wildcard ../*.h)
endif

# add the files from the module's header list if it is a tb subfolder that is being synthesized
SYNTH_MODULE_SRC_H := $(addprefix $(SYNTH_DIR)/, $(notdir $(SYNTH_MODULE_H)))
SYNTH_MODULE_TB_CPP := $(addprefix $(SYNTH_DIR)/, $(notdir $(TESTBENCH_SRC)))

# used to check if the files that are needed for synthesis have changed
# this is used to prevent copy-syn-files-master from being called unnessecarily
# when no file has changed (used in $(V_TARGET) target)
SYNTH_DEP_FILES = $(wildcard $(SYNTH_DIR)/*.h) $(wildcard $(SYNTH_DIR)/*.cpp) $(wildcard $(SYNTH_DIR)/*.hpp) $(wildcard **/*.v)

# the module may depend on sources from another module.
# this target copies the files mentioned in the external module's MODULE_SRC
# variable to $(SYNTH_DIR)
.PHONY: copy-syn-files
copy-syn-files: $(SYNTH_MODULE_SRC_CPP) $(SYNTH_SCTOOL_INC) $(SYNTH_MODULE_SRC_H)
	@for dir in $(MODULE_DEPENDENCY_DIRS); do \
		$(MAKE) -C $${dir} copy-syn-files SYNTH_DIR=$(SYNTH_DIR);\
		$(MAKE) -C $${dir} custom-copy-verilog SYNTH_DIR=$(SYNTH_DIR);\
	done

# this is called in the makefile that was used by the user on console
# it additionally copies the tb sources and add the common module to the list of dependencies
.PHONY: copy-syn-files-master
copy-syn-files-master: copy-syn-files $(SYNTH_MODULE_TB_CPP)
	@$(MAKE) -C $(SYSC_DIR)/common copy-syn-files SYNTH_DIR=$(SYNTH_DIR)

# install all .h files present in the module root directory in the include folder  $(PNS_SYSC_INC_DIR)
# additionally creates a folder to hold built object file
$(SYNTH_SCTOOL_INC): $(SYSC_MODULE_HEADERS)
	@mkdir --parents $(SYNTH_DIR)
	@[ -L $(SYNTH_DIR)/$(@F) ] || ln -s $(abspath $(@F)) $(SYNTH_DIR)/$(@F);

# target for creating links to the sources in the dedicated
# synthesis directory
$(SYNTH_DIR)/%:
	@[ -d $(SYNTH_DIR) ] || mkdir --parents $(SYNTH_DIR); \
	[ -L $(SYNTH_DIR)/$(@F) ] || ln -s $(abspath $(filter %$* ,$(MODULE_SRC) $(SYNTH_MODULE_H) $(TESTBENCH_SRC))) $(SYNTH_DIR)/$*;


$(SYNTH_CONFIG_H): $(CONFIG_H)
	@mkdir --parents $(SYNTH_DIR); \
	[ -L $@ ] || ln -s $< $@;

# ensure presence of $(SYNTH_DIR)
$(SYNTH_DIR):
	mkdir --parents $(SYNTH_DIR)

# ensure presence of $(PNS_SYSC_SV_DIR)
$(PNS_SYSC_SV_DIR):
	mkdir --parents $(PNS_SYSC_SV_DIR)

# phony target for human consumption
.PHONY: build-verilog
build-verilog: $(V_TARGET)

# custom target to copy verilog files
.PHONY: custom-copy-verilog
custom-copy-verilog::



# convert systemc to verilog
$(V_TARGET):: $(PNS_V_DIR) $(SYNTH_CONFIG_H) $(SYNTH_DEP_FILES) | copy-syn-files-master custom-copy-verilog
	$(TOOLS_BIN_DIR)/pn-hls-icsc -s \
		-t $(SYNTHESIS_TOP_MODULE) \
		-i $(SYNTH_DIR)\
		-o $(dir $(V_TARGET))\
		-O $(basename $(notdir $(V_TARGET))) \
		$(foreach flag, $(PICONUT_MEMU_NUCLEUS_SELECT), -D $(flag))



# ensure presence of $(PNS_V_DIR)
$(PNS_V_DIR):
	mkdir --parents $(PNS_V_DIR)

################################################################################
######             Helpers              ########################################
################################################################################

.PHONY: sysc-lib-info
sysc-lib-info:
	@echo "Path to SystemC-Library: $(SYSTEMC_LIB)"
	@if [ -f $(SYSTEMC_LIB)/libsystemc.so ];then \
	readelf -p .comment /data/icsc_home/lib/libsystemc.so; \
	else \
	echo ""; \
	echo "can not find library at specified path"; \
	echo "You must either set ICSC_HOME if you want to use icsc libraries"; \
	echo "or SYSTEMC_LIB and SYSTEMC_INCLUDE if you want to specify another "; \
	echo "systemc library source."; \
	fi

## generate a schematic of the RTL design in svg format
$(RTL_TARGET): $(V_TARGET)
	$(TOOLS_BIN_DIR)/pn-rtl-schematic -r -s $(TOOLS_SHARE_DIR)/default.svg \
	-O $(notdir $@) -o $(dir $@) $<

PHONY: build-rtl-schematic
build-rtl-schematic: $(RTL_TARGET)

## generate a schematic of the techology for ep5 for the  design in svg format
$(TECH_ECP5_TARGET): $(V_TARGET)
	$(TOOLS_BIN_DIR)/pn-rtl-schematic -t -s $(TOOLS_SHARE_DIR)/default.svg \
	-O $(notdir $@) -o $(dir $@) $<

PHONY: build-tech-ecp5-schematic
build-tech-ecp5-schematic: $(TECH_ECP5_TARGET)

################# Make Dependency file for external checks #####################

$(SIM_MODULE_DEPENDENCY_FILE): $(MODULE_SRC) $(TESTBENCH_SRC) $(SYSC_MODULE_HEADERS)
	@echo $(addprefix $(SYSC_MODULE_DIR)/, $(MODULE_SRC) $(TESTBENCH_SRC) $(SYSC_MODULE_HEADERS)) > $(SIM_MODULE_DEPENDENCY_FILE)


################################################################################
######             Clean Targets    ############################################
################################################################################

# in this section there are the "normal" clean, and clean-all that only
# depend on other targets but do nothing themselves.
# the actual work is done in the -external targets
# otherwise calling make $(external_module) clean as done in clean-interdependencies
# and clean-all-interdependencies would cause unnessecary recursive invocations
# the clean-vcd command removes the trace file of the simulation

# normal clean, deletes build artefacts as well as library and include files
# basically everything to do with simulation
.PHONY: clean
clean: clean-all

#to be overwritten in the module make
.PHONY: clean-custom
clean-custom:

# see section beginning for an explaination
.PHONY: clean-external
clean-external: clean-custom clean-lib
	@rm -f *.[aod] $(SIM_OBJ) $(SIM_OBJ:%.o=%.d) $(SIM_TARGET) $(SCTOOL_TARGET)  \
					$(SYSC_MODULE_OBJ) $(SYSC_MODULE_OBJ:%.o=%.d)
	@rm -rf $(SYNTH_DIR) $(SYNTH_BUILD_DIR)
	@rm -rf *-ees $(RTL_TARGET) $(TECH_ECP5_TARGET)


.PHONY: clean-lib
clean-lib:
	@rm -rf $(SYSC_MODULE_INC) $(SIM_MODULE_DEPENDENCY_FILE)
	@rm -rf $(SYSC_MODULE_LIB)

# clean all additionally deletes artifacts from synthesis allow adding addtional cleaning steps from other make files
.PHONY: clean-all
clean-all:: clean-all-external clean-all-interdependencies

# see section beginning for an explaination
.PHONY: clean-all-external
clean-all-external: clean-external
	@rm -rf $(PNS_SYSC_SV_DIR)
	@rm -rf $(PNS_V_DIR)
	@rm -rf $(PNS_BUILD_DIR)
	@rm -rf $(PNS_LIB_DIR)

# see section beginning for an explaination
.PHONY: clean-interdependencies
clean-interdependencies:
	@for dir in $(SIM_MODULE_DEPENDENCY_DIRS); do \
		$(MAKE) -C $${dir} clean-external ;\
	done

# see section beginning for an explaination
.PHONY: clean-all-interdependencies
clean-all-interdependencies:
	@for dir in $(SIM_MODULE_DEPENDENCY_DIRS); do \
		$(MAKE) -C $${dir} clean-all-external ;\
	done

# see section beginning for an explaination
.PHONY: clean-vcd
clean-vcd:
	@rm -f *.vcd

################################################################################
######             HEEEEELP!!!      ############################################
################################################################################
.PHONY: help
help::
	@echo "Makefile for the SystemC Synthesis/Simulation"
	@echo "This makefile allows the user to build the libuart needed for the test benches"
	@echo "and the hls with icsc"
	@echo
	@echo "Usage: make [<target>] [<parameter>=<value> ...]"
	@echo
	@echo "Targets:"
	@echo
	@echo "  build-tb       : creates an executable testbench named $(SIM_TARGET)"
	@echo
	@echo "  build-lib      : builds the module library used to include it in higher modules"
	@echo "                  Parameters: "
	@echo "                    DEBUG_FLAGS='-g -pg'  "
	@echo "                    DEBUG_FLAGS='-g -pg'  "
	@echo
	@echo "  build-verilog  : runs pn-icsc-hls to synthesis SystemC module to Verilog"
	@echo
	@echo "  build-rtl-schematic : creates a schematic of the RTL design in svg format"
	@echo
	@echo "  build-tech-ecp5-schematic : creates a schematic of the techology for ep5 for the  design in svg format"
	@echo
	@echo "  run-tb         : executes the built simulation executable"
	@echo
	@echo "  run-tb-trace   : executes the built simulation executable with vcd tracing"
	@echo "  			      the parameter TRACE_LVL selects the level of tracing"
	@echo "					  [Default: 2]"
	@echo
	@echo "  run-tb-opts    : executes the built simulation executable with additional options"
	@echo "                  Parameters are set with PROG_OPTS"
	@echo
	@echo "  sysc-lib-info"
	@echo "  clean       : Cleanup files created by build-testbench and build-sim"
	@echo "  clean-vcd   : Remove the vcd file created by the simulation"
	@echo "  clean-all   : Same as clean, additionally clean build-syn artefacts"
	@echo
	@echo "  Parameters:"
	@echo
	@echo "  TRACE_LVL	 : Level of tracing for tracing in testbenches [Default: 2]"
	@echo "  PROG_OPTS	 : Options for the simulation executable"
