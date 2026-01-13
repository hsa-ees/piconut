#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#  Description:
#    General definitions for the PicoNut build system.
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





# General Notes
# =============
#
# 1. Requirements
#
#   This script requires GNU Make 4.4 or later
#   (i.e. for the .WAIT special target, see https://lists.gnu.org/archive/html/info-gnu/2022-10/msg00008.html#:~:text=*%20New%20feature%3A%20The%20.WAIT%20special%20target)
#
#
# 2. Precedence of variable settings
#
#   All variables defined in this file using "=" or ":=" are considered read-only
#   and must not be set outside of this script.
#   For developers editing this file: Please mind the subtle differences in evaluation
#     ":=": Assignments are evaluated while the Makefile (and its includes) are read
#           in the order they appear (subsequent assignments override previous ones).
#     "=":  Assignments are evaluated at build-time, independent on the order they
#           appear.
#
#   For all other variables, the following order of precedence must be maintained
#   (highest priority first):
#     a) Command line
#     b) Shell environment
#     c) System configuration ($PN_SYSTEM/piconut-config.mk)
#     d) Installation configuration ($PN_INSTALLED_DIR/piconut-config.mk or $PN_SOURCE_DIR/piconut-config.mk)
#     e) Module configuration (inside .h files),
#        Board definition ($PN_BOARD_DIR)
#     f) General defaults (this file)
#
#   To guarantee this, the following must be taken care of:
#   - All assignments in the system configuration, installation configuration, modules
#     and in this file (c-f) must be conditional (e.g. by using "?=" or "#ifndef" in C).
#   - a) before b) is provided automatically by GNU make.
#   - This file must guarantee the ordering of c, d and f.
#   - Modules must not depend on board definitions and vice versa.





################################################################################
#                                                                              #
#   Build Settings                                                             #
#                                                                              #
################################################################################


# NOTE:
#   Settings without a 'PN_' prefix affect the build process and nothing else.
#   They are usually set on the command line or by the user's environment
#   during development, but not in the code.
#
#   Unlike other settings, these basic build settings do not have a "PN_" prefix.


# Default target ...
.PHONY: all
all: build


# Verbosity ...
#   By default (0), executed commands are abbreviated to improve the readability
#   of the build output. If set to 1, the original commands are printed.
VERBOSE ?= 0


# Debug level ...
#   Define the preset compilation options:
#   0: Release build, optimizations (usually -O3) enabled, debug infos may be missing
#   1: Debug build, debug infos are available, no optimizations that may complicate debugging
#   >= 2: Additional debug output (for future use)
DEBUG ?= 0


# Selected technologies ...
PN_BUILD_TECHS := sim:syn
ifneq (,$(TECHS))
  PN_BUILD_TECHS := $(TECHS)
endif
PN_BUILD_TECHS := $(subst :, ,$(PN_BUILD_TECHS))


# PicoNut installation (optional) ...
#   Define the absolute path to a system-wide or user-wide binary installation
#   of PicoNut.
#   If defined, PN_INSTALLED_DIR and PREFIX will be preset accordingly.
#   Usually, PICONUT is set by an environment variable, e.g. in the user's
#   .bashrc file.
#
#   Recommended settings are:
#       PICONUT := ~/.piconut    # for a user-local installation
#       PICONUT := /opt/piconut  # for a system-wide installation (may require root privileges)
#
PICONUT ?= ~/.piconut


# PicoNut build directory ...
#   Place where all built artefacts are written to. For optimum performance,
#   this should be on a local filesystem, ideally a RAM disk.
ifndef PICONUT_BUILD
  PICONUT_BUILD := /var/tmp/piconut-build
endif


# Installation destination ...
#   Define the place where the 'install' target places its files.
#   This is not to be confused with PN_INSTALLED_DIR, where preinstalled files
#   are read from (but never written to).
#
#   Unless given on the command line (or enviromnent) PREFIX is preset with
#     a) $(PN_SYSTEM_DIR)/piconut if a system is defined (PN_SYSTEM_DIR set) or
#     b) $(PICONUT) otherwise.
#
#PREFIX :=


# Board for synthesis targets ...
#   Select the hardware board and implicitly the target FPGA device or ASIC technology
#   to be used for synthesis targets.
#
#   TBD: Change to a generic technology as soon as the "generic" board is defined
#
PICONUT_BOARD ?= ulx3s





################################################################################
#                                                                              #
#   Module properties                                                          #
#                                                                              #
################################################################################


# Variables defining module properties, set inside the module Makefile before
# 'piconut.mk' is included.


# Submodules ...
#   By this variable, the module defines its submodules, if present.
#   The build system ensures that the targets 'build', 'install' and 'verify'
#   first process these submodules recursively, so that their outputs can
#   be used by this (main) module.
#   By default, there are no submodules.
PN_SUBMODULES ?=


# Pure hardware/software ...
#   By default, this master build file presents all rules for building hardware
#   and software together. This may lead to conflicts in some cases. For example,
#   .cpp files are considered to be hardware (SystemC), and .c files are considered
#   to be RISC-V software. To avoid conflicts, one or both of the following
#   variables can be set 0 to disable (automatic) rules not needed.
#   Modules without local code, just submodules, may disable both.
PN_BUILD_HW ?= 1
PN_BUILD_SW ?= 1


# Prebuilt common modules ...
#   Decide whether the common modules need to be built before the current module.
#   By default, the common modules are built. The only case where this is not
#   wanted are the common modules themselves and systems built based on an installation.
#   Note: If any submodule has set/left this active (1), the submodules must
#   leave it active, too.
PN_BUILD_COMMON ?= 1


# Build initializer / Root module ...
#   If set, force the build process to prebuilt the config and common layer modules.
#   Usually, these base modules are built automatically on the primary invocation
#   of `make`, which is adequate in most cases. Only if this mechanism fails,
#   this variable may be set.
#   (Default: 1 for the PicoNut source tree root or the current system's root, else 0)
#PN_BUILD_INIT ?=





################################################################################
#                                                                              #
#   Directories                                                                #
#                                                                              #
################################################################################


# Note: All *_DIR variables defined here contain absolute paths.


# PN_INSTALLED_DIR: Where an existing installation can be found (optional) ...
#   Directory where pre-installed libraries, headers and pre-synthesized modules
#   can be found. This is derived from the (environment) variable PICONUT.
#   Recommended settings are:
#
#   PICONUT := ~/.piconut     # for a personal installation
#   PICONUT := /opt/piconut   # for a machine-wide installation
#
ifneq (,$(PICONUT))
  PN_INSTALLED_DIR ?= $(realpath $(PICONUT))
endif





#################### Source tree and module identification #####################


# PN_SOURCE_DIR: PicoNut source tree (internal or external) ...
#   This is the root directory of the PicoNut source tree to be used.
#   By default, it is set automatically to the location of this `piconut.mk` file.
ifndef PN_SOURCE_DIR
  PN_SOURCE_DIR := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))
endif
export PN_SOURCE_DIR


# PN_MODULE_SOURCE_DIR: Sources of the current module ...
#   This is the directory of the module currently built.
PN_MODULE_SOURCE_DIR := $(realpath $(CURDIR))
export PN_MODULE_SOURCE_DIR


# PN_MODULE_ID: Module ID ...
#   The identifier of the module currently built. It determines the relative
#   path of build and installation artefacts.
#   For a module contained in the PicoNut source tree, this is the relative
#   path to the module.
#   For an external module, PN_MODULE_ID is set to PN_MODULE_SOURCE_DIR with a
#   leading '/' removed, which is basically the absolute path.
PN_MODULE_ID := $(PN_MODULE_SOURCE_DIR)
PN_MODULE_ID := $(PN_MODULE_ID:$(PN_SOURCE_DIR)/%=%)
  # relative path inside PN_SOURCE_DIR (or PN_MODULE_SOURCE_DIR unmodified if outside)
ifeq ($(PN_MODULE_ID),$(PN_SOURCE_DIR))
  PN_MODULE_ID :=
    # special case: Module is the root of the source tree: the MODULE ID is "" then.
endif
PN_MODULE_ID := $(PN_MODULE_ID:/%=%)
  # remove leading slash ("/")
export PN_MODULE_ID


# PN_MODULE_NAME: Local module name (without hierarchy) ...
#   Name of the module (without path).
#   For hardware modules, it is required that the top-level module and the main
#   source file are named after PN_MODULE_NAME.
#   For software modules, it is recommended that the main source file, an eventual
#   library and an eventual main class are named after PN_MODULE_NAME.
PN_MODULE_NAME = $(notdir $(PN_MODULE_ID))
export PN_MODULE_NAME





#################### System dir and identification #############################


# PN_SYSTEM_DIR: System directory (optional) ...
#   This points to what is sometimes also called a "board support package" (BSP).
#   It is the root directory of a system project.
#~ $(info ### PN_SYSTEM = $(PN_SYSTEM))
#~ $(info ### PN_SYSTEM_DIR = $(PN_SYSTEM_DIR))
ifeq (,$(PN_SYSTEM_DIR))
  ifneq (,$(PN_SYSTEM))
    ifneq (,$(realpath $(PN_SYSTEM)/piconut-config.mk))
      PN_SYSTEM_DIR := $(PN_SYSTEM)
    else
      PN_SYSTEM_DIR := $(PN_SOURCE_DIR)/systems/$(PN_SYSTEM)
      ifeq (,$(realpath $(PN_SYSTEM_DIR)/piconut-config.mk))
        undefine PN_SYSTEM_DIR
        $(warning PN_SYSTEM="$(PN_SYSTEM)" points to a directory without a 'piconut-config.mk' file: ignoring system.)
      endif
    endif
    PN_SYSTEM_DIR := $(realpath $(PN_SYSTEM_DIR))
  else
    PN_SYSTEM_DIR :=
  endif
endif
export PN_SYSTEM_DIR


# PN_SYSTEM_ID: System ID ...
#   The identifier of the system context.
#   For a system contained in the PicoNut source tree, this is the relative
#   path to the system.
#   For an external sytem, PN_SYSTEM_ID is set to PN_SYSTEM_DIR with a
#   leading '/' removed, which is basically the absolute path of the system.
ifneq (,$(PN_SYSTEM_DIR))
  PN_SYSTEM_ID := $(PN_SYSTEM_DIR)
  PN_SYSTEM_ID := $(PN_SYSTEM_ID:$(PN_SOURCE_DIR)/%=%)
    # relative path inside PN_SOURCE_DIR (or PN_SYSTEM_DIR unmodified if outside)
  PN_SYSTEM_ID := $(PN_SYSTEM_ID:/%=%)
    # remove leading slash ("/")
else
  PN_SYSTEM_ID :=
endif
export PN_SYSTEM_ID


# PN_SYSTEM_NAME: Local system name (without hierarchy) ...
#   Name of the system (without path).
ifneq (,$(PN_SYSTEM_DIR))
  PN_SYSTEM_NAME := $(notdir $(PN_SYSTEM_ID))
else
  PN_SYSTEM_NAME :=
endif
export PN_SYSTEM_NAME





#################### Build tree and related variables ##########################


# PN_BUILD_DIR: Global build dir ...
#   Directory to which build artifacts are written.
$(shell mkdir -p $(PICONUT_BUILD))
PN_BUILD_DIR := $(realpath $(PICONUT_BUILD))
export PN_BUILD_DIR


# PN_SYSTEM_BUILD_DIR: Build tree for the current system (if defined) ...
ifneq (,$(PN_SYSTEM_DIR))
  PN_SYSTEM_BUILD_DIR := $(PN_BUILD_DIR)/$(PN_SYSTEM_ID)
else
  PN_SYSTEM_BUILD_DIR :=
endif
export PN_SYSTEM_BUILD_DIR


# PN_MODULE_BUILD_DIR: Build dir for the current module ...
#   This is the (only) place where build artefacts for the present module go.
ifeq (,$(PN_MODULE_ID))
  # Special case: Module is the root of the PicoNut source tree ...
  PN_MODULE_BUILD_DIR := $(PN_BUILD_DIR)
else
  ifeq (,$(PN_SYSTEM_ID))
    # Normal case: No system context ...
    PN_MODULE_BUILD_DIR := $(PN_BUILD_DIR)/$(PN_MODULE_ID)
  else
    # With system context: Use a subdirectory for the system ...
    #   The system itself is built into the root of this subdirectory.
    #   Prebuilt modules are built into the 'piconut/' folder of the system subdirectory.
    #   Hence, `make clean` in the system root cleans
    ifeq ($(PN_MODULE_SOURCE_DIR),$(PN_SYSTEM_DIR))
      # System root ...
      PN_MODULE_BUILD_DIR := $(PN_SYSTEM_BUILD_DIR)
    else
      RELPATH := $(PN_MODULE_SOURCE_DIR:$(PN_SYSTEM_DIR)/%=%)
      ifneq ($(RELPATH),$(PN_MODULE_SOURCE_DIR))
        # Submodule of the system ...
        PN_MODULE_BUILD_DIR := $(PN_SYSTEM_BUILD_DIR)/$(RELPATH)
      else
        # PicoNut module ...
        PN_MODULE_BUILD_DIR := $(PN_BUILD_DIR)/$(PN_SYSTEM_ID)/piconut/$(PN_MODULE_ID)
      endif
    endif
  endif
endif
export PN_MODULE_BUILD_DIR





#################### Stage tree and related variables ##########################


# PN_STAGE_DIR: Installation-like tree for implicit installations ...
#   This directory is required for multi-module/recursive builds where outputs
#   of (sub)modules are used by other modules. In a global context, the stage
#   directory resides inside PN_BUILD_DIR. In a system context, this is inside
#   the system.
ifneq (,$(PN_SYSTEM_DIR))
  PN_STAGE_DIR := $(PN_SYSTEM_BUILD_DIR)/pub
#~   PN_STAGE_DIR := $(PN_SYSTEM_DIR)/piconut
else
  PN_STAGE_DIR := $(PN_BUILD_DIR)/pub
endif
export PN_STAGE_DIR





#################### Tools #####################################################


# PN_TOOLS_DIR & friends: Tools ...
ifneq (,$(wildcard $(PN_STAGE_DIR)/tools))
  PN_TOOLS_DIR := $(PN_STAGE_DIR)/tools
else
  ifneq (,$(wildcard $(PN_INSTALLED_DIR)/tools))
    PN_TOOLS_DIR := $(PN_INSTALLED_DIR)/tools
  else
    ifneq (,$(wildcard $(PN_SOURCE_DIR)/tools))
      PN_TOOLS_DIR := $(PN_SOURCE_DIR)/tools
    endif
  endif
endif
ifeq (,$(PN_TOOLS_DIR))
  $(error "Cannot find the PicoNut tools!")
endif
export PN_TOOLS_DIR
export PN_TOOLS_BIN := $(PN_TOOLS_DIR)/bin
export PN_TOOLS_ETC := $(PN_TOOLS_DIR)/etc





#################### Boards ####################################################


# PN_BOARDS_DIR: Board definitions ...
ifneq (,$(wildcard $(PN_SOURCE_DIR)/boards))
  PN_BOARDS_DIR := $(PN_SOURCE_DIR)/boards
else
  ifneq (,$(PN_INSTALLED_DIR))
    ifneq (,$(wildcard $(PN_INSTALLED_DIR)/boards))
      PN_BOARDS_DIR := $(PN_INSTALLED_DIR)/boards
    else
      $(warning PN_INSTALLED_DIR=$(PN_INSTALLED_DIR) does not contain board definitions.)
    endif
  endif
endif
export PN_BOARDS_DIR





################################################################################
#                                                                              #
#   Include Global and System-Specific Configuration                           #
#                                                                              #
################################################################################


# PicoNut version and build date ...
PN_BUILD_VERSION ?= $(shell git describe --tags --match "v[0-9]*\.[0-9]*" --long --dirty='*' --abbrev=4 --always 2>/dev/null || echo "v0.0-0")
PN_BUILD_DATE ?= $(shell date +%Y-%m-%d)


# Global settings ...
PN_CONFIG_SOURCES := $(PN_SOURCE_DIR)/piconut-config.mk


# System-specific settings (if available) ...
ifneq (,$(PN_SYSTEM_DIR))
  PN_CONFIG_SOURCES := $(PN_SYSTEM_DIR)/piconut-config.mk $(PN_CONFIG_SOURCES)
endif


# Include them ...
include $(PN_CONFIG_SOURCES)


# Console output when building ...
ifneq (0,$(VERBOSE))
  PN_BUILD_PREFIX :=
  PN_BUILD_SUBMAKE_FLAGS :=
else
  PN_BUILD_PREFIX := "[$(PN_MODULE_ID)] "
  PN_BUILD_SUBMAKE_FLAGS := -s
  #PN_BUILD_SUBMAKE_FLAGS := --no-print-directory
endif


# PN_BOARD_DIR: Selected board ...
ifneq (,$(PICONUT_BOARD))
  PN_CFG_BOARD := $(PICONUT_BOARD)
endif
PN_BOARD_DIR := $(PN_BOARDS_DIR)/$(PN_CFG_BOARD)
export PN_BOARD_DIR





################################################################################
#                                                                              #
#   Hardware Simulation (SystemC)                                              #
#                                                                              #
################################################################################





######################### Tools ################################################


# Tools for simulation ...
export PN_HW_CC      := gcc
export PN_HW_CXX     := g++
export PN_HW_AS      := as
export PN_HW_CPP     := cpp
export PN_HW_LD      := g++
export PN_HW_AR      := ar
export PN_HW_OBJDUMP := objdump
export PN_HW_GDB     := gdb





######################### Default flags ########################################

# Note: Headers and libraries are and must be searched in the following order:
#   1. Build tree (headers) to allow building without an installation directory present.
#      NOTE: The build tree must have been prepared by building and "installing"
#            the 'sw/common' and 'hw/common' modules.
#   2. System project directory, PN_SYSTEM_DIR (if a system is defined)
#   3. Global installation directory, PN_INSTALLED_DIR (if available)


# PN_HW_CFLAGS ...
#   These are all standard options to compile PicoNut-compliant SystemC code for simulation, including:
#   - SystemC library selection (default / ICSC / Accelera)
#   - debug options (depending on DEBUG)
#   - search path(s) for PN_SYSTEM_DIR and PN_INSTALLED_DIR
#   It does not include search paths to other modules inside the PicoNut source
#   tree. If there are such dependencies, they must be documented, and PN_HW_CFLAGS
#   be expanded accordingly in the main makefile.

# ... basic & debug flags ...
PN_HW_CFLAGS := -MMD
ifeq ($(DEBUG),0)
  PN_HW_CFLAGS += -O3
else
  PN_HW_CFLAGS += -O0 -g
endif

# ... SystemC library selection ...
ifeq ($(PN_SYSTEMC),icsc)
  ifeq (,$(ICSC_HOME))
    $(error "'PN_SYSTEMC=icsc' requires ICSC_HOME to be set (is unset).")
  endif
  PN_HW_CFLAGS += -std=gnu++17 -I$(SYSTEMC_HOME)/include
    # [2025-08-09] '-std=gnu++17' seems to be required for the current version of ICSC

else
ifeq ($(PN_SYSTEMC),accelera)
  ifeq (,$(SYSTEMC_HOME))
    $(error "'PN_SYSTEMC=accelera' requires SYSTEMC_HOME to be set (is unset).")
  endif
  PN_HW_CFLAGS += -I$(SYSTEMC_HOME)/include

endif
endif

# ... search path ...
PN_HW_CFLAGS += -I$(PN_STAGE_DIR)/hw/include
ifneq (,$(PN_SYSTEM_DIR))
  PN_HW_CFLAGS += -I$(PN_SYSTEM_DIR)/hw/include
endif
ifneq (,$(PN_INSTALLED_DIR))
  PN_HW_CFLAGS += -I$(PN_INSTALLED_DIR)/hw/include
endif


# PN_HW_LDFLAGS ...
#   These are all standard options to link PicoNut-compliant SystemC code for
#   simulation, including:
#   - SystemC library (selectable: default / ICSC / Accelera)
#   - search path(s) for PN_SYSTEM_DIR and PN_INSTALLED_DIR
#   - library 'pn_common'
#   If there are dependencies on other modules, they must be documented, and
#   PN_HW_LDFLAGS be expanded accordingly in the main makefile.

# Search path for SystemC ...
#   (sanitizing ICSC_HOME or SYSTEMC_HOME is done in the PN_HW_CFLAGS section)
PN_HW_LDFLAGS :=
ifeq ($(PN_SYSTEMC),icsc)
  PN_HW_LDFLAGS += -L$(ICSC_HOME)/lib
else
ifeq ($(PN_SYSTEMC),accelera)
  PN_HW_LDFLAGS += -L$(SYSTEMC_HOME)/lib-linux
endif
endif

# ... search path for PicoNut modules ...
PN_HW_LDFLAGS += -L$(PN_STAGE_DIR)/hw/lib
ifneq (,$(PN_SYSTEM_DIR))
  PN_HW_LDFLAGS += -L$(PN_SYSTEM_DIR)/hw/lib
endif
ifneq (,$(PN_INSTALLED_DIR))
  PN_HW_LDFLAGS += -L$(PN_INSTALLED_DIR)/hw/lib
endif

# ... all libs ...
PN_HW_LDFLAGS += -lpn_common -lsystemc
#~ PN_HW_LDFLAGS := $(PN_SOURCE_DIR)/hw/common/base.cpp -lsystemc
	# TBD: Eliminate '$(PN_SOURCE_DIR)/hw/common/base.cpp'





######################### Predefined Rules #####################################


# Activate these rules only if hardware is or may be built (PN_BUILD_HW=1) ...
ifeq (1,$(PN_BUILD_SW))


# Compiling C++ files: C++ (not C) files with a target directory containing "/sim/"
# are assumed to be hardware (SystemC) ...
$(PN_MODULE_BUILD_DIR)/sim/%.o: %.cpp
ifneq (0,$(VERBOSE))
	@mkdir -p $(dir $@)
	$(PN_HW_CXX) -c -o $@ $(PN_HW_CFLAGS) $<
else
	@echo $(PN_BUILD_PREFIX)HW-CXX $<; \
	mkdir -p $(dir $@) && \
	$(PN_HW_CXX) -c -o $@ $(PN_HW_CFLAGS) $<
endif


# Linking simulator (or testbench) ...
#   Files ending with "_tb" or "_sim" are assumed to be executables for the host.
$(PN_MODULE_BUILD_DIR)/sim/%_tb:
	@mkdir -p $(dir $@) && if test "$^" = ""; then echo "$(@:$(PN_BUILD_DIR)/%=%): Missing object files in makefile rule!"; exit 3; fi
ifneq (0,$(VERBOSE))
	$(PN_HW_LD) -o $@ $^ $(PN_HW_LDFLAGS)
else
	@echo $(PN_BUILD_PREFIX)HW-LD $(@:$(PN_MODULE_BUILD_DIR)/%=%); \
	$(PN_HW_LD) -o $@ $^ $(PN_HW_LDFLAGS)
endif

$(PN_MODULE_BUILD_DIR)/sim/%_sim:
	@mkdir -p $(dir $@) && if test "$^" = ""; then echo "$(@:$(PN_BUILD_DIR)/%=%): Missing object files in makefile rule!"; exit 3; fi
ifneq (0,$(VERBOSE))
	$(PN_HW_LD) -o $@ $^ $(PN_HW_LDFLAGS)
else
	@echo $(PN_BUILD_PREFIX)HW-LD $(@:$(PN_MODULE_BUILD_DIR)/%=%); \
	$(PN_HW_LD) -o $@ $^ $(PN_HW_LDFLAGS)
endif


# Building a simulation archive ...
#   The target allows to add object files and existing archives, which are then merged.
#   To support archives, $(PN_HW_AR) is run in script mode.
$(PN_MODULE_BUILD_DIR)/sim/%.a:
	@mkdir -p $(dir $@) && if test "$^" = ""; then echo "$(@:$(PN_BUILD_DIR)/%=%): Missing object files in makefile rule!"; exit 3; fi;
	@echo $(PN_BUILD_PREFIX)HW-AR $(@:$(PN_MODULE_BUILD_DIR)/%=%); \
	SCRIPT="create $@\n" && \
	for OBJ in $(filter %.o,$^); do SCRIPT="$${SCRIPT}addmod $${OBJ}\n"; done && \
	for LIB in $(filter %.a,$^); do SCRIPT="$${SCRIPT}addlib $${LIB}\n"; done && \
	SCRIPT="$${SCRIPT}save\nend" && \
	if [ 0$(VERBOSE) -gt 0 ]; then echo $${SCRIPT} | sed 's#^#($(PN_HW_AR)) #'; fi && \
	echo "$$SCRIPT" | $(PN_HW_AR) -M


# End of section for PN_BUILD_HW=1 ...
endif





######################### Running Testbenches ##################################


# Macro to simplify the execution of a testbench inside a 'verify-*' target recipe.
# Unless in verbose mode, it runs the provided command and prints its output
# only in case of an error, i.e. if the exit code is non-zero.


ifneq (0,$(VERBOSE))
  PN_HW_RUN :=
else
  PN_HW_RUN := @f() { cmd="$$@"; echo "$(PN_BUILD_PREFIX)HW-RUN $$cmd"; out=`$$cmd 2>&1` || (echo "$$out" && exit 1) }; f
endif





################################################################################
#                                                                              #
#   Synthesis, Phase 1: SystemC (ICSC)                                         #
#                                                                              #
################################################################################


# Search path for PicoNut modules ...
PN_HW_SYN_PATH += $(PN_STAGE_DIR)/hw/syn
ifneq (,$(PN_SYSTEM_DIR))
  PN_HW_SYN_PATH += $(PN_SYSTEM_DIR)/hw/syn
endif
ifneq (,$(PN_INSTALLED_DIR))
  PN_HW_SYN_PATH += $(PN_INSTALLED_DIR)/hw/syn
endif


# ICSC: Helper variables ...
PN_ICSC_DIR = $(PN_MODULE_BUILD_DIR)/syn/icsc
PN_ICSC_DESIGN = $(PN_MODULE_NAME)
ifneq (,$(wildcard $(PN_MODULE_BUILD_DIR)/syn/icsc.sources))
  PN_ICSC_SOURCES := $(shell cat $(PN_MODULE_BUILD_DIR)/syn/icsc.sources)
endif


# ICSC: Exported variables ...
PN_SYSTEMC_OUTPUT := $(PN_MODULE_BUILD_DIR)/syn/$(PN_MODULE_NAME).v
PN_SYSTEMC_SV := $(PN_MODULE_BUILD_DIR)/syn/$(PN_MODULE_NAME).sv


# Collect exported source files ...
.PHONY: icsc-collect-sources
icsc-collect-sources:
ifndef PN_ICSC_DONT_PREPARE
	@for X in `$(MAKE) -s -C $(PN_SOURCE_DIR)/hw/common icsc-sources`; do echo "$(PN_SOURCE_DIR)/hw/common/$$X"; done
endif
	@for MOD in $(PN_SYSTEMC_SUBMODULES); do \
	  LIST=`$(MAKE) -s -C $${MOD} PN_ICSC_DONT_PREPARE=1 icsc-collect-sources`; \
	  for X in $$LIST; do echo "$$MOD/$$X"; done; \
	done && \
	for X in `$(MAKE) -s icsc-sources`; do echo "$$X"; done


# Sources file ...
#   'icsc.sources' is used to correctly trigger rebuilds whenever any source file
#   changes. The variable PN_ICSC_SOURCES contains the list of sources if 'icsc.sources'
#   existed in the beginning. However, it may be undefined or outdated. Hence, any
#   target requiring the sources must read 'icsc.sources', *not* PN_ISCS_SOURCES.
#   However, by having $(PN_ICSC_DIR)/Makefile depend on PN_ICSC_SOURCES, changes in
#   the source files will trigger a resynthesis.
#   However, if source files are added, `make clean` is required and recommended.
$(PN_MODULE_BUILD_DIR)/syn/icsc.sources: $(PN_ICSC_SOURCES)
	@echo $(PN_BUILD_PREFIX)HW-ICSC-MK $(notdir $@); \
	mkdir -p $(PN_MODULE_BUILD_DIR)/syn && \
	for X in `$(MAKE) -s icsc-collect-sources` $(PN_SYSTEMC_SRC); do \
	  echo `realpath $$X`; \
	done > $@



# ICSC step 1a: Collect all sources and create a Makefile ...
$(PN_ICSC_DIR)/Makefile: $(PN_ICSC_SOURCES) $(PN_MODULE_BUILD_DIR)/syn/icsc.sources $(PN_HW_CONFIG_H)
	@echo $(PN_BUILD_PREFIX)HW-ICSC-PREP $(PN_ICSC_DESIGN); \
	SOURCES=`cat $(PN_MODULE_BUILD_DIR)/syn/icsc.sources` && \
	mkdir -p $(PN_ICSC_DIR) && \
	echo "cmake_minimum_required(VERSION 3.18)"                 >  $(PN_ICSC_DIR)/CMakeLists.txt && \
	echo "set_property(GLOBAL PROPERTY RULE_MESSAGES OFF)"      >> $(PN_ICSC_DIR)/CMakeLists.txt && \
	echo                                                        >> $(PN_ICSC_DIR)/CMakeLists.txt && \
	echo "project($(PN_ICSC_DESIGN))"                           >> $(PN_ICSC_DIR)/CMakeLists.txt && \
	echo "find_package(SVC REQUIRED)"                           >> $(PN_ICSC_DIR)/CMakeLists.txt && \
	echo                                                        >> $(PN_ICSC_DIR)/CMakeLists.txt && \
	echo "set(PROC_TECH fpga)"                                  >> $(PN_ICSC_DIR)/CMakeLists.txt && \
	echo "set(CMAKE_CXX_STANDARD 17)"                           >> $(PN_ICSC_DIR)/CMakeLists.txt && \
	echo "set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")" >> $(PN_ICSC_DIR)/CMakeLists.txt && \
	echo                                                        >> $(PN_ICSC_DIR)/CMakeLists.txt && \
	echo "enable_testing()"                                     >> $(PN_ICSC_DIR)/CMakeLists.txt && \
	echo                                                        >> $(PN_ICSC_DIR)/CMakeLists.txt && \
	echo "add_compile_options(-Wall -ggdb -O0)"                 >> $(PN_ICSC_DIR)/CMakeLists.txt && \
	echo "add_executable($(PN_ICSC_DESIGN) $$SOURCES)"          >> $(PN_ICSC_DIR)/CMakeLists.txt && \
	echo "target_include_directories($(PN_ICSC_DESIGN) PUBLIC "$(PN_STAGE_DIR)/hw/include")"        >> $(PN_ICSC_DIR)/CMakeLists.txt && \
	if [ -e "$(PN_SYSTEM_DIR)/hw/include" ]; then \
	  echo "target_include_directories($(PN_ICSC_DESIGN) PUBLIC "$(PN_SYSTEM_DIR)/hw/include")"     >> $(PN_ICSC_DIR)/CMakeLists.txt; \
	fi && \
	if [ -e "$(PN_INSTALLED_DIR)/hw/include" ]; then \
	  echo "target_include_directories($(PN_ICSC_DESIGN) PUBLIC "$(PN_INSTALLED_DIR)/hw/include")"  >> $(PN_ICSC_DIR)/CMakeLists.txt; \
	fi && \
	DEFS="-D__SYNTHESIS__=1 -DPN_TOPLEVEL_IS_`echo $(PN_MODULE_NAME)=1 | tr '[:lower:]' '[:upper:]'`"; \
	echo "target_compile_definitions($(PN_ICSC_DESIGN) PUBLIC $$DEFS)"                              >> $(PN_ICSC_DIR)/CMakeLists.txt && \
	echo                                            >> $(PN_ICSC_DIR)/CMakeLists.txt && \
	echo "svc_target($(PN_ICSC_DESIGN) ELAB_TOP "i_dut")"       >> $(PN_ICSC_DIR)/CMakeLists.txt
ifneq (0,$(VERBOSE))
	@( bash -c ". $(ICSC_HOME)/setenv.sh && cd $(PN_ICSC_DIR) && cmake ." )
else
	@( bash -c ". $(ICSC_HOME)/setenv.sh && cd $(PN_ICSC_DIR) && cmake ." ) | ( grep -v '^--' || true )
endif


.PHONY: icsc-prep
icsc-prep: $(PN_ICSC_DIR)/Makefile


# ICSC step 1b: Compile the C++/SystemC code ...
#   The sub-make does not call a PicoNut module, so no dependency on 'build-prepare' is required.
$(PN_ICSC_DIR)/$(PN_ICSC_DESIGN)_sctool: $(PN_ICSC_DIR)/Makefile $(PN_ICSC_SOURCES) $(PN_HW_CONFIG_H)
ifneq (0,$(VERBOSE))
	$(MAKE) -C $(PN_ICSC_DIR) $(PN_ICSC_DESIGN)_sctool > /dev/null || $(MAKE) -C $(PN_ICSC_DIR) clean
else
	@echo $(PN_BUILD_PREFIX)HW-ICSC-MAKE $(PN_ICSC_DESIGN); \
	$(MAKE) -C $(PN_ICSC_DIR) $(PN_ICSC_DESIGN)_sctool > /dev/null || $(MAKE) -C $(PN_ICSC_DIR) clean
endif

.PHONY: icsc-comp
icsc-comp: $(PN_ICSC_DIR)/$(PN_ICSC_DESIGN)_sctool


# ICSC step 1c: Run synthesis to get a SystemVerilog file ...
#  SystemVerilog allows to include external Verilog files, e.g. for blackbox submodules.
#  Hence, we use this format to store partial designs.
$(PN_SYSTEMC_SV): $(PN_ICSC_DIR)/$(PN_ICSC_DESIGN)_sctool
ifneq (0,$(VERBOSE))
	@( bash -c ". $(ICSC_HOME)/setenv.sh && cd $(PN_ICSC_DIR) && \
	  ./$(PN_ICSC_DESIGN)_sctool | tee $(PN_MODULE_BUILD_DIR)/syn/$(PN_ICSC_DESIGN).icsc.log 2>&1 && \
	  mv sv_out/$(PN_ICSC_DESIGN).sv $(PN_MODULE_BUILD_DIR)/syn" )
else
	@echo $(PN_BUILD_PREFIX)HW-ICSC-SYN $(PN_ICSC_DESIGN); \
	( bash -c ". $(ICSC_HOME)/setenv.sh && cd $(PN_ICSC_DIR) && \
	  ./$(PN_ICSC_DESIGN)_sctool > $(PN_MODULE_BUILD_DIR)/syn/$(PN_ICSC_DESIGN).icsc.log 2>&1 && \
	  mv sv_out/$(PN_ICSC_DESIGN).sv $(PN_MODULE_BUILD_DIR)/syn" ) \
	|| ( cat $(PN_MODULE_BUILD_DIR)/syn/$(PN_ICSC_DESIGN).icsc.log && exit 1 )
endif

.PHONY: icsc-syn
icsc-syn: $(PN_SYSTEMC_SV)


# Convert SystemVerilog to Verilog (generic) ...
$(PN_MODULE_BUILD_DIR)/syn/%.v: $(PN_MODULE_BUILD_DIR)/syn/%.sv
ifneq (0,$(VERBOSE))
	sv2v -I$(PN_MODULE_SOURCE_DIR) $(PN_HW_SYN_PATH:%=-I%) $(PN_HW_SYN_PATH:%=-y%) $< > $@ \
	|| (rm -f $@; exit 1)
else
	@echo $(PN_BUILD_PREFIX)HW-SV2V $(notdir $@) && \
	sv2v -I$(PN_MODULE_SOURCE_DIR) $(PN_HW_SYN_PATH:%=-I%) $(PN_HW_SYN_PATH:%=-y%) $< > $@ \
	|| (rm -f $@; exit 1)
endif


# Phony target for easy access ...
.PHONY: build-systemc
build-systemc: $(PN_SYSTEMC_OUTPUT)





#################### Rules to be defined by user ###############################


# Empty 'icsc-sources' rule for (sub)modules without ICSC source files...
.PHONY: icsc-sources
icsc-sources:





################################################################################
#                                                                              #
#   Synthesis, Phases 2 & 3: RTL Synthesis, Place and Route, Boards            #
#                                                                              #
################################################################################


# Variables to be set by Makefile:
#
#   PN_SYSTEMC_SRC : SystemC source files
#
#   PN_NETLIST_SRC : RTL source files (arbitrary combination of
#                    - Verilog (.v)
#                    - VHDL (.vhdl)
#                    - tool-specific snapshot (Yosys: .json)
#                    files.
#                    Entries with a relative path are assumed to be external and
#                    searched in (in that order):
#                    PN_MODULE_DIR, PN_STAGE_DIR, PN_SYSTEM_DIR, PN_INSTALLED_DIR
#
#   PN_LAYOUT_SRC  : Presynthesized modules for layout generation
#
#   PN_TOPLEVEL    : Entity name of the top-level module, preset according to
#                    the module name prefixed with "m_";
#                    can, but should not be changed


# Variables preset by 'piconut.mk' (names are fixed and cannot be changed):
#
#   PN_SYSTEMC_OUTPUT  : Output of the SystemC synthesis (Verilog)
#   PN_NETLIST_OUTPUT  : Output of the gate-level synthesis (tool-specific)
#   PN_LAYOUT_OUTPUT   : Output of Place & Route (typically bitfile)


PN_TOPLEVEL := m_$(PN_MODULE_NAME)





#################### Presets for board definitions #############################


# Yosys script commands to read all source files ...
$(PN_MODULE_BUILD_DIR)/syn/read_sources.yosys:
	@rm -fr $@ && mkdir -p $(dir $@) && touch $@
	@for SRC in $(PN_NETLIST_SRC); do \
	  PN=$$SRC; \
	  if [ "$${PN%%/*}" != "" ]; then \
	    test -f $$PN || PN=$(PN_MODULE_BUILD_DIR)/syn/$$SRC; \
	    test -f $$PN || PN=$(PN_STAGE_DIR)/hw/syn/$$SRC; \
	    test -f $$PN || PN=$(PN_SYSTEM_DIR)/piconut/hw/syn/$$SRC; \
	    test -f $$PN || PN=$(PN_INSTALLED_DIR)/hw/syn/$$SRC; \
	    if test ! -f $$PN; then echo "PN_NETLIST_SRC: File not found: $$SRC"; rm $@; exit 3; fi; \
	  fi; \
	  if [ "$${PN##*.}" = "json" ]; then \
	    echo "read_json $$PN" >> $@; \
	  elif [ "$${PN##*.}" = "v" ]; then \
	    echo "read_verilog $$PN" >> $@; \
	  elif [ "$${PN##*.}" = "vhdl" ]; then \
	    echo "ghdl $$PN -e" >> $@; \
	  fi \
	done || (rm $@; exit 1)





#################### Include board definitions #################################


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


# Base file names for outputs ...
PN_NETLIST_OUTPUT_BASE := $(PN_MODULE_BUILD_DIR)/syn/$(PN_MODULE_NAME).$(PN_CFG_BOARD)
PN_LAYOUT_OUTPUT_BASE := $(PN_MODULE_BUILD_DIR)/syn/$(PN_MODULE_NAME).$(PN_CFG_BOARD)


# If BITFILE is passed (typically on the command line), set PN_LAYOUT_OUTPUT ...
#   This is typically used together with the 'program' target.
ifneq (,$(BITFILE))
  PN_LAYOUT_OUTPUT := $(BITFILE)
endif


# Defaults for optional board settings ...
PN_NETLIST_REPORT := true
PN_LAYOUT_REPORT := true


# Include the selected board definition ...
include $(PN_BOARD_DIR)/piconut-board.mk





#################### Rules and Targets #########################################


# Alias targets to collect any additional dependencies for synthesis stages ...
#   Since various module-specific source files are set after this section
#   was parsed, their dependencies cannot be set here, but must be specified
#   in the end of the Makefile.
#
#   In the general case, make files must contain the following lines AFTER
#   the respective variables have been set (unused variables can be skipped):
#
#     $(PN_NETLIST_OUTPUT): $(PN_NETLIST_SRC)
#     $(PN_LAYOUT_OUTPUT): $(PN_LAYOUT_SRC)
#


# Netlist generation (RTL synthesis), board-specific ...
$(PN_NETLIST_OUTPUT): $(PN_HW_CONFIG_H)
ifneq (0,$(VERBOSE))
	$(PN_NETLIST_CMD)
else
	@echo $(PN_BUILD_PREFIX)HW-NETLIST $(notdir $@); \
	( $(PN_NETLIST_CMD) ) > $@.log 2>&1 || (cat $@.log && exit 1)
endif
	@( $(PN_NETLIST_REPORT) ) | sed 's#^#[$(PN_MODULE_ID)] HW-NETLIST $(notdir $@): #'


.PHONY: build-netlist
build-netlist: $(PN_NETLIST_OUTPUT)


# Layout generation (e.g. FPGA bitfile), board-specific and system-specific ...
$(PN_LAYOUT_OUTPUT): $(PN_NETLIST_OUTPUT) $(PN_BOARD_CONSTRAINTS)
ifneq (0,$(VERBOSE))
	$(PN_LAYOUT_CMD)
else
	@echo $(PN_BUILD_PREFIX)HW-LAYOUT $(notdir $@); \
	( $(PN_LAYOUT_CMD) ) > $@.log 2>&1 || (cat $@.log && exit 1)
endif
	@( $(PN_LAYOUT_REPORT) ) | sed 's#^#[$(PN_MODULE_ID)] HW-LAYOUT $(notdir $@): #'


.PHONY: build-layout
build-layout: $(PN_LAYOUT_OUTPUT)


# FPGA programming ...
.PHONY: program
program: build-prepare .WAIT stage-submodules .WAIT $(PN_LAYOUT_OUTPUT)
ifneq (0,$(VERBOSE))
	$(PN_PROGRAM_CMD)
else
	@echo $(PN_BUILD_PREFIX)HW-BIT $(notdir $@); \
	( $(PN_PROGRAM_CMD) ) > $<.program.log 2>&1 || (cat $<.program.log && exit 1)
endif


# Automatic dependencies ...
$(PN_LAYOUT_OUTPUT): $(PN_LAYOUT_SRC)





################################################################################
#                                                                              #
#   Building Software (RISC-V)                                                 #
#                                                                              #
################################################################################





######################### Tools ################################################


# Prefix for the RISCV toolchain ...
#   If RISCV is defined, the tools are take from there. If not, the RISC-V tools
#   must be discoverable by the PATH variable.
PN_SW_CROSS := riscv64-unknown-elf
ifneq (,$(RISCV))
  PN_SW_CROSS := $(RISCV)/bin/$(PN_SW_CROSS)
endif
export PN_SW_CROSS


# Tools (default invocations) ...
export PN_SW_CC      := $(PN_SW_CROSS)-gcc
export PN_SW_CXX     := $(PN_SW_CROSS)-g++
export PN_SW_AS      := $(PN_SW_CROSS)-as
export PN_SW_CPP     := $(PN_SW_CROSS)-cpp
export PN_SW_LD      := $(PN_SW_CROSS)-gcc
export PN_SW_AR      := $(PN_SW_CROSS)-ar
export PN_SW_OBJDUMP := $(PN_SW_CROSS)-objdump
export PN_SW_OBJCOPY := $(PN_SW_CROSS)-objcopy
export PN_SW_SIZE    := $(PN_SW_CROSS)-size
export PN_SW_GDB     := $(PN_SW_CROSS)-gdb





######################### Default flags ########################################

# Note: Headers and libraries are and must be searched in the following order:
#   1. Build tree (headers) to allow building without an installation directory present.
#      NOTE: The build tree must have been prepared by building and "installing"
#            the 'sw/common' and 'hw/common' modules.
#   2. System project directory, PN_SYSTEM_DIR (if a system is defined)
#   3. Global installation directory, PN_INSTALLED_DIR (if available)


# PN_SW_CFLAGS ...
#   These are all standard options to compile PicoNut-compliant RISC-V code, including:
#   - RISC-V architecture/ISA selection
#   - debug options (depending on DEBUG)
#   - search path(s) for PN_SYSTEM_DIR and PN_INSTALLED_DIR
#   It does not include search paths to other modules inside the PicoNut source
#   tree. If there are such dependencies, they must be documented, and PN_SW_CFLAGS
#   be expanded accordingly in the main makefile.

# ... GCC specs ...
ifneq (,$(RISCV_SPECS))
  PN_SW_CFLAGS += -specs=$(RISCV_SPECS)
endif

# ... architecture selection ...
PN_SW_CFLAGS += -march=$(PN_MARCH) -mabi=ilp32

# ... basic & debug flags ...
PN_SW_CFLAGS += -MMD
ifeq ($(DEBUG),0)
  PN_SW_CFLAGS += -O3
else
  PN_SW_CFLAGS += -O0 -g
endif

# ... search path ...
ifneq (,$(RISCV_LIBC))
  PN_SW_CFLAGS += -I$(RISCV_LIBC)/include
endif
PN_SW_CFLAGS += -I$(PN_STAGE_DIR)/sw/include
ifneq (,$(PN_SYSTEM_DIR))
  PN_SW_CFLAGS += -I$(PN_SYSTEM_DIR)/sw/include
endif
ifneq (,$(PN_INSTALLED_DIR))
  PN_SW_CFLAGS += -I$(PN_INSTALLED_DIR)/sw/include
endif

# PN_SW_ASFLAGS ...
#   Flags for assembly code files (.S) ...
PN_SW_ASFLAGS := -g -march=$(PN_MARCH) -mabi=ilp32

# PN_SW_LDFLAGS ...
#   These are all standard options to link PicoNut-compliant RISC-V code, including:
#   - RISC-V architecture/ISA selection
#   - OS-related options, depending on PN_SW_OS
#   - search path(s) for PN_SYSTEM_DIR and PN_INSTALLED_DIR
#   It does not include search paths to other modules inside the PicoNut source
#   tree. If there are such dependencies, they must be documented, and PN_SW_LDFLAGS
#   be expanded accordingly in the main makefile.

# ... GCC specs ...
ifneq (,$(RISCV_SPECS))
  PN_SW_LDFLAGS += -specs=$(RISCV_SPECS)
endif

# ... architecture flags ...
PN_SW_LDFLAGS += -march=$(PN_MARCH) -mabi=ilp32

# ... search path ...
ifneq (,$(RISCV_LIBC))
  PN_SW_CFLAGS += -I$(RISCV_LIBC)/lib
endif
PN_SW_LDFLAGS += -L$(PN_STAGE_DIR)/sw/lib
ifneq (,$(PN_SYSTEM_DIR))
  PN_SW_LDFLAGS += -L$(PN_SYSTEM_DIR)/sw/lib
endif
ifneq (,$(PN_INSTALLED_DIR))
  PN_SW_LDFLAGS += -L$(PN_INSTALLED_DIR)/sw/lib
endif

# ... OS selection ...
ifeq ($(PN_SW_OS),std)
  # no special options
else
ifeq ($(PN_SW_OS),bare)
  # bare metal system: use own startup files and 'piconut.ld'
  # TBD: Eliminate dependency from PN_SOURCE_DIR (let respective common code be installed by 'common' module)
  PN_SW_LDFLAGS += -static -nostartfiles
  PN_SW_LDFLAGS += -lc -lpiconut
  PN_SW_LDFLAGS += -T $(PN_SOURCE_DIR)/sw/common/piconut.ld
  PN_SW_LDFLAGS += \
		-Xlinker --defsym CFG_START_ADDRESS=$(PN_CFG_CPU_RESET_ADR) \
    -Xlinker --defsym CFG_CODE_SIZE=$(PN_CFG_SYS_CODE_SIZE) \
    -Xlinker --defsym CFG_RAM_SIZE=$(PN_CFG_SYS_RAM_SIZE) \
    -Xlinker --defsym CFG_STACK_SIZE=$(PN_CFG_SYS_STACK_SIZE) \
    -Xlinker --defsym CFG_HEAP_SIZE=$(PN_CFG_SYS_HEAP_SIZE)
else
  $(error Unknown OS environment: PN_SW_OS=$(PN_SW_OS))
endif
endif





######################### Predefined Rules ###################################


# Activate these rules only if software is or may be built (PN_BUILD_SW=1) ...
ifeq (1,$(PN_BUILD_SW))


# Compiling C files:
#   Note: C (not C++) files not contained in a "sim" or "syn" subdirectory
#   are automatically assumed to be software (RISC-V).
$(PN_MODULE_BUILD_DIR)/%.o: %.c
ifneq (0,$(VERBOSE))
	@mkdir -p $(dir $@)
	$(PN_SW_CC) -c -o $@ $(PN_SW_CFLAGS) $<
else
	@echo $(PN_BUILD_PREFIX)SW-CC $<; \
	mkdir -p $(dir $@) && \
	$(PN_SW_CC) -c -o $@ $(PN_SW_CFLAGS) $<
endif


# Compiling assembly files ...
#   Note: Assembly files are automatically assumed to be software (RISC-V).
#   These files are explicitly passed through the C preprocessor (CPP), since
#   CPP allows to share macros with C code.
$(PN_MODULE_BUILD_DIR)/%.o: %.S
ifneq (0,$(VERBOSE))
	@mkdir -p $(dir $@)
	$(PN_SW_CPP) $< | $(PN_SW_AS) $(PN_SW_ASFLAGS) -o $@ -
else
	@mkdir -p $(dir $@); \
	echo $(PN_BUILD_PREFIX)SW-AS $<; \
	$(PN_SW_CPP) $< | $(PN_SW_AS) $(PN_SW_ASFLAGS) -o $@ -
endif

# [2025-05-08]
#   The following does not work either. Despite the 'v' in '-march=rv32iv',
#   no vector instructions are accepted:
#
#     $(RV_CC) -march=rv32iv $(RV_ASFLAGS) -o $@ $<


# Linking software (RISC-V) ...
#   Note: RISC-V binaries have the suffix ".rv32" (or .rv64) for legibility.
$(PN_MODULE_BUILD_DIR)/%.rv32:
	@mkdir -p $(dir $@) && if test "$^" = ""; then echo "$(@:$(PN_BUILD_DIR)/%=%): Missing object files in makefile rule!"; exit 3; fi
ifneq (0,$(VERBOSE))
	$(PN_SW_LD) -o $@ $^ $(PN_SW_LDFLAGS)
else
	@echo $(PN_BUILD_PREFIX)SW-LD $(@:$(PN_MODULE_BUILD_DIR)/%=%); \
	$(PN_SW_LD) -o $@ $^ $(PN_SW_LDFLAGS)
endif


# Building a library archive ...
#   The target allows to add object files and existing archives, which are then merged.
#   To support archives, $(PN_HW_AR) is run in script mode.
$(PN_MODULE_BUILD_DIR)/%.a:
	@mkdir -p $(dir $@) && if test "$^" = ""; then echo "$(@:$(PN_BUILD_DIR)/%=%): Missing object files in makefile rule!"; exit 3; fi;
	@echo $(PN_BUILD_PREFIX)HW-AR $(@:$(PN_MODULE_BUILD_DIR)/%=%); \
	SCRIPT="create $@\n" && \
	for OBJ in $(filter %.o,$^); do SCRIPT="$${SCRIPT}addmod $${OBJ}\n"; done && \
	for LIB in $(filter %.a,$^); do SCRIPT="$${SCRIPT}addlib $${LIB}\n"; done && \
	SCRIPT="$${SCRIPT}save\nend" && \
	if [ 0$(VERBOSE) -gt 0 ]; then echo $${SCRIPT} | sed 's#^#($(PN_SW_AR)) #'; fi && \
	echo "$$SCRIPT" | $(PN_SW_AR) -M


# End of section for PN_BUILD_SW=1 ...
endif





################################################################################
#                                                                              #
#   Installation Macros                                                        #
#                                                                              #
################################################################################


# "install" targets must use the following macros to write to the installation target.


# Internal command (not to be used directly) ...
#   NOTE: All PN_INSTALL_* macros below must have exactly 4 arguments!!
PN_INSTALL := install -vCDb
  # Option "-b" will create backups with the suffix '~' for existing files.
  # A check procedure can detect conflicts by searching for files ending with '~'.
ifeq (0,$(VERBOSE))
  PN_INSTALL := @f() { opts="$$1 $$2 $$3"; dst="$$4"; shift 4; $(PN_INSTALL) $$opts $$dst $$@ | sed -e "s!^.* '$(PN_BUILD_DIR)/!\[$(PN_MODULE_ID)\] STAGE !g" -e "s/.$$//g"; }; f
endif


# Installing hardware parts ...
PN_INSTALL_HW_INCLUDE := $(PN_INSTALL) -m 644 -t $(PN_STAGE_DIR)/hw/include
PN_INSTALL_HW_LIB := $(PN_INSTALL) -m 644 -t $(PN_STAGE_DIR)/hw/lib
PN_INSTALL_HW_BIN := $(PN_INSTALL) -m 644 -t $(PN_STAGE_DIR)/hw/bin
PN_INSTALL_HW_SYN := $(PN_INSTALL) -m 644 -t $(PN_STAGE_DIR)/hw/syn


# Installing software parts ...
PN_INSTALL_SW_INCLUDE := $(PN_INSTALL) -m 644 -t $(PN_STAGE_DIR)/sw/include
PN_INSTALL_SW_LIB := $(PN_INSTALL) -m 644 -t $(PN_STAGE_DIR)/sw/lib
PN_INSTALL_SW_BIN := $(PN_INSTALL) -m 644 -t $(PN_STAGE_DIR)/sw/bin
  # Software (RISC-V) binaries still have mode 644, since they are not executable
  # on the development machine.


# Installing PicoNut development tools ...
PN_INSTALL_TOOL_LIB := $(PN_INSTALL) -m 644 -t $(PN_STAGE_DIR)/lib
PN_INSTALL_TOOL_BIN := $(PN_INSTALL) -m 644 -t $(PN_STAGE_DIR)/bin


# Install a generic directory tree ...
#   This should be used very carefully and only if no other PN_INSTALL_* macros
#   are possible.
ifneq (0,$(VERBOSE))
  PN_INSTALL_TREE := f() { dst=`realpath -m $(PN_STAGE_DIR)/$$1`; src="$$2"; mkdir -p $${dst%/*} && cp -vau $$src $$dst; }; f
else
  PN_INSTALL_TREE := @f() { dst=`realpath -m $(PN_STAGE_DIR)/$$1`; src="$$2"; mkdir -p $${dst%/*} && cp -vau $$src $$dst | sed -e "s!^.* '$(PN_BUILD_DIR)/!\[$(PN_MODULE_ID)\] STAGE !g" -e "s/.$$//g"; }; f
#~   PN_INSTALL_TREE := @f() { dst="$(PN_STAGE_DIR)/$$1"; src="$$2"; echo $(PN_BUILD_PREFIX)STAGE $$dst "<-" $$src; rm -fr $$dst/$$src; mkdir -p $${dst%/*} && cp -r $$src $$dst; }; f
endif




######################### For system environments ##############################


# Prebuild command to be used in a 'build-prepare' recipe ...
#   NOTE: Use of the macro outside a 'build-prepare' recipe is not supported.
ifneq (0,$(VERBOSE))
	PN_PREBUILD := $(MAKE) $(PN_BUILD_SUBMAKE_FLAGS) -C $(PN_SOURCE_DIR) stage
else
	PN_PREBUILD := @$(MAKE) $(PN_BUILD_SUBMAKE_FLAGS) -C $(PN_SOURCE_DIR) stage
endif





################################################################################
#                                                                              #
#   Build preparation: Config, Common and dependency layers                    #
#                                                                              #
################################################################################





#################### Layer 0: Config ###########################################


# Configuration files for various languages and  ...
# TBD: Add PN_HW_CONFIG_VHDL and PN_HW_CONFIG_VERILOG with respective rules
#      for VHDL and Verilog modules.
PN_HW_CONFIG_H = $(PN_STAGE_DIR)/hw/include/piconut-config.h
PN_SW_CONFIG_H = $(PN_STAGE_DIR)/sw/include/piconut-config.h


# Generating C Headers (Hardware) ...
$(PN_HW_CONFIG_H): $(PN_CONFIG_SOURCES)
	@echo HW-MK $(PN_HW_CONFIG_H:$(PN_STAGE_DIR)/%=%)
	@mkdir -p $(PN_STAGE_DIR)/hw/include
	@echo "#pragma once\n" > $@
	@echo "\n/* Version identification ... */" >> $@
	@$(PN_SOURCE_DIR)/tools/bin/version2h.py $(PN_BUILD_VERSION) >> $@
	@echo "#define PN_BUILD_DATE \"$(PN_BUILD_DATE)\"\n" >> $@
	@echo "\n/* Configuration parameters ... */" >> $@
	@echo "$(foreach v, $(filter PN_CFG_%,$(.VARIABLES)),#define $(v) $(value $(v))\n)" \
	  | sed 's#^\s*##' >> $@
	@echo "\n/* Automatic preprocessor definitions ... */" >> $@
	@echo "#define PN_CFG_NUCLEUS_IS_"`echo $(PN_CFG_NUCLEUS) | tr '[:lower:]' '[:upper:]'` >> $@
	@echo "#define PN_CFG_MEMBRANA_IS_"`echo $(PN_CFG_MEMBRANA) | tr '[:lower:]' '[:upper:]'` >> $@


# Generating C Headers (Software, just copy of the hardware headers) ...
$(PN_SW_CONFIG_H): $(PN_HW_CONFIG_H)
	@echo SW-MK $(PN_SW_CONFIG_H:$(PN_STAGE_DIR)/%=%)
	@mkdir -p $(PN_STAGE_DIR)/sw/include
	@cp -a $(PN_HW_CONFIG_H) $(PN_SW_CONFIG_H)


# Phony to build & stage the config ...
.PHONY: config-stage
config-stage: $(PN_HW_CONFIG_H) $(PN_SW_CONFIG_H)


# Phony to install ...
.PHONY: config-install
config-install: config-stage
	$(PN_INSTALL_SW_INCLUDE) $(PN_SW_CONFIG_H)
	$(PN_INSTALL_HW_INCLUDE) $(PN_HW_CONFIG_H)


# Phony to clean config ...
.PHONY: config-clean
config-clean:
	@echo "CLEAN $(PN_HW_CONFIG_H) $(PN_SW_CONFIG_H)"
	@rm -f $(PN_HW_CONFIG_H) $(PN_SW_CONFIG_H)


# Phony to force a re-generation (needed if the PicoNut version changed) ...
.PHONY: config-update
config-update: config-clean .WAIT $(PN_HW_CONFIG_H) $(PN_SW_CONFIG_H)





#################### Layer 1: Common (+ Tools) #################################


# Build + stage 'common' (hardware) ...
#   The dependency on `config-stage` replaces the mandatory dependency on `build-prepare`.
.PHONY: common-hw-stage
common-hw-stage: config-stage
ifneq (0,$(VERBOSE))
	$(MAKE) $(PN_BUILD_SUBMAKE_FLAGS) -C $(PN_SOURCE_DIR)/hw/common stage
else
	@$(MAKE) $(PN_BUILD_SUBMAKE_FLAGS) -C $(PN_SOURCE_DIR)/hw/common stage
endif

# Build + stage 'common' (software) ...
#   The dependency on `config-stage` replaces the mandatory dependency on `build-prepare`.
.PHONY: common-sw-stage
common-sw-stage: config-stage
ifneq (0,$(VERBOSE))
	$(MAKE) $(PN_BUILD_SUBMAKE_FLAGS) -C $(PN_SOURCE_DIR)/sw/common stage
else
	@$(MAKE) $(PN_BUILD_SUBMAKE_FLAGS) -C $(PN_SOURCE_DIR)/sw/common stage
endif


# Build tools ...
#   The dependency on `config-stage` replaces the mandatory dependency on `build-prepare`.
.PHONY: tools-stage
tools-stage: config-stage
ifneq (0,$(VERBOSE))
	$(MAKE) $(PN_BUILD_SUBMAKE_FLAGS) -C $(PN_SOURCE_DIR)/tools stage
else
	@$(MAKE) $(PN_BUILD_SUBMAKE_FLAGS) -C $(PN_SOURCE_DIR)/tools stage
endif


# Build everything of Layer 1 (common + tools) ...
#   Tools are omitted in a system context.
.PHONY: common-stage
common-stage: common-hw-stage common-sw-stage
ifeq (,$(PN_SYSTEM_DIR))
common-stage: tools-stage
endif




#################### Build preparation #########################################


# This section defines some abstract targets to manage cross-layer dependencies.


# Important Note
# --------------
#
# When using recursive Make invocations, layers 0 and 1 are built only in the
# primary (root) invocation. This is to avoid multiple builds of these modules,
# which would a) waste CPU time and b) lead to serious file access conflicts
# during parallel builds. Resolving this by means of dependencies would also be
# prone to race conditions, depending on the file system in use. Hence, the
# "build once" condition is implemented by means of exported Make variables.
#
# The following rules must be fullfilled:
#
# 1. For modules with submodules: If any submodule depends on `prepare-config`
#    or `prepare-common`, the enclosing module must depend on these targets, too.
#    `prepare-common` implies `prepare-config`, so that `prepare-config` does not
#    need to be set if `prepare-common` is already present as a dependency.
#
#    If this is not fullfilled, the statement `PN_BUILD_INIT := 1` may be added
#    to the main module's Makefile before the `include .../piconut.mk` line.
#
# 2. Any rule that invokes a sub-make (`$(MAKE) ...`) must depend on
#    `build-prepare`!
#


# Build preparation target ...
#   This target is fullfilled before any local `build-*` target is invoked
#   (e.g `build-sim`, `build-syn` or `build-all`).
#   `build`, `install`, `verify` and similar targets depend on it.
#   While rarely needed, modules may define additional dependencies for
#   module-specific preparations.
#   By default, layers 0 and 1 (Config and Common) are built.
#
.PHONY: build-prepare
build-prepare:


# Target to ensure that config is built and staged ...
.PHONY: prepare-config
prepare-config:


# Target to ensure that common+tools is built and staged ...
.PHONY: prepare-common
prepare-common: prepare-config


# Auto-prepare common modules unless explicitly disabled ...
#   This is a very common case, so that we introduce PN_BUILD_COMMON with a default of 1.
ifeq (1,$(PN_BUILD_COMMON))
build-prepare: prepare-common
endif


# Automatically activate building config and common on the first/root invocation of make ...

#   Autodetect root of a source tree or root of a system ...
ifneq (,$(filter $(realpath $(PN_MODULE_SOURCE_DIR)), $(PN_SOURCE_DIR) $(PN_SYSTEM_DIR)))
  PN_BUILD_INIT ?= 1
endif

#   Reset "have" flags on init ...
ifeq (1,$(PN_BUILD_INIT))
  PN_HAVE_CONFIG := 0
  PN_HAVE_COMMON := 0
endif
export PN_BUILD_INIT := 0

#   Build config if required ...
ifneq (1,$(PN_HAVE_CONFIG))
prepare-config: config-stage
endif
export PN_HAVE_CONFIG := 1

#   Build common if required ...
ifneq (1,$(PN_HAVE_COMMON))
prepare-common: common-stage
endif
export PN_HAVE_COMMON := 1





################################################################################
#                                                                              #
#   Global Targets: build-*, verify-*, install-*, clean                        #
#                                                                              #
################################################################################





######################### Build ################################################


# Create the the module build dir (but only main dir, for convenience) ...
$(PN_MODULE_BUILD_DIR):
	@mkdir -p $@


# Build and stage submodules ...
#   Building a submodule implies staging it.
.PHONY: $(PN_SUBMODULES:%=stage-submodule-%)
$(PN_SUBMODULES:%=stage-submodule-%): build-prepare
ifneq (0,$(VERBOSE))
	$(MAKE) $(PN_BUILD_SUBMAKE_FLAGS) -C $(PN_MODULE_SOURCE_DIR)/$(@:stage-submodule-%=%) stage
else
	@$(MAKE) $(PN_BUILD_SUBMAKE_FLAGS) -C $(PN_MODULE_SOURCE_DIR)/$(@:stage-submodule-%=%) stage
endif

.PHONY: stage-submodules
stage-submodules: $(PN_SUBMODULES:%=stage-submodule-%)


# Empty tech-specific rules (for modules without own content, e.g. 'hw/') ...
.PHONY: $(PN_BUILD_TECHS:%=build-%)
$(PN_BUILD_TECHS:%=build-%):


# Build this module for all techs ...
.PHONY: build-all
build-all: $(PN_BUILD_TECHS:%=build-%)


# Build everything in the right order ...
#   Preparation must be finished before submodules, because it may include the common layer.
#   Submodules must be finished before the current module, since their outputs may be required here.
.PHONY: build
build: $(PN_MODULE_BUILD_DIR) .WAIT build-prepare .WAIT stage-submodules .WAIT build-all





######################### Verify ###############################################


# Verify submodules ...
.PHONY: $(PN_SUBMODULES:%=verify-submodule-%)
$(PN_SUBMODULES:%=verify-submodule-%): build-prepare
ifneq (0,$(VERBOSE))
	$(MAKE) $(PN_BUILD_SUBMAKE_FLAGS) -C $(PN_MODULE_SOURCE_DIR)/$(@:verify-submodule-%=%) verify
else
	@$(MAKE) $(PN_BUILD_SUBMAKE_FLAGS) -C $(PN_MODULE_SOURCE_DIR)/$(@:verify-submodule-%=%) verify
endif

.PHONY: verify-submodules
verify-submodules: $(PN_SUBMODULES:%=verify-submodule-%)


# Empty tech-specific rules (for modules without own content, e.g. 'hw/') ...
.PHONY: $(PN_BUILD_TECHS:%=verify-%)
$(PN_BUILD_TECHS:%=verify-%): verify-%: install-%


# Verify this module for all techs ...
.PHONY: verify-all
verify-all: install-all $(PN_BUILD_TECHS:%=verify-%)


# Verify everything ...
.PHONY: verify
verify: build-prepare .WAIT verify-submodules .WAIT verify-all





######################### Install (on stage) ###################################


# To follow general conventions, on the local level (i.e. as referenced in the
# module Makefiles), the targets are called 'install-*'. However, they always
# write into PN_STAGE_DIR. The globally operating or the internal targets
# are prefixed "stage-" to reflect that they always write to PN_STAGE_DIR,
# not PREFIX.


# Empty tech-specific rules (for modules without own content, e.g. 'hw/') ...
#   We auto-depend on the tech-specific build target.
.PHONY: $(PN_BUILD_TECHS:%=install-%)
$(PN_BUILD_TECHS:%=install-%): install-%: build-%


# Stage this module for all techs ...
#   We auto-depend on the tech-independent build target.
.PHONY: install-all
install-all: build-all $(PN_BUILD_TECHS:%=install-%)


# Stage everything ...
.PHONY: stage
stage: build-prepare .WAIT stage-submodules .WAIT install-all





######################### Install (anywhere) ###################################


# Preset PREFIX ...
ifeq (,$(PREFIX))
  ifneq (,$(PN_SYSTEM_DIR))
    PREFIX := $(PN_SYSTEM_DIR)/piconut
  else
    PREFIX := $(PICONUT)
  endif
endif


# Install to PREFIX ...
#   NOTE: To allow the detection of name collisions, the stage tree is cleaned
#         first. This will cause a larger number of rebuilds, since with the stage
#         directory, the config files and common code is removed as well.
#         This is normal.
#
# TBD+: Add some code to identify files ending with "~" (= name collisions) and report them as errors.
#
.PHONY: install
install: clean-stage .WAIT stage
	@if [ "$(PREFIX)" = "" ]; then \
	  echo "Variable PREFIX is unset: Cannot install"; \
	  exit 1; \
	fi
	@echo "INSTALL $(PN_STAGE_DIR) -> $(PREFIX)"
	@mkdir -p $(PREFIX) && cp -a $(PN_STAGE_DIR)/* $(PREFIX)/





######################### Clean ################################################


# Clean ...
#   Remove the subdirectory of the current module, making any module-specific
#   removals unnecessary.
#   Note: The stage directory is also removed.
.PHONY: clean
clean: clean-stage
ifneq (0,$(VERBOSE))
	rm -fr $(PN_MODULE_BUILD_DIR)
else
	@echo CLEAN $(PN_MODULE_BUILD_DIR)
	@rm -fr $(PN_MODULE_BUILD_DIR)
endif


# Clean the staging area ...
.PHONY: clean-stage
clean-stage:
ifneq (0,$(VERBOSE))
	rm -fr $(PN_STAGE_DIR)
else
	@echo CLEAN $(PN_STAGE_DIR)
	@rm -fr $(PN_STAGE_DIR)
endif





################################################################################
#                                                                              #
#   Help                                                                       #
#                                                                              #
################################################################################


define PN_HELP_TARGETS
	@echo "  build       : Build everything (see MODULES and TECHS below)"
	@echo
	@echo "  verify      : Verify all modules (see MODULES and TECHS below)"
	@echo
	@echo "  install     : Install PicoNut to the directory given by parameter PREFIX"
	@echo
	@echo "  clean       : Cleanup build artefacts"
	@echo
	@echo "  help        : Print this help"

endef

define PN_HELP_TARGETS_SYSTEM
	@echo
	@echo "  program     : Program the FPGA with the system bitfile"

endef


define PN_HELP_PARAMETERS
	@echo "  TECHS :   Basic technologies to build for, given by a colon-separated list."
	@echo "            Example: TECHS=sim:syn"
	@echo "            Possible values:"
	@echo "              sim: Simulation (requires GCC, SystemC)"
	@echo "              syn: Synthesis (requires ICSC and typically Yosys; technology is defined by PICONUT_BOARD)"
	@echo "            (Default = $(subst $() $(),:,$(PN_BUILD_TECHS)))"
	@echo
	@echo "  DEBUG   : 1 = Build without optimizations, 0 = Build release variant (default)"
	@echo
	@echo "  VERBOSE : 1 = Show full commands during build, 0 = Short output (default)"
	@echo
	@echo "  PREFIX  : Destination directory for installations ('install' target)"
	@echo
	@echo "  PICONUT_BOARD : FPGA board or IC technology for synthesis targets (optional)"
	@echo
	@echo "  RISCV         : Installation directory the RISC-V GCC toolchain (optional)"
	@echo
	@echo "  RISCV_SPECS   : GCC specs (optional)"
	@echo
	@echo "  RISCV_LIBC    : GCC libc (optional)"
	@echo
	@echo "  ICSC_HOME     : Installation directory of ICSC (optional, required for synthesis)"

endef

define PN_HELP_PARAMETERS_SYSTEM
	@echo
	@echo "  BITFILE : FPGA bitfile (for the 'program' target)"

endef


ifneq (,$(PN_SYSTEM_DIR))
  PN_HELP_TARGETS := $(PN_HELP_TARGETS) $(HELP_TARGETS_SYSTEM)
  PN_HELP_PARAMETERS := $(PN_HELP_PARAMETERS) $(HELP_PARAMETERS_SYSTEM)
endif


.PHONY: help
help:
	@echo
	@echo "----- Welcome to the PicoNut Build System -----"
	@echo
	@echo "Usage: make [<target>] [<parameter>=<value> ...]"
	@echo
	@echo "Targets:"
	@echo
	$(PN_HELP_TARGETS)
	@echo
	@echo "Parameters:"
	@echo
	$(PN_HELP_PARAMETERS)
	@echo





################################################################################
#                                                                              #
#   Scratch Area (to be removed)                                               #
#                                                                              #
################################################################################


# Debugging ...
#   Requires Debian packages: makefile2graph, graphviz
.PHONY: debug-view-deps
debug-view-deps:
	$(MAKE) -nd | make2graph -b | dot -Tpng -o Makefile-graph.png


test:
	@echo "PN_SOURCE_DIR = $(PN_SOURCE_DIR)"
	@echo "PN_INSTALLED_DIR = $(PN_INSTALLED_DIR)"
	@echo "PN_SYSTEM_DIR = $(PN_SYSTEM_DIR)"
	@echo "PN_BUILD_DIR = $(PN_BUILD_DIR)"
	@echo "PN_SYSTEM_BUILD_DIR = $(PN_SYSTEM_BUILD_DIR)"
	@echo "PN_MODULE_BUILD_DIR = $(PN_MODULE_BUILD_DIR)"
	@echo "PN_STAGE_DIR = $(PN_STAGE_DIR)"
	@echo
	@echo "PN_MODULE_ID = $(PN_MODULE_ID)"
	@echo "PN_MODULE_NAME = $(PN_MODULE_NAME)"
	@echo "PN_MODULE_SOURCE_DIR = $(PN_MODULE_SOURCE_DIR)"
	@echo
	@echo "PN_SUBMODULES = $(PN_SUBMODULES)"
	@echo "PN_SUBMODULES_FILTERED = $(PN_SUBMODULES_FILTERED)"
	@echo
	@echo "PN_NETLIST_OUTPUT_BASE = $(PN_NETLIST_OUTPUT_BASE)"
	@echo "PN_LAYOUT_OUTPUT_BASE = $(PN_LAYOUT_OUTPUT_BASE)"
	@echo
	@echo "PN_ICSC_SOURCES = $(PN_ICSC_SOURCES)"
	@echo "PN_NETLIST_SRC = $(PN_NETLIST_SRC)"
	@echo
	@echo "PN_SW_CROSS(exported) = $$PN_SW_CROSS"
	@echo "PN_SW_OBJDUMP(exported) = $$PN_SW_OBJDUMP"
