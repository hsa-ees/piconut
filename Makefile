#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2010-2021 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
#                          Felix Wagner <felix.wagner1@hs-augsburg.de>
#                          Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
#					  2024 Lukas Bauer <lukas.bauer1@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#  Description:
#    PicoNut top-level Makefile.
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
#     Settings                                                                 #
#                                                                              #
################################################################################


# When settings.sh is sourced you can access the root of the PicoNut project
# - PICONUT_HOME is to be set within a project / specific make files to point to the PicoNut (editable) sources to work with.

# define default target
.PHONY: all
all: build-all

# This file is located at the root of the PicoNut git/install dir
PICONUT_HOME ?= $(abspath $(CURDIR))

ifdef INSTALL_PREFIX
INSTALL_DIR = $(abspath $(INSTALL_PREFIX))
endif

# include common variables
include directory-base.mk

# PicoNut Xilinx IP core target directory
IP_XILINX_TARGET_DIR ?= paranut_ip

################################################################################
#                                                                              #
#     Help targets                                                 #
#                                                                              #
################################################################################

# Update version file
$(VERSION_ENV): update-version

.PHONY: update-version
update-version:
	$(TOOLS_BIN_DIR)/generate_versionfile.sh -g -t $(VERSION_ENV) -p ./


# Print usage info ...
.PHONY: help
help:
	@echo
	@echo "Main Makefile for building and installing PicoNut"
	@echo
	@echo "Usage: make [<target>] [<parameter>=<value> ...]"
	@echo
	@echo "Targets:"
	@echo
	@echo "  build                 : Build SystemC simulator and the PicoNut support library,"
	@echo "                          prepare for installation"
	@echo "  clean                 : Delete most files created during the build process"
	@echo
	@echo "  help                  : Print this help"
	@echo
	@echo "  hello                 : Build and simulate an example PicoNut program"
	@echo "                          (from '$(SW_APPLICATIONS_DIR)/hello_newlib')"
	@echo
	@echo "  build-all             : Build everything, including software projects and systems"
	@echo
	@echo "  build-sim             : Build the SystemC simulator ('libparanutsim.a' and 'pn-sim')"
	@echo "  build-lib             : Build the PicoNut support library ('libparanut.a')"
	@echo "  build-doc             : Build the PicoNut documentation files"
	@echo
	@echo "  install               : Install PicoNut to the directory given by parameter INSTALL_DIR"
	@echo "                          (default: $(INSTALL_DIR_DEFAULT))"
	@echo
	@echo "  run-application       : Run an application defined by APPLICATION="
	@echo "                          (located in $(SW_APPLICATIONS_DIR))"
	@echo "Parameters:"
	@echo
	@echo "  INSTALL_PREFIX        : Target directory for make install"
	@echo "  USE_SYSC_ICSC         : Use the ICSC config section from systemc-config.mk to enable its libsystemc"
	@echo "  USE_SYSC_ACCELLERA    : Use the Accellera config section from systemc-config.mk to enable its libsystemc"

# Not implemented yet needs to be done
# @echo "  build-systems     : Build all systems (may be defined by PN_SYSTEMS,"
# @echo "                      defaults to the ones declared in 'systems/Makefile')"
# @echo "  build-sw          : Build all software projects (as defined by PN_SW_DIRS)"
# @echo "  test-sim          : Run test to verify basic functionality of simulation"
# @echo "  test-vhdl         : Run testbenches to test vhdl code"
# @echo "  test-sysc         : Run testbenches to test sysc code"

# Build and run "hello world" example ...
PHONY: hello
hello: build
	$(MAKE) -C $(SW_DIR)/applications/hello_newlib sim





################################################################################
#                                                                              #
#     Building: Simulator, Support Libraries                                   #
#                                                                              #
################################################################################

# Build target alias for build-all
.PHONY: build
build: build-all

# Build everything except vendor-specific synthesis products ...
.PHONY: build-all
build-all: build-sim build-lib


.PHONY: build-sim
build-sim: $(PN_CONFIG_MK) $(VERSION_ENV)
	@echo
	@echo "######### Building SystemC Simulator #########";
	@echo
	$(MAKE) -C $(PNS_HW_DIR) build-sim


# Build the PicoNut support library/libraries ...
.PHONY: build-lib
build-lib: $(VERSION_ENV)
	@echo
	@echo "######### Building PicoNut Support Library #########";
	@echo
	+$(MAKE) -C $(HAL_DIR)/libuzlib build-lib

.PHONY: build-doc
build-doc:
	@echo
	@echo "######### Building PicoNut Documentation #########";
	@echo
	$(MAKE) -C $(DOC_SRC_DIR)/pn-manual html latexpdf


################################################################################
#                                                                              #
#     Running: Targets for running software from the main Makefile             #
#                                                                              #
################################################################################
.PHONY: run-application
run-application: build
	$(MAKE) -C $(SW_DIR)/applications/$(APPLICATION) sim

################################################################################
#                                                                              #
#     Installing                                                               #
#                                                                              #
################################################################################

# Settings files
ROOT_FILES =

# Top-Level info, config and Makefile
SRC_FILES = README.md \
            Makefile \
			directory-base.mk \
			systemc-config.mk


# Install the PicoNut repository to the directory specified by INSTALL_DIR ...
.PHONY: install
install: build
	@echo
	@echo "######### Installing to $(INSTALL_DIR) #########"
	@echo
	@if [ -d src ]; then \
	  echo "INFO: The install target is only available inside the source repository!";\
	else \
	  # Create target dir \
	    mkdir -p $(INSTALL_DIR) \
	  # Install ROOT_FILES \
	    for FILE in $(ROOT_FILES); do \
	      install -Dp -m 644 -t $(INSTALL_DIR)/`dirname $$FILE` $$FILE; \
	    done; \
	  # Install SRC_FILES \
	    for FILE in $(SRC_FILES); do \
	      install -Dp -m 644 -t $(INSTALL_DIR)/`dirname $$FILE` $$FILE; \
	    done; \
	  # Install DOCUMENTATION \
	    $(MAKE) -C $(DOC_SRC_DIR)/pn-manual install; \
	  # Install SW \
	    $(MAKE) -C $(SW_DIR) install; \
	  # Install HW \
	    $(MAKE) -C $(HW_DIR) install; \
	  # Install tools \
	    $(MAKE) -C $(TOOLS_DIR) install; \
	  # Install config creator \
	    $(MAKE) -C $(CFG_CREATOR_DIR) install; \
	  # Install default system \
	    $(MAKE) -C $(SYSTEMS_DIR) install; \
	  # Add $(VERSION_ENV) \
	    cp $(VERSION_ENV)  $(INSTALL_DIR)/version.env; \
	fi

################################################################################
#                                                                              #
#     Testing                                                                  #
#                                                                              #
################################################################################

# test basic functionality of simulator
PHONY: test-sim
test-sim: test-hello_newlib




################################################################################
#                                                                              #
#     Cleaning                                                                 #
#                                                                              #
################################################################################

# Clean up most files created during the build process
PHONY: clean
clean: clean-hw  clean-systems clean-doc clean-sw
	@rm -f $(VERSION_ENV)

# These are most likely never intentionally called by a user, but include them here for documentation
# These are also not tracked by version control and created by prepase_sdl.sh in sw/sdl_demo

PHONY: clean-hw
clean-hw:
	@$(MAKE) --silent -C $(HW_DIR) clean-all

PHONY: clean-sw
clean-sw:
	@$(MAKE) --silent -C $(SW_DIR) clean-all

PHONY: clean-systems
clean-systems:
	@$(MAKE) --silent -C $(SYSTEMS_DIR) clean-all

PHONY: clean-doc
clean-doc:
	@$(MAKE) --silent -C $(DOC_SRC_DIR)/pn-manual clean
