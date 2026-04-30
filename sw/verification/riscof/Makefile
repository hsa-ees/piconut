#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025 Niklas Sirch <niklas.sirch1@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#  Description:
#      Makefile for running RISCOF with custom tests
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

PN_BUILD_COMMON := 0
PN_BUILD_HW := 0
PN_BUILD_SW := 0

include ../../../piconut.mk

# TBD: include in main piconut.mk as this is common for all python-based tools
export PYTHONPYCACHEPREFIX := $(PN_MODULE_BUILD_DIR)/__pycache__


## Configuration
# Set this to update the riscv-arch-test version
RISCV_ARCH_TEST_VERSION ?= 3.10.0
RISCV_ARCH_TEST ?= $(CURDIR)/riscv-arch-test

## Variables
RISCOF ?= $(shell command -v riscof)
VERBOSE ?= 0
SIMULATOR_BIN ?= $(PN_BUILD_DIR)/systems/refdesign/hw/sim/refdesign_sim

define HELP_TARGETS
	@echo "	run                : Run riscof"
	@echo "	update-arch-test   : Ensure riscv-arch-test is cloned and up to date"
	@echo "	clean              : Delete files created from riscof"
endef

define HELP_PARAMETERS
	@echo "	VERBOSE            : To enable verbose debug output"
	@echo "	SIM_TRACE_LEVEL    : Set the vcd trace level (default: 0)" # From normal simulator
	@echo "	SIMULATOR_BIN      : Set the system to build (default: $(PN_BUILD_DIR)/systems/refdesign/hw/sim/refdesign_sim)"
endef

PN_HELP_PARAMETERS = $(HELP_PARAMETERS)
PN_HELP_TARGETS = $(HELP_TARGETS)

## Targets

	

.PHONY: update-arch-test
update-arch-test:
	@if [ ! -d "$(RISCV_ARCH_TEST)" ]; then \
		echo "$(PN_BUILD_PREFIX) Cloning riscv-arch-test..."; \
		git clone https://github.com/riscv-non-isa/riscv-arch-test $(RISCV_ARCH_TEST); \
		git -C $(RISCV_ARCH_TEST) checkout $(RISCV_ARCH_TEST_VERSION); \
	fi

	@present_arch_test_version=$$(git -C $(RISCV_ARCH_TEST) describe --tags --abbrev=0 --match [0-9]*.[0-9]*.[0-9]* 2>/dev/null || true); \
	if [ "$$present_arch_test_version" != "$(RISCV_ARCH_TEST_VERSION)" ]; then \
		echo "$(PN_BUILD_PREFIX) Updating to riscv-arch-test version $(RISCV_ARCH_TEST_VERSION)"; \
		git -C $(RISCV_ARCH_TEST) fetch --tags; \
		git -C $(RISCV_ARCH_TEST) reset --hard $(RISCV_ARCH_TEST_VERSION); \
	fi

copy-custom-tests: update-arch-test
	@echo "$(PN_BUILD_PREFIX) Copying custom tests"
	@cp -r custom-tests/* $(RISCV_ARCH_TEST)/riscv-test-suite/
	@git -C $(RISCV_ARCH_TEST) add .
	@git -C $(RISCV_ARCH_TEST) commit -m 'temp: custom-tests' >/dev/null 2>&1 || true

RISCOF_WORK_DIR := $(PN_MODULE_BUILD_DIR)/riscof_work
RISCOF_LOG_FILE := $(PN_MODULE_BUILD_DIR)/riscof.log

RISCOF_RUN_CMD := DUT_EXE=$(SIMULATOR_BIN) $(RISCOF) --verbose debug run --config=config.ini \
	--suite=$(RISCV_ARCH_TEST)/riscv-test-suite/ \
	--env=$(RISCV_ARCH_TEST)/riscv-test-suite/env \
	--work-dir=$(RISCOF_WORK_DIR) \
	--no-browser

.PHONY: run
run: update-arch-test copy-custom-tests
	@if [ ! -f "$(SIMULATOR_BIN)" ]; then \
		echo "$(PN_BUILD_PREFIX) Simulator binary '$(SIMULATOR_BIN)' not found. Aborting."; \
		echo "$(PN_BUILD_PREFIX) Please build the simulator or use present verify target from the system"; \
		exit 1; \
	fi
	@echo "$(PN_BUILD_PREFIX) Running riscof"
ifneq ($(VERBOSE),0)
	@$(RISCOF_RUN_CMD); \
	if ! grep -iq '>0failed<' $(RISCOF_WORK_DIR)/report.html; then \
		echo "$(PN_BUILD_PREFIX) riscof failed."; \
		exit 1; \
	fi; \
	echo "$(PN_BUILD_PREFIX) riscof executed successfully" 
else
	@mkdir -p $(dir $(RISCOF_LOG_FILE))
	@$(RISCOF_RUN_CMD) > $(RISCOF_LOG_FILE) 2>&1; \
	if ! grep -iq '>0failed<' $(RISCOF_WORK_DIR)/report.html; then \
		cat $(RISCOF_LOG_FILE); \
		echo "$(PN_BUILD_PREFIX) riscof failed."; \
		exit 1; \
	fi
endif

clean-riscof:
	rm -rf $(RISCV_ARCH_TEST)
