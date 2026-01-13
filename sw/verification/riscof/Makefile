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

################# Directory Makefile Include ###################################
PICONUT_HOME ?= $(CURDIR)/../../..
include $(PICONUT_HOME)/directory-base.mk

################## Configuration ###############################################
# Set this to update the riscv-arch-test version
RISCV_ARCH_TEST_VERSION ?= 3.9.1

################## Variables ###################################################
RISCOF ?= $(shell command -v riscof)
VERBOSE ?= 0
TRACE_LVL ?= 0
VERBOSE_SUB_CMD := $(if $(filter 1,$(VERBOSE)),--verbose debug,)

# Colors
PINK=\033[1;35m
RED=\033[1;31m
GREEN=\033[1;32m
NC=\033[0m

################## Targets #####################################################

all: help

.PHONY: help
help:
	@echo "Makefile to run the RISC-V architecture test framework (Riscof)"
	@echo "Targets:"
	@echo "  run                : Run riscof"
	@echo "  run-trace          : Run riscof with simulator trace enabled"
	@echo "  update-arch-test   : Ensure riscv-arch-test is cloned and up to date"
	@echo "  clean              : Delete files created from riscof"
	@echo "  clean-system       : Delete files created during the build process of the specified system"
	@echo "  clean-all          : Delete files created during the build process of the software and from riscof"
	@echo "PARAMETERS:"
	@echo "  VERBOSE            : To enable verbose debug output"
	@echo "  TRACE_LVL          : Set the vcd trace level"

.PHONY: run
run: build-sim update-arch-test copy-custom-tests run-riscof

.PHONY: run-trace
run-trace: prepare-trace run

prepare-trace:
	@$(eval TRACE_LVL := 2)

build-sim:
	+$(MAKE) -C $(PNS_HW_DIR) build-sim

.PHONY: update-arch-test
update-arch-test:
	@if [ ! -d "riscv-arch-test" ]; then \
		echo "$(PINK)Cloning riscv-arch-test...$(NC)"; \
		git clone https://github.com/riscv-non-isa/riscv-arch-test; \
		git -C riscv-arch-test checkout $(RISCV_ARCH_TEST_VERSION); \
	fi

	@present_arch_test_version=$$(git -C riscv-arch-test describe --tags --abbrev=0 --match [0-9]*.[0-9]*.[0-9]* 2>/dev/null || true); \
	if [ "$$present_arch_test_version" != "$(RISCV_ARCH_TEST_VERSION)" ]; then \
		echo "$(PINK)Updating to riscv-arch-test version $(RISCV_ARCH_TEST_VERSION)$(NC)"; \
		git -C riscv-arch-test fetch --tags; \
		git -C riscv-arch-test reset --hard $(RISCV_ARCH_TEST_VERSION); \
	fi

copy-custom-tests:
	@echo "$(PINK)Copying custom tests...$(NC)"
	@cp -r custom-tests/* riscv-arch-test/riscv-test-suite/
	@git -C riscv-arch-test add .
	@git -C riscv-arch-test commit -m 'temp: custom-tests' >/dev/null 2>&1 || true

run-riscof:
	@echo "$(PINK)Running riscof...$(NC)"
	@TRACE_LVL=$(TRACE_LVL) $(RISCOF) $(VERBOSE_SUB_CMD) run --config=config.ini \
		--suite=riscv-arch-test/riscv-test-suite/ \
		--env=riscv-arch-test/riscv-test-suite/env \
		--no-browser; \
		status=$$?; \
		if [ $$status -eq 0 ]; then \
			echo "$(GREEN)riscof completed successfully.$(NC)"; \
		else \
			echo "$(RED)riscof failed with exit code $$status.$(NC)"; \
			exit $$status; \
		fi

.PHONY: clean-riscof, clean, clean-system, clean-all
clean-riscof:
clean:
	rm -rf riscv-arch-test riscof_work

clean-system:
	+$(MAKE) -C $(PNS_HW_DIR) clean

clean-all: clean-riscof clean-system
