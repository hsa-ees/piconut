#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
#                2024 Lukas Bauer <lukas.bauer1@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#  Description:
#    Master Makefile for the documentation.
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


PN_BUILD_COMMON := 0
PN_BUILD_HW := 0
PN_BUILD_SW := 0


include ../piconut.mk


SPHINX_SRC := manual
SPHINX_OPTS ?=




################################################################################
#                                                                              #
#   Help                                                                       #
#                                                                              #
################################################################################


define HELP_TARGETS_LOCAL
	@echo
	@echo "  sphinx-*    : Call a Sphinx target directly. Try 'make sphinx-help' for more information."

endef


PN_HELP_TARGETS := $(PN_HELP_TARGETS) $(HELP_TARGETS_LOCAL)





################################################################################
#                                                                              #
#   Global Targets                                                             #
#                                                                              #
################################################################################


# Common variables
SPHINX_CMD = sphinx-build -M html $(SPHINX_SRC) $(PN_MODULE_BUILD_DIR) $(SPHINX_OPTS)
PARAM_WARN_FILTER = grep -v "warning: argument '.*' of command @param is not found in the argument list of SC_MODULE"

build-all:
	@mkdir -p $(PN_MODULE_BUILD_DIR)
ifneq (0,$(VERBOSE))
	@{ \
		BUILD_DIR=$(PN_MODULE_BUILD_DIR) $(SPHINX_CMD) 2>&1 >&3 \
		| $(PARAM_WARN_FILTER) >&2; \
	} 3>&1
else
	@echo "$(PN_BUILD_PREFIX)SPHINX $(SPHINX_SRC)"
	@(BUILD_DIR=$(PN_MODULE_BUILD_DIR) $(SPHINX_CMD) \
	  > $(PN_MODULE_BUILD_DIR)/manual.log 2>&1 \
	  || (cat $(PN_MODULE_BUILD_DIR)/manual.log && exit 1))
endif

# Verifies that there are no warnings e.g. for broken links or doxygen
verify: clean
	@mkdir -p $(PN_MODULE_BUILD_DIR)
	@# cannot reuse build-all target as VERBOSE cannot be set here
	@BUILD_DIR=$(PN_MODULE_BUILD_DIR) $(SPHINX_CMD) \
	  2>&1 | $(PARAM_WARN_FILTER) > $(PN_MODULE_BUILD_DIR)/manual.log

	@if grep -i "warning:" $(PN_MODULE_BUILD_DIR)/manual.log | grep -v "RemovedInSphinx90Warning"; then \
		cat $(PN_MODULE_BUILD_DIR)/manual.log; \
		echo "Verification failed due to warnings."; \
		exit 1; \
	fi



install-all:
	$(PN_INSTALL_TREE) doc/manual $(PN_MODULE_BUILD_DIR)/html


# General target: allows to call any Sphinx target, if desired ...
sphinx-%: Makefile
	@sphinx-build -M $(@:sphinx-%=%) $(SPHINX_SRC) $(PN_MODULE_BUILD_DIR) $(SPHINX_OPTS) $(O)


# View ...
.PHONY: view
view: build-all
	xdg-open $(PN_MODULE_BUILD_DIR)/html/index.html &
