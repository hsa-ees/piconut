#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#  Description:
#    Main Makefile of the project.
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


# Submodules ...
#   NOTE: By default, systems and doc (layer 3) are not built.
#         They must be built by setting MODULES explicitly.
#
#    default modules for a top-level build ...
PN_SUBMODULES := hw sw boards
#    override if MODULES is defined ...
ifneq (,$(MODULES))
  PN_SUBMODULES := $(subst :, ,$(MODULES))
endif


# Include PicoNut setup ...
include piconut.mk





################################################################################
#                                                                              #
#   Help                                                                       #
#                                                                              #
################################################################################


define HELP_TARGETS_LOCAL
	@echo
	@echo "  hello       : Build a (simulated) demo system and run a RISC-V program"
	@echo
	@echo "  hello-ulx3s : Build a hardware demo system for the ULX3S board (TBD)"
	@echo "                (requires the board and the ICSC & Yosys toolchain)"
	@echo
	@echo "  howdy       : Ask me how I'm doing and how *you* can help *me*!"

endef


PN_HELP_TARGETS := $(PN_HELP_TARGETS) $(HELP_TARGETS_LOCAL)


define HELP_PARAMETERS_LOCAL
	@echo "  MODULES : Select subset of modules, given by a colon-separated list of paths."
	@echo "            Example: MODULES=hw/cpu:hw/uart_soft:sw/apps/hello_piconut"
	@echo "            (Default = $(subst $() $(),:,$(PN_SUBMODULES)))"
	@echo
	@echo "            Note: The documentation and the supplied systems are not built by"
	@echo "            default. To build the documentation or all demo systems,"
	@echo "            add 'MODULES=doc' or 'MODULES=systems' to the make invocation."
	@echo

endef


PN_HELP_PARAMETERS := $(HELP_PARAMETERS_LOCAL) $(PN_HELP_PARAMETERS)





################################################################################
#                                                                              #
#   Global Targets                                                             #
#                                                                              #
################################################################################


# Demos ...
#   The recipe commands are not hidden by intention.
.PHONY: hello
hello:
	$(MAKE) -C systems/refdesign TECHS=sim run-hello


.PHONY: hello-ulx3s
hello-ulx3s:
	$(MAKE) -C systems/refdesign TECHS=syn build
	$(MAKE) -C systems/refdesign TECHS=syn program


# Howdy ...
PN_HOWDY_EXCLUDES := *~ *.svg build

.PHONY: howdy
howdy:
	@echo
	@echo "Hi, I'm fine! Thank you for asking me about how I'm doing."
	@echo
	@echo "However, the PicoNut project is still growing, and there are a lot of things"
	@echo "left to do. If you want to help, please look into the following places:"
	@echo
	@( grep -nr --exclude-dir=.git $(PN_HOWDY_EXCLUDES:%=--exclude=%) -E "TBD\+(\([[:alnum:]_]+\))?:" || true )
	@( grep -nr --exclude-dir=.git $(PN_HOWDY_EXCLUDES:%=--exclude=%) -E "TBD(\([[:alnum:]_]+\))?:" | grep -v "TBD(Sam)" || true )
	@( grep -nr --exclude-dir=.git $(PN_HOWDY_EXCLUDES:%=--exclude=%) -E "TBD\-(\([[:alnum:]_]+\))?:" || true )
	@echo
	@echo "If you decide to help:"
	@echo
	@echo "1. Please first mark the place with your name, for example, change it"
	@echo "   to 'TBD(Sam): ...', so that others see that you are working on it."
	@echo
	@echo "2. Then resolve the issue until you believe it is resolved."
	@echo
	@echo "3. Mark the issue presumably resolved by changing the title to 'TBD(Sam):RESOLVED:'"
	@echo "   (replace 'Sam' with your name or initials)."
	@echo
	@echo "4. AFTER the original reporter has agreed that the issue is resolved, remove the"
	@echo "   comment and, depending on your location and role:"
	@echo "   - commit the fix (authorized core developers)"
	@echo "   - prepare and submit a merge request (developers with access to the Gitlab repo)"
	@echo "   - send a patch to the PicoNut team: https://ees.tha.de/piconut"
	@echo
	@echo "Thank you for improving PicoNut!"
	@echo
