#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025-2026 Gundolf Kiefer <gundolf.kiefer@tha.de>
#     Technische Hochschule Augsburg / Technical University of Applied Sciences
#
#  Description:
#    Makefile for the RISC-V AI Playground (Jupyter Notebook).
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


include ../../../../piconut.mk





################################################################################
#                                                                              #
#   Building a JupyterLab / PyTorch Environment                                #
#                                                                              #
################################################################################


.phony: pyenv
pyenv: $(PN_MODULE_BUILD_DIR)/.pyenv-complete


$(PN_MODULE_BUILD_DIR)/.pyenv-complete:
	mkdir -p $(PN_MODULE_BUILD_DIR) && \
	cd $(PN_MODULE_BUILD_DIR) && \
	rm -fr venv && \
	python3 -m venv venv && \
	. venv/bin/activate && \
	pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu && \
	pip3 install jupyterlab matplotlib pandas && \
	touch $(PN_MODULE_BUILD_DIR)/.pyenv-complete





################################################################################
#                                                                              #
#   Running Notebooks                                                          #
#                                                                              #
################################################################################


.phony: run
run:
	VENV=$(PN_INSTALLED_DIR)/tools/share/venv-ai-playground; \
	test -f $(PN_MODULE_BUILD_DIR)/.pyenv-complete && VENV=$(PN_MODULE_BUILD_DIR)/venv; \
	test -d $$VENV || ( echo "ERROR: No Python environment found: Run 'make pyenv' first!"; exit 1; ) && \
	. $$VENV/bin/activate && \
	jupyter-lab playground-lenet5.ipynb





################################################################################
#                                                                              #
#   Global & Exported Targets                                                  #
#                                                                              #
################################################################################


build-all: pyenv


install-all:
	$(PN_INSTALL_TREE_BRIEF) tools/share/venv-ai-playground $(PN_MODULE_BUILD_DIR)/venv
	sed -i "s#$(PN_MODULE_BUILD_DIR)/venv#$(PN_STAGE_DIR)/tools/share/venv-ai-playground#g" $(PN_STAGE_DIR)/tools/share/venv-ai-playground/pyvenv.cfg $(PN_STAGE_DIR)/tools/share/venv-ai-playground/bin/*

  # NOTE [2026-01-21]: The final sed command is a workaround and may not work in the future.
  #   At present, this is the only way to relocate the Python VEnv.
