#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2026 Johannes Hofmann <johannes.hofmann1@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
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
PN_BUILD_SW := 1

# Model source (local or download) ...
# Note: By using local the dataset ~900MB will be downloaded.
MODEL_SOURCE := download

# Environment ...
PN_SW_OS = bare


# Model download ...
NC_PICONUT_ASSETS := fMAt6PoRYwRCRPr
NC_DATASET_PATH := ai/datasets/ecg_classification-02072026/CPSC.zip
NC_MODEL_PATH := ai/models/ecg_classification-07072026/ecg_classification_model.tflite


# Include PicoNut setup ...
include ../../../../../piconut.mk





################################################################################
#                                                                              #
#   Python Virtual Environment                                                 #
#                                                                              #
################################################################################


PYENV := $(PN_MODULE_BUILD_DIR)/.pyenv-complete

.phony: pyenv
pyenv: $(PYENV)

$(PYENV):
ifneq (0,$(VERBOSE))
	mkdir -p $(PN_MODULE_BUILD_DIR) && \
	cd $(PN_MODULE_BUILD_DIR) && \
	rm -fr venv && \
	python3 -m venv venv && \
	. venv/bin/activate && \
	pip3 install --upgrade pip && \
	pip3 install -r $(PN_MODULE_SOURCE_DIR)/piconut_ecg-diagnosis/requirements.txt && \
	touch $(PYENV)
else
	@echo $(PN_BUILD_PREFIX)CREATING-PYENV venv-ai-ecg_classification; \
	mkdir -p $(PN_MODULE_BUILD_DIR) && \
	cd $(PN_MODULE_BUILD_DIR) && \
	rm -fr venv && \
	python3 -m venv venv && \
	. venv/bin/activate && \
	pip3 install --upgrade pip > /dev/null && \
	pip3 install -r $(PN_MODULE_SOURCE_DIR)/piconut_ecg-diagnosis/requirements.txt > /dev/null && \
	touch $(PYENV)
endif


define ACTIVATE_VENV
VENV=$(PN_INSTALLED_DIR)/tools/share/venv-ai-ecg_classification; \
test -f $(PYENV) && VENV=$(PN_MODULE_BUILD_DIR)/venv; \
test -d $$VENV || { echo "ERROR: No Python environment found: Run 'make pyenv' first!"; exit 1; }; \
. $$VENV/bin/activate
endef





################################################################################
#                                                                              #
#   Model                                                                      #
#                                                                              #
################################################################################


MODULE := $(PN_MODULE_NAME)

MODEL := $(PN_MODULE_BUILD_DIR)/download/$(MODULE).tflite
MODEL_SRC := $(PN_MODULE_BUILD_DIR)/gen/$(MODULE)_data.cpp
vpath $(MODULE)_data.cpp $(PN_MODULE_BUILD_DIR)/gen


# Get Model
.phony: model
model: $(MODEL)

ifeq (download, $(MODEL_SOURCE))
$(MODEL):
ifneq (0,$(VERBOSE))
	mkdir -p $(dir $@); \
	curl -u $(NC_PICONUT_ASSETS): \
		https://cloud.hs-augsburg.de/public.php/webdav/$(NC_MODEL_PATH) -o $@
else
	@echo $(PN_BUILD_PREFIX)DOWNLOAD $(@:$(PN_MODULE_BUILD_DIR)/%=%); \
	mkdir -p $(dir $@); \
	curl -su $(NC_PICONUT_ASSETS): \
		https://cloud.hs-augsburg.de/public.php/webdav/$(NC_MODEL_PATH) -o $@
endif



else ifeq (local,$(MODEL_SOURCE))

DATASET_ZIP := $(PN_MODULE_BUILD_DIR)/CPSC.zip
DATASET_DIR := $(PN_MODULE_BUILD_DIR)/dataset/CPSC


$(DATASET_ZIP):
ifneq (0,$(VERBOSE))
	mkdir -p $(dir $@); \
	curl -u $(NC_PICONUT_ASSETS): \
		https://cloud.hs-augsburg.de/public.php/webdav/$(NC_DATASET_PATH) -o $@
else
	@echo $(PN_BUILD_PREFIX)DOWNLOAD $(@:$(PN_MODULE_SOURCE_DIR)/%=%); \
	mkdir -p $(dir $@); \
	curl -su $(NC_PICONUT_ASSETS): \
		https://cloud.hs-augsburg.de/public.php/webdav/$(NC_DATASET_PATH) -o $@
endif

$(DATASET_DIR): $(DATASET_ZIP)
ifneq (0,$(VERBOSE))
	mkdir -p $@
	cd $@ && unzip -o -j $<
else
	@echo $(PN_BUILD_PREFIX)EXTRACT $(<:$(PN_MODULE_SOURCE_DIR)/%=%); \
	mkdir -p $@; \
	cd $@ && unzip -o -j $< > /dev/null
endif


$(MODEL): $(DATASET_DIR) $(PYENV)
	mkdir -p $(PN_MODULE_BUILD_DIR)/model/export
	$(ACTIVATE_VENV) && cd piconut_ecg-diagnosis && \
	python3 preprocess.py --data-dir $(DATASET_DIR) && \
	python3 baselines.py  --data-dir $(DATASET_DIR) --classifier LR && \
	python3 main.py       --data-dir $(DATASET_DIR) --leads all --use-gpu \
		--model-path $(PN_MODULE_BUILD_DIR)/model/resnet34_CPSC_all_42.pth && \
	python3 predict.py    --data-dir $(DATASET_DIR) --leads all --use-gpu \
		--model-path $(PN_MODULE_BUILD_DIR)/model/resnet34_CPSC_all_42.pth && \
	python3 export_tflite.py --data-dir $(DATASET_DIR) --leads all \
		--model-path $(PN_MODULE_BUILD_DIR)/model/resnet34_CPSC_all_42.pth \
		--onnx-path $(PN_MODULE_BUILD_DIR)/model/export/model.onnx \
		--output-dir $(PN_MODULE_BUILD_DIR)/model/export
	mkdir -p $(dir $@)
	cp $(PN_MODULE_BUILD_DIR)/model/export/model_int8.tflite $@

endif


# Convert model to source code ...
$(MODEL_SRC): $(MODEL)
	@mkdir -p $(dir $@)
	@echo '#include "$(PN_MODULE_SOURCE_DIR)/$(MODULE)_data.h"' > $@
	@echo "" >> $@
	@xxd -n $(MODULE)_data -i $< | \
		sed -e 's/unsigned char/const uint8_t/g' \
		    -e 's/unsigned int/const size_t/g' >> $@





################################################################################
#                                                                              #
#   Building                                                                   #
#                                                                              #
################################################################################


# Define the library ...
SRC := $(MODULE)_data.cpp $(MODULE)_settings.cpp


# Define set of object files and output binary ...
OBJ := $(SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/%.o)
LIB := $(PN_MODULE_BUILD_DIR)/lib$(MODULE).a


# Automatic dependencies ...
-include $(OBJ:%.o=%.d)


# Set object files to link ...
$(LIB): $(OBJ)





################################################################################
#                                                                              #
#   Exported Targets                                                           #
#                                                                              #
################################################################################


# Exported targets ...
.PHONY: build-all
build-all: $(LIB)


.PHONY: install-all
install-all:
	$(PN_INSTALL_SW_INCLUDE) $(MODULE)_data.h
	$(PN_INSTALL_SW_INCLUDE) $(MODULE)_settings.h
	$(PN_INSTALL_SW_LIB) $(LIB)
ifneq ($(wildcard $(PN_MODULE_BUILD_DIR)/venv),)
	$(PN_INSTALL_TREE_BRIEF) tools/share/venv-ai-ecg_classification $(PN_MODULE_BUILD_DIR)/venv
	sed -i "s#$(PN_MODULE_BUILD_DIR)/venv#$(PN_STAGE_DIR)/tools/share/venv-ai-ecg_classification#g" $(PN_STAGE_DIR)/tools/share/venv-ai-ecg_classification/pyvenv.cfg $(PN_STAGE_DIR)/tools/share/venv-ai-ecg_classification/bin/*
endif
