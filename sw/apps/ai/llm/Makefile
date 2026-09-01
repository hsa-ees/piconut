#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2026 Johannes Hofmann <johannes.hofmann1@tha.de>
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

PN_BUILD_HW := 0
PN_BUILD_SW := 1

# Environment ...
PN_SW_OS := bare


# Base ISA (for compiling) ...
PN_MARCH := rv32im_zicsr # VEXT=OFF 
#~ PN_MARCH := rv32im_zve32x # VEXT=ON

# Base ISA (for SPIKE simulation) ...
SPIKE_MARCH := rv32im # VEXT=OFF
#~ SPIKE_MARCH := rv32imv_zvl1024b_zicntr_zihpm # VEXT=ON


# Memory ...
PN_CFG_SYS_CODE_SIZE  := 0x0A000000 # VEXT=OFF
#~ PN_CFG_SYS_CODE_SIZE  := 0x0A000000 # VEXT=ON (not tested)
PN_CFG_SYS_RAM_SIZE   := 0x02500000


# Include PicoNut setup ...
include ../../../../piconut.mk


build-prepare:
	$(PN_PREBUILD) MODULES=sw/libs/piconut_muriscv-nn:sw/libs/ai/models/llm_model





################################################################################
#                                                                              #
#   Building                                                                   #
#                                                                              #
################################################################################


# Define the app ...
APP := $(PN_MODULE_NAME)
SRC := main.cpp \
	$(APP)_data/$(APP)_input_data.cpp

PN_SW_CFLAGS += -DTFLITE_EMULATE_FLOAT -DTF_LITE_STATIC_MEMORY \
	-DTF_LITE_USE_GLOBAL_CMATH_FUNCTIONS -DTF_LITE_USE_GLOBAL_MAX -DTF_LITE_USE_GLOBAL_MIN \
	-fno-exceptions -fno-rtti -fno-use-cxa-atexit \
  -I$(PN_STAGE_DIR)/sw/include/pn_tflm \
  -I$(PN_STAGE_DIR)/sw/include/pn_tflm/third_party/ruy \
  -I$(PN_STAGE_DIR)/sw/include/pn_tflm/third_party/gemmlowp \
  -I$(PN_STAGE_DIR)/sw/include/pn_tflm/third_party/kissfft \
  -I$(PN_STAGE_DIR)/sw/include/pn_tflm/third_party/flatbuffers \
  -I$(PN_STAGE_DIR)/sw/include/pn_tflm/third_party/flatbuffers/include \
	-I$(PN_STAGE_DIR)/sw/include/pn_tflm/Include \
	-I$(PN_STAGE_DIR)/sw/include/pn_tflm/Include/CMSIS/NN -march=$(PN_MARCH) -mabi=ilp32

LD_FLAGS := -lllm_model -lpn_tflm -lm -std=c++11 -lstdc++
PN_SW_LDFLAGS += $(LD_FLAGS)

# Define set of object files and output binary ...
OBJ := $(SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/%.o)
BIN := $(PN_MODULE_BUILD_DIR)/$(APP).rv32
SPIKE_BIN = $(PN_MODULE_BUILD_DIR)/$(APP)-spike.rv32

# Linking for Spike (no libpiconut, piconut.ld, ...)
$(SPIKE_BIN): $(OBJ)
ifneq (0,$(VERBOSE))
	$(PN_SW_LD) -o $@ $^ -march=$(PN_MARCH) -mabi=ilp32 -L$(PN_STAGE_DIR)/sw/lib $(LD_FLAGS)
else
	@echo $(PN_BUILD_PREFIX)SW-LD-SPIKE $(@:$(PN_MODULE_BUILD_DIR)/%=%); \
	$(PN_SW_LD) -o $@ $^ -march=$(PN_MARCH) -mabi=ilp32 -L$(PN_STAGE_DIR)/sw/lib $(LD_FLAGS)
endif

# Automatic dependencies ...
#   The default compile command $(PN_SW_CC) generates a .d file for each .o file
#   containing the .h files it depends on. These files are included here.
-include $(OBJ:%.o=%.d)


# Set object files to link ...
$(BIN): $(OBJ)





################################################################################
#                                                                              #
#   Exported Targets                                                           #
#                                                                              #
################################################################################


build-all: $(SPIKE_BIN) $(BIN)

install-all:
	$(PN_INSTALL_SW_BIN) $(BIN)



##### Additional Main Targets #####

.PHONY: sim
sim: build-prepare .WAIT $(SPIKE_BIN)
	spike --isa=$(SPIKE_MARCH) pk -s $(SPIKE_BIN)

