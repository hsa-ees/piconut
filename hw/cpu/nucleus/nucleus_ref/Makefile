#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
#                     Tristan Kundrat <tristan.kundrat@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#  Description:
#    Makefile of the module CPU which provides the PicoNut cores of
#    all currently available Capability levels
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
#   Define the submodules this module depends on.
#
#   The build system will automatically build them by the 'build' target before
#   executing the 'build-all' recipe defined below. Submodules are "installed"
#   into $PN_BUILD_DIR, so that no search paths for compiling/linking must be
#   set here.
#   Only PN_HW_LDFLAGS must be extended to load the submodules when linking.
#


# Include PicoNut setup ...
include ../../../../piconut.mk




################################################################################
#                                                                              #
#   Source Files                                                               #
#                                                                              #
################################################################################


# Module name ...
#   By convention,
#   - the module name is equal to the directory name,
#   - the top-level entity has the same name, prefixed with "m_",
#   - the top-level entity is defined in a file named $(MODULE).cpp,
#   - the testbench is defined in a file named $(MODULE)_tb.cpp.
#   The following lines can be copied to new modules with very little
#   modifications. However, if required and adequate for some module, other
#   variables may be used. Any variables not starting with "PN_" and which are
#   not special variables (like DEBUG, VERBOSE, ... - see `make help`) are local
#   and can be defined freely in this file.
MODULE := $(PN_MODULE_NAME)

# Module sources that are used for simulation and for synthesis ...
# Change the sources for the multiplication and division modules here.

#TBD: Add a conditional include to the M-Extension here, like it is done with the F-Extension
MODULE_SRC := $(MODULE).cpp \
			  alu/alu.cpp \
			  alu/m_extension/mul/shift_and_add.cpp \
			  alu/m_extension/div/shift_and_subtract.cpp \
			  alu/scalar_crypto/scalar_crypto.cpp \
			  pc/pc.cpp \
			  ir/ir.cpp \
			  regfile/regfile.cpp \
			  extender/extender.cpp \
			  byteselector/byteselector.cpp \
			  controller/controller.cpp \
			  immgen/immgen.cpp \
			  datahandler/datahandler.cpp \
			  csr/csr.cpp \
			  csr_master/csr_master.cpp \

# Conditionally append the implementation files if F_EXTENSION is enabled
ifeq ($(PN_CFG_ENABLE_F_EXTENSION),1)
MODULE_SRC += f_extension/f_extension.cpp \
			  f_extension/regfile_float/regfile_float.cpp \
              f_extension/classifier_float/classifier_float.cpp \
              f_extension/adder_float/adder_float.cpp \
              f_extension/multiplier_float/multiplier_float.cpp \
              f_extension/divisor_float/divisor_float.cpp \
              f_extension/compare_float/compare_float.cpp \
              f_extension/converter_float/converter_float.cpp \
              f_extension/sqrt_float/sqrt_float.cpp \
              f_extension/mul_add_float/mul_add_float.cpp
endif


# Testbench and its source(s), containing `sc_main` ...
TESTBENCH := $(MODULE)_tb/$(MODULE)_tb
TESTBENCH_SRC := $(TESTBENCH).cpp

TESTBENCH_ALU := alu/alu_tb/alu_tb
# Change the sources for the multiplication and division modules here.
TESTBENCH_ALU_SRC := alu/alu.cpp \
					 alu/m_extension/mul/shift_and_add.cpp \
					 alu/m_extension/div/shift_and_subtract.cpp \
					 alu/scalar_crypto/scalar_crypto.cpp \
					 alu/alu_tb/alu_tb.cpp

TESTBENCH_CRYPTO := alu/scalar_crypto/scalar_crypto_tb/scalar_crypto_tb
TESTBENCH_CRYPTO_SRC := alu/scalar_crypto/scalar_crypto.cpp \
						alu/scalar_crypto/scalar_crypto_tb/scalar_crypto_tb.cpp

TESTBENCH_PC := pc/pc_tb/pc_tb
TESTBENCH_PC_SRC := pc/pc.cpp pc/pc_tb/pc_tb.cpp

TESTBENCH_IR := ir/ir_tb/ir_tb
TESTBENCH_IR_SRC := ir/ir.cpp ir/ir_tb/ir_tb.cpp

TESTBENCH_REGFILE := regfile/regfile_tb/regfile_tb
TESTBENCH_REGFILE_SRC := regfile/regfile.cpp regfile/regfile_tb/regfile_tb.cpp

TESTBENCH_EXTENDER := extender/extender_tb/extender_tb
TESTBENCH_EXTENDER_SRC := extender/extender.cpp \
						  extender/extender_tb/extender_tb.cpp

TESTBENCH_BYTESELECTOR := byteselector/byteselector_tb/byteselector_tb
TESTBENCH_BYTESELECTOR_SRC := byteselector/byteselector.cpp \
							  byteselector/byteselector_tb/byteselector_tb.cpp

TESTBENCH_CONTROLLER := controller/controller_tb/controller_tb
TESTBENCH_CONTROLLER_SRC := controller/controller.cpp \
							controller/controller_tb/controller_tb.cpp

TESTBENCH_IMMGEN := immgen/immgen_tb/immgen_tb
TESTBENCH_IMMGEN_SRC := immgen/immgen.cpp immgen/immgen_tb/immgen_tb.cpp

TESTBENCH_DATAHANDLER := datahandler/datahandler_tb/datahandler_tb
TESTBENCH_DATAHANDLER_SRC := datahandler/datahandler.cpp \
							 datahandler/datahandler_tb/datahandler_tb.cpp

TESTBENCH_CSR := csr/csr_tb/csr_tb
TESTBENCH_CSR_SRC := csr/csr.cpp csr/csr_tb/csr_tb.cpp

TESTBENCH_CSR_MASTER := csr_master/csr_master_tb/csr_master_tb
TESTBENCH_CSR_MASTER_SRC := csr_master/csr_master.cpp \
							csr_master/csr_master_tb/csr_master_tb.cpp




################################################################################
#                                                                              #
#   Simulation                                                                 #
#                                                                              #
################################################################################


# Define set of object files and output binaries ...
#   As many hardware modules, this section builds
#   a) a shared library with the simulation model of the module ('MODULE_*'),
#   b) a simulation binary based on a testbench using this library ('TESTBENCH_*').
MODULE_OBJ := $(MODULE_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
MODULE_LIB := $(PN_MODULE_BUILD_DIR)/sim/lib$(MODULE).a

TESTBENCH_OBJ := $(TESTBENCH_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
TESTBENCH_BIN := $(PN_MODULE_BUILD_DIR)/sim/$(TESTBENCH)

TESTBENCH_ALU_OBJ := $(TESTBENCH_ALU_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
TESTBENCH_ALU_BIN := $(PN_MODULE_BUILD_DIR)/sim/$(TESTBENCH_ALU)

TESTBENCH_CRYPTO_OBJ := $(TESTBENCH_CRYPTO_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
TESTBENCH_CRYPTO_BIN := $(PN_MODULE_BUILD_DIR)/sim/$(TESTBENCH_CRYPTO)

TESTBENCH_PC_OBJ := $(TESTBENCH_PC_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
TESTBENCH_PC_BIN := $(PN_MODULE_BUILD_DIR)/sim/$(TESTBENCH_PC)

TESTBENCH_IR_OBJ := $(TESTBENCH_IR_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
TESTBENCH_IR_BIN := $(PN_MODULE_BUILD_DIR)/sim/$(TESTBENCH_IR)

TESTBENCH_REGFILE_OBJ := $(TESTBENCH_REGFILE_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
TESTBENCH_REGFILE_BIN := $(PN_MODULE_BUILD_DIR)/sim/$(TESTBENCH_REGFILE)

TESTBENCH_EXTENDER_OBJ := $(TESTBENCH_EXTENDER_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
TESTBENCH_EXTENDER_BIN := $(PN_MODULE_BUILD_DIR)/sim/$(TESTBENCH_EXTENDER)

TESTBENCH_BYTESELECTOR_OBJ := $(TESTBENCH_BYTESELECTOR_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
TESTBENCH_BYTESELECTOR_BIN := $(PN_MODULE_BUILD_DIR)/sim/$(TESTBENCH_BYTESELECTOR)

TESTBENCH_CONTROLLER_OBJ := $(TESTBENCH_CONTROLLER_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
TESTBENCH_CONTROLLER_BIN := $(PN_MODULE_BUILD_DIR)/sim/$(TESTBENCH_CONTROLLER)

TESTBENCH_IMMGEN_OBJ := $(TESTBENCH_IMMGEN_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
TESTBENCH_IMMGEN_BIN := $(PN_MODULE_BUILD_DIR)/sim/$(TESTBENCH_IMMGEN)

TESTBENCH_DATAHANDLER_OBJ := $(TESTBENCH_DATAHANDLER_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
TESTBENCH_DATAHANDLER_BIN := $(PN_MODULE_BUILD_DIR)/sim/$(TESTBENCH_DATAHANDLER)

TESTBENCH_CSR_OBJ := $(TESTBENCH_CSR_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
TESTBENCH_CSR_BIN := $(PN_MODULE_BUILD_DIR)/sim/$(TESTBENCH_CSR)

TESTBENCH_CSR_MASTER_OBJ := $(TESTBENCH_CSR_MASTER_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
TESTBENCH_CSR_MASTER_BIN := $(PN_MODULE_BUILD_DIR)/sim/$(TESTBENCH_CSR_MASTER)

# Automatic dependencies ...
#   The default compile command $(PN_HW_CC) generates a .d file for each .o file
#   containing the .h files it depends on. These files are included here.
-include $(MODULE_OBJ:%.o=%.d)
-include $(TESTBENCH_OBJ:%.o=%.d)


# Set object files to link ...
#   The PicoNut setup script ('piconut.mk') contains predefined (implicit) rules
#   for compiling and linking. If they are used (as in this file), an additional
#   linking rule must be supplied here to identify all object files the binary
#   depends on. The recipe must be left out, it is supplied by 'piconut.mk'.
#


#   dependencies of the library ...
$(MODULE_LIB): $(MODULE_OBJ)

#   dependencies of the executable ...
$(TESTBENCH_BIN): $(TESTBENCH_OBJ) $(MODULE_LIB)


$(TESTBENCH_ALU_BIN): $(TESTBENCH_ALU_OBJ) $(MODULE_LIB)

TESTBENCH_ALU_RUN := verify-$(TESTBENCH_ALU_BIN)

.PHONY: $(TESTBENCH_ALU_RUN)
$(TESTBENCH_ALU_RUN): $(TESTBENCH_ALU_BIN)
	$(PN_HW_RUN) $<

$(TESTBENCH_CRYPTO_BIN): $(TESTBENCH_CRYPTO_OBJ) $(MODULE_LIB)

TESTBENCH_CRYPTO_RUN := verify-crypto

.PHONY: $(TESTBENCH_CRYPTO_RUN)
$(TESTBENCH_CRYPTO_RUN): $(TESTBENCH_CRYPTO_BIN)
	$(PN_HW_RUN) $<


$(TESTBENCH_PC_BIN): $(TESTBENCH_PC_OBJ) $(MODULE_LIB)

TESTBENCH_PC_RUN := verify-$(TESTBENCH_PC_BIN)

.PHONY: $(TESTBENCH_PC_RUN)
$(TESTBENCH_PC_RUN): $(TESTBENCH_PC_BIN)
	$(PN_HW_RUN) $<


$(TESTBENCH_IR_BIN): $(TESTBENCH_IR_OBJ) $(MODULE_LIB)

TESTBENCH_IR_RUN := verify-$(TESTBENCH_IR_BIN)

.PHONY: $(TESTBENCH_IR_RUN)
$(TESTBENCH_IR_RUN): $(TESTBENCH_IR_BIN)
	$(PN_HW_RUN) $<


$(TESTBENCH_REGFILE_BIN): $(TESTBENCH_REGFILE_OBJ) $(MODULE_LIB)

TESTBENCH_REGFILE_RUN := verify-$(TESTBENCH_REGFILE_BIN)

.PHONY: $(TESTBENCH_REGFILE_RUN)
$(TESTBENCH_REGFILE_RUN): $(TESTBENCH_REGFILE_BIN)
	$(PN_HW_RUN) $<


$(TESTBENCH_EXTENDER_BIN): $(TESTBENCH_EXTENDER_OBJ) $(MODULE_LIB)

TESTBENCH_EXTENDER_RUN := verify-$(TESTBENCH_EXTENDER_BIN)

.PHONY: $(TESTBENCH_EXTENDER_RUN)
$(TESTBENCH_EXTENDER_RUN): $(TESTBENCH_EXTENDER_BIN)
	$(PN_HW_RUN) $<


$(TESTBENCH_BYTESELECTOR_BIN): $(TESTBENCH_BYTESELECTOR_OBJ) $(MODULE_LIB)

TESTBENCH_BYTESELECTOR_RUN := verify-$(TESTBENCH_BYTESELECTOR_BIN)

.PHONY: $(TESTBENCH_BYTESELECTOR_RUN)
$(TESTBENCH_BYTESELECTOR_RUN): $(TESTBENCH_BYTESELECTOR_BIN)
	$(PN_HW_RUN) $<


$(TESTBENCH_CONTROLLER_BIN): $(TESTBENCH_CONTROLLER_OBJ) $(MODULE_LIB)

TESTBENCH_CONTROLLER_RUN := verify-$(TESTBENCH_CONTROLLER_BIN)

.PHONY: $(TESTBENCH_CONTROLLER_RUN)
$(TESTBENCH_CONTROLLER_RUN): $(TESTBENCH_CONTROLLER_BIN)
	$(PN_HW_RUN) $<


$(TESTBENCH_IMMGEN_BIN): $(TESTBENCH_IMMGEN_OBJ) $(MODULE_LIB)

TESTBENCH_IMMGEN_RUN := verify-$(TESTBENCH_IMMGEN_BIN)

.PHONY: $(TESTBENCH_IMMGEN_RUN)
$(TESTBENCH_IMMGEN_RUN): $(TESTBENCH_IMMGEN_BIN)
	$(PN_HW_RUN) $<


$(TESTBENCH_DATAHANDLER_BIN): $(TESTBENCH_DATAHANDLER_OBJ) $(MODULE_LIB)

TESTBENCH_DATAHANDLER_RUN := verify-$(TESTBENCH_DATAHANDLER_BIN)

.PHONY: $(TESTBENCH_DATAHANDLER_RUN)
$(TESTBENCH_DATAHANDLER_RUN): $(TESTBENCH_DATAHANDLER_BIN)
	$(PN_HW_RUN) $<


$(TESTBENCH_CSR_BIN): $(TESTBENCH_CSR_OBJ) $(MODULE_LIB)

TESTBENCH_CSR_RUN := verify-$(TESTBENCH_CSR_BIN)

.PHONY: $(TESTBENCH_CSR_RUN)
$(TESTBENCH_CSR_RUN): $(TESTBENCH_CSR_BIN)
	$(PN_HW_RUN) $<


$(TESTBENCH_CSR_MASTER_BIN): $(TESTBENCH_CSR_MASTER_OBJ) $(MODULE_LIB)

TESTBENCH_CSR_MASTER_RUN := verify-$(TESTBENCH_CSR_MASTER_BIN)

.PHONY: $(TESTBENCH_CSR_MASTER_RUN)
$(TESTBENCH_CSR_MASTER_RUN): $(TESTBENCH_CSR_MASTER_BIN)
	$(PN_HW_RUN) $<



#################### Exported global targets ###################################


# Build ...
.PHONY: build-lib
build-lib: $(MODULE_LIB)

.PHONY: build-tb
build-tb: $(TESTBENCH_BIN)

build-sim: build-lib build-tb


# Verify ...
#   Run the main testbench.
.PHONY: verify-sim
verify-sim: $(TESTBENCH_BIN) $(TESTBENCH_ALU_RUN) $(TESTBENCH_PC_RUN) $(TESTBENCH_IR_RUN) $(TESTBENCH_REGFILE_RUN) $(TESTBENCH_EXTENDER_RUN) $(TESTBENCH_BYTESELECTOR_RUN) $(TESTBENCH_CONTROLLER_RUN) $(TESTBENCH_BYTESELECTOR_RUN) $(TESTBENCH_CONTROLLER_RUN) $(TESTBENCH_IMMGEN_RUN) $(TESTBENCH_DATAHANDLER_RUN) $(TESTBENCH_CSR_RUN) $(TESTBENCH_CSR_MASTER_RUN)
	$(PN_HW_RUN) $<

# Install ...
.PHONY: install-all
install-all:
	$(PN_INSTALL_HW_INCLUDE) $(MODULE).h

.PHONY: install-sim
install-sim:
	$(PN_INSTALL_HW_LIB) $(MODULE_LIB)





################################################################################
#                                                                              #
#   Synthesis                                                                  #
#                                                                              #
################################################################################





#################### Phase 1: SystemC synthesis (build-systemc) ####################


# The following definitions direct the synthesis by the Intel Compiler for
# SystemC (ICSC). ICSC comes with its own build system an C++ compiler, and it
# may handle a complete tree of modules at once.


# Module sources for ICSC ...
#   Module source files including the testbench. Sources exported by an
#   'icsc-sources' recipe must be left out.
PN_SYSTEMC_SRC := $(MODULE_SRC) $(TESTBENCH_SRC)


# Submodules for ICSC ...
#   Define the submodules that ICSC must process together with this one.
PN_SYSTEMC_SUBMODULES := $(PN_SUBMODULES)





#################### Phase 2: Gate-level synthesis (build-netlist) #############

# The following rules define a *reusable module*: Local synthesis is performed
# up to a gate-level, board-specific netlist, and this netlist is installed by
# the 'install-syn' target. The 'verify-syn' target (only) checks if the netlist
# can be built successfully.
#
# For a *simple module*, which is synthesized together with its super-module,
# the 'build-syn' and 'install-syn' targets can be left empty to save overall
# build time. The 'verify-syn' target, however, should be the same as for a
# reusable module to check for synthesis issues due to this current module.


# Synthesis for netlist ...
#   Name all RTL files (Verilog, VDHL) and presynthesized modules to be passed
#   to the synthesis tool. In most cases, (only) the SystemC synthesis result
#   (PN_SYSTEMC_OUTPUT) will be passed here.
PN_NETLIST_SRC := $(PN_SYSTEMC_OUTPUT)


# Declare dependencies ...
#   Declare the source files on which the netlist generation step depends.
#   Usually, the following line can be left untouched.
$(PN_NETLIST_OUTPUT): $(PN_NETLIST_SRC)





#################### Exported global targets ###################################


# Build ...
#   Synthesize only if a synthesizable Membrana is selected.
ifneq (membrana_soft,$(PN_CFG_MEMBRANA))
build-syn: build-netlist
endif


# Verify ...
ifneq (membrana_soft,$(PN_CFG_MEMBRANA))
verify-syn: build-netlist
endif


# Install ...
install-syn: build-syn
ifneq (membrana_soft,$(PN_CFG_MEMBRANA))
	$(PN_INSTALL_HW_SYN) $(PN_SYSTEMC_OUTPUT) $(PN_NETLIST_OUTPUT)
endif
