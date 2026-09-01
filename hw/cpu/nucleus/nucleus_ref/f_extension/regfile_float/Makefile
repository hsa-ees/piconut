include ../../../../../../piconut.mk

MODULE := $(PN_MODULE_NAME)
MODULE_SRC := $(MODULE).cpp

TESTBENCH := $(MODULE)_tb/$(MODULE)_tb
TESTBENCH_SRC := $(TESTBENCH).cpp

MODULE_OBJ := $(MODULE_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
MODULE_LIB := $(PN_MODULE_BUILD_DIR)/sim/lib$(MODULE).a

TESTBENCH_OBJ := $(TESTBENCH_SRC:%.cpp=$(PN_MODULE_BUILD_DIR)/sim/%.o)
TESTBENCH_BIN := $(PN_MODULE_BUILD_DIR)/sim/$(TESTBENCH)

-include $(MODULE_OBJ:%.o=%.d)
-include $(TESTBENCH_OBJ:%.o=%.d)

$(MODULE_LIB): $(MODULE_OBJ)

$(TESTBENCH_BIN): $(TESTBENCH_OBJ) $(MODULE_LIB)

# Build ...
.PHONY: build-lib
build-lib: $(MODULE_LIB)

.PHONY: build-tb
build-tb: $(TESTBENCH_BIN)

build-sim: build-lib build-tb

.PHONY: verify-sim
verify-sim: $(TESTBENCH_BIN)
	$(PN_HW_RUN) $<

.PHONY: install-all
install-all:
	$(PN_INSTALL_HW_INCLUDE) $(MODULE).h

.PHONY: install-sim
install-sim:
	$(PN_INSTALL_HW_LIB) $(MODULE_LIB)