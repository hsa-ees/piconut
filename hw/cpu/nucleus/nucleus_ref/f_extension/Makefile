PN_CFG_ENABLE_F_EXTENSION = 1

include ../../../../../piconut.mk

PN_TEST_DIR = $(PN_MODULE_BUILD_DIR)/tests
PN_HW_CFLAGS += -DPN_TEST_DIR=\"$(PN_TEST_DIR)\"

MODULE := $(PN_MODULE_NAME)
MODULE_SRC := $(MODULE).cpp \
			  regfile_float/regfile_float.cpp \
			  classifier_float/classifier_float.cpp \
			  adder_float/adder_float.cpp \
			  multiplier_float/multiplier_float.cpp \
			  divisor_float/divisor_float.cpp \
			  compare_float/compare_float.cpp \
			  converter_float/converter_float.cpp \
			  sqrt_float/sqrt_float.cpp \
			  mul_add_float/mul_add_float.cpp

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

.PHONY: build-tests
build-tests:
	mkdir -p $(PN_TEST_DIR)
	testfloat_gen f32_add -rnear_maxMag > $(PN_TEST_DIR)/f32_add_rnear_maxMag_tests.txt
	testfloat_gen f32_add -rnear_even >   $(PN_TEST_DIR)/f32_add_rnear_even_tests.txt
	testfloat_gen f32_add -rminMag >      $(PN_TEST_DIR)/f32_add_rminMag_tests.txt
	testfloat_gen f32_add -rmin >         $(PN_TEST_DIR)/f32_add_rmin_tests.txt
	testfloat_gen f32_add -rmax >         $(PN_TEST_DIR)/f32_add_rmax_tests.txt

	testfloat_gen f32_div -rnear_maxMag > $(PN_TEST_DIR)/f32_div_rnear_maxMag_tests.txt
	testfloat_gen f32_div -rnear_even >   $(PN_TEST_DIR)/f32_div_rnear_even_tests.txt
	testfloat_gen f32_div -rminMag >      $(PN_TEST_DIR)/f32_div_rminMag_tests.txt
	testfloat_gen f32_div -rmin >         $(PN_TEST_DIR)/f32_div_rmin_tests.txt
	testfloat_gen f32_div -rmax >         $(PN_TEST_DIR)/f32_div_rmax_tests.txt

	testfloat_gen f32_mul -rnear_maxMag > $(PN_TEST_DIR)/f32_mul_rnear_maxMag_tests.txt
	testfloat_gen f32_mul -rnear_even >   $(PN_TEST_DIR)/f32_mul_rnear_even_tests.txt
	testfloat_gen f32_mul -rminMag >      $(PN_TEST_DIR)/f32_mul_rminMag_tests.txt
	testfloat_gen f32_mul -rmin >         $(PN_TEST_DIR)/f32_mul_rmin_tests.txt
	testfloat_gen f32_mul -rmax >         $(PN_TEST_DIR)/f32_mul_rmax_tests.txt

	testfloat_gen f32_mulAdd -rnear_maxMag > $(PN_TEST_DIR)/f32_mulAdd_rnear_maxMag_tests.txt
	testfloat_gen f32_mulAdd -rnear_even >   $(PN_TEST_DIR)/f32_mulAdd_rnear_even_tests.txt
	testfloat_gen f32_mulAdd -rminMag >      $(PN_TEST_DIR)/f32_mulAdd_rminMag_tests.txt
	testfloat_gen f32_mulAdd -rmin >         $(PN_TEST_DIR)/f32_mulAdd_rmin_tests.txt
	testfloat_gen f32_mulAdd -rmax >         $(PN_TEST_DIR)/f32_mulAdd_rmax_tests.txt

	testfloat_gen f32_to_i32 -rnear_maxMag > $(PN_TEST_DIR)/f32_to_i32_rnear_maxMag_tests.txt
	testfloat_gen f32_to_i32 -rnear_even >   $(PN_TEST_DIR)/f32_to_i32_rnear_even_tests.txt
	testfloat_gen f32_to_i32 -rminMag >      $(PN_TEST_DIR)/f32_to_i32_rminMag_tests.txt
	testfloat_gen f32_to_i32 -rmin >         $(PN_TEST_DIR)/f32_to_i32_rmin_tests.txt
	testfloat_gen f32_to_i32 -rmax >         $(PN_TEST_DIR)/f32_to_i32_rmax_tests.txt

	testfloat_gen f32_to_ui32 -rnear_maxMag > $(PN_TEST_DIR)/f32_to_ui32_rnear_maxMag_tests.txt
	testfloat_gen f32_to_ui32 -rnear_even >   $(PN_TEST_DIR)/f32_to_ui32_rnear_even_tests.txt
	testfloat_gen f32_to_ui32 -rminMag >      $(PN_TEST_DIR)/f32_to_ui32_rminMag_tests.txt
	testfloat_gen f32_to_ui32 -rmin >         $(PN_TEST_DIR)/f32_to_ui32_rmin_tests.txt
	testfloat_gen f32_to_ui32 -rmax >         $(PN_TEST_DIR)/f32_to_ui32_rmax_tests.txt

	testfloat_gen i32_to_f32 -rnear_maxMag > $(PN_TEST_DIR)/i32_to_f32_rnear_maxMag_tests.txt
	testfloat_gen i32_to_f32 -rnear_even >   $(PN_TEST_DIR)/i32_to_f32_rnear_even_tests.txt
	testfloat_gen i32_to_f32 -rminMag >      $(PN_TEST_DIR)/i32_to_f32_rminMag_tests.txt
	testfloat_gen i32_to_f32 -rmin >         $(PN_TEST_DIR)/i32_to_f32_rmin_tests.txt
	testfloat_gen i32_to_f32 -rmax >         $(PN_TEST_DIR)/i32_to_f32_rmax_tests.txt

	testfloat_gen ui32_to_f32 -rnear_maxMag > $(PN_TEST_DIR)/ui32_to_f32_rnear_maxMag_tests.txt
	testfloat_gen ui32_to_f32 -rnear_even >   $(PN_TEST_DIR)/ui32_to_f32_rnear_even_tests.txt
	testfloat_gen ui32_to_f32 -rminMag >      $(PN_TEST_DIR)/ui32_to_f32_rminMag_tests.txt
	testfloat_gen ui32_to_f32 -rmin >         $(PN_TEST_DIR)/ui32_to_f32_rmin_tests.txt
	testfloat_gen ui32_to_f32 -rmax >         $(PN_TEST_DIR)/ui32_to_f32_rmax_tests.txt