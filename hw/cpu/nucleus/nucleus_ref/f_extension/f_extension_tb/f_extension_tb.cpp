/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2026 Daniel Sommerfeldt <daniel.sommerfeldt1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg


  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation and/or
     other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *************************************************************************/

#include "../f_extension.h"

#include <systemc.h>
#include <piconut.h>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

sc_signal<sc_uint<32>> PN_NAME(data_in);
sc_signal<sc_uint<5>> PN_NAME(select_in);
sc_signal<sc_uint<5>> PN_NAME(rs1_select_in);
sc_signal<sc_uint<5>> PN_NAME(rs2_select_in);
sc_signal<sc_uint<5>> PN_NAME(rs3_select_in);
sc_signal<bool> PN_NAME(en_load_in);

sc_signal<sc_uint<32>> PN_NAME(rs2_out);

sc_signal<bool> PN_NAME(stb_in);
sc_signal<sc_uint<32>> PN_NAME(instruction_in);
sc_signal<sc_uint<3>> PN_NAME(rounding_mode_in);

sc_signal<bool> PN_NAME(ready_out);
sc_signal<sc_uint<32>> PN_NAME(f_out);

void run_cycle(int cycles = 1) {
    for(int i = 0; i < cycles; i++) {
        clk = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

void load_reg(int value, int reg) {
    en_load_in = 1;
    data_in = value;
    select_in = reg;
    run_cycle();
    en_load_in = 0;
}

unsigned int extract_bits(unsigned int value, int start, int length) {
    unsigned int mask = (1U << length) - 1;

    return (value >> start) & mask;
}

unsigned int run_instruction(int instruction) {
    instruction_in = instruction;
    rs1_select_in = extract_bits(instruction, 15, 5);
    rs2_select_in = extract_bits(instruction, 20, 5);
    rs3_select_in = extract_bits(instruction, 27, 5);
    stb_in = 1;
    while(ready_out.read() == 1) {
        run_cycle();
    }
    stb_in = 0;
    while(ready_out.read() == 0) {
        run_cycle();
    }
    return f_out.read();
}

void classify_tests() {
    PN_INFOF(("STARTING CLASSIFY TESTS"));
    PN_INFOF(("---"));
    // Load float classes
    load_reg(0b1'11111111'00000000000000000000000, 0); // Negative Infinity
    load_reg(0b1'01111111'00000000000000000000000, 1); // Negative Normal
    load_reg(0b1'00000000'00000000000000000000001, 2); // Negative Subnormal
    load_reg(0b1'00000000'00000000000000000000000, 3); // Negative Zero
    load_reg(0b0'00000000'00000000000000000000000, 4); // Positive Zero
    load_reg(0b0'00000000'00000000000000000000001, 5); // Positive Subnormal
    load_reg(0b0'01111111'00000000000000000000000, 6); // Positive Normal
    load_reg(0b0'11111111'00000000000000000000000, 7); // Positive Infinity
    load_reg(0b0'11111111'00000000000000000000001, 8); // Signaling NaN
    load_reg(0b0'11111111'10000000000000000000000, 9); // Quiet Nan

    unsigned int result;

    PN_INFOF(("Test 1: Negative Infinity"));
    result = run_instruction(0b11100'00'00000'00000'001'00000'1010011); // fclass.s x0, f0
    PN_INFOF(("        Expected: 1"));
    PN_INFOF(("        Result:   %d", result));
    PN_ASSERTM(result == 1, "TEST 1: Failed");
    PN_INFOF(("Test 1: Success"));

    PN_INFOF(("---"));

    PN_INFOF(("Test 2: Negative Normal"));
    result = run_instruction(0b11100'00'00000'00001'001'00000'1010011); // fclass.s x0, f1
    PN_INFOF(("        Expected: 2"));
    PN_INFOF(("        Result:   %d", result));
    PN_ASSERTM(result == 2, "TEST 2: Failed");
    PN_INFOF(("Test 2: Success"));

    PN_INFOF(("---"));

    PN_INFOF(("Test 3: Negative Subnormal"));
    result = run_instruction(0b11100'00'00000'00010'001'00000'1010011); // fclass.s x0, f2
    PN_INFOF(("        Expected: 4"));
    PN_INFOF(("        Result:   %d", result));
    PN_ASSERTM(result == 4, "TEST 3: Failed");
    PN_INFOF(("Test 3: Success"));

    PN_INFOF(("---"));

    PN_INFOF(("Test 4: Negative Zero"));
    result = run_instruction(0b11100'00'00000'00011'001'00000'1010011); // fclass.s x0, f3
    PN_INFOF(("        Expected: 8"));
    PN_INFOF(("        Result:   %d", result));
    PN_ASSERTM(result == 8, "TEST 4: Failed");
    PN_INFOF(("Test 4: Success"));

    PN_INFOF(("---"));

    PN_INFOF(("Test 5: Positive Zero"));
    result = run_instruction(0b11100'00'00000'00100'001'00000'1010011); // fclass.s x0, f4
    PN_INFOF(("        Expected: 16"));
    PN_INFOF(("        Result:   %d", result));
    PN_ASSERTM(result == 16, "TEST 5: Failed");
    PN_INFOF(("Test 5: Success"));

    PN_INFOF(("---"));

    PN_INFOF(("Test 6: Positive Subnormal"));
    result = run_instruction(0b11100'00'00000'00101'001'00000'1010011); // fclass.s x0, f5
    PN_INFOF(("        Expected: 32"));
    PN_INFOF(("        Result:   %d", result));
    PN_ASSERTM(result == 32, "TEST 6: Failed");
    PN_INFOF(("Test 6: Success"));

    PN_INFOF(("---"));

    PN_INFOF(("Test 7: Positive Normal"));
    result = run_instruction(0b11100'00'00000'00110'001'00000'1010011); // fclass.s x0, f6
    PN_INFOF(("        Expected: 64"));
    PN_INFOF(("        Result:   %d", result));
    PN_ASSERTM(result == 64, "TEST 7: Failed");
    PN_INFOF(("Test 7: Success"));

    PN_INFOF(("---"));

    PN_INFOF(("Test 8: Positive Infinity"));
    result = run_instruction(0b11100'00'00000'00111'001'00000'1010011); // fclass.s x0, f7
    PN_INFOF(("        Expected: 128"));
    PN_INFOF(("        Result:   %d", result));
    PN_ASSERTM(result == 128, "TEST 8: Failed");
    PN_INFOF(("Test 8: Success"));

    PN_INFOF(("---"));

    PN_INFOF(("Test 9: Signaling NaN"));
    result = run_instruction(0b11100'00'00000'01000'001'00000'1010011); // fclass.s x0, f8
    PN_INFOF(("        Expected: 256"));
    PN_INFOF(("        Result:   %d", result));
    PN_ASSERTM(result == 256, "TEST 9: Failed");
    PN_INFOF(("Test 9: Success"));

    PN_INFOF(("---"));

    PN_INFOF(("Test 10: Quiet NaN"));
    result = run_instruction(0b11100'00'00000'01001'001'00000'1010011); // fclass.s x0, f9
    PN_INFOF(("        Expected: 512"));
    PN_INFOF(("        Result:   %d", result));
    PN_ASSERTM(result == 512, "TEST 10: Failed");
    PN_INFOF(("Test 10: Success"));
}

void berkeley_tests(std::string filename, int instruction) {
    std::ifstream test_file(filename);
    if (!test_file.is_open()) {
        PN_INFOF(("Could not open test file! Make sure it's in the execution directory."));
        return;
    }

    uint32_t a, b, expected;
    std::string flags;
    int test_count = 0;
    int pass_count = 0;

    while (test_file >> std::hex >> a >> b >> expected >> flags) {
        test_count++;
        
        load_reg(a, 0);
        load_reg(b, 1);

        uint32_t actual = run_instruction(instruction);

        if (actual == expected) {
            pass_count++;
        } else {
            PN_INFOF(("Fail @ Line %d | A: %08x B: %08x | Exp: %08x Got: %08x", 
                        test_count, a, b, expected, actual));
        }
    }

    test_file.close();

    PN_INFOF(("========================================"));
    PN_INFOF((" FINAL RESULT: %d / %d PASSED", pass_count, test_count));
    PN_INFOF(("========================================"));
}

void berkeley_single_argument_tests(std::string filename, int instruction) {
    std::ifstream test_file(filename);
    if (!test_file.is_open()) {
        PN_INFOF(("Could not open test file! Make sure it's in the execution directory."));
        return;
    }

    uint32_t a, expected;
    std::string flags;
    int test_count = 0;
    int pass_count = 0;

    while (test_file >> std::hex >> a >> expected >> flags) {
        test_count++;

        load_reg(a, 0);
        
        uint32_t actual = run_instruction(instruction);

        if (actual == expected) {
            pass_count++;
        } else {
            PN_INFOF(("Fail @ Line %d | A: %08x | Exp: %08x Got: %08x", 
                        test_count, a, expected, actual));
        }
    }

    test_file.close();

    PN_INFOF(("========================================"));
    PN_INFOF((" FINAL RESULT: %d / %d PASSED", pass_count, test_count));
    PN_INFOF(("========================================\n"));
}

void berkeley_triple_argument_tests(std::string filename, int instruction) {
    std::ifstream test_file(filename);
    if (!test_file.is_open()) {
        PN_INFOF(("Could not open test file! Make sure it's in the execution directory."));
        return;
    }

    uint32_t a, b, c, expected;
    std::string flags;
    int test_count = 0;
    int pass_count = 0;

    while (test_file >> std::hex >> a >> b >> c >> expected >> flags) {
        test_count++;
        
        load_reg(a, 0);
        load_reg(b, 1);
        load_reg(c, 2);

        uint32_t actual = run_instruction(instruction);

        if (actual == expected) {
            pass_count++;
        } else {
            PN_INFOF(("Fail @ Line %d | A: %08x B: %08x C: %08x | Exp: %08x Got: %08x", 
                        test_count, a, b, c, expected, actual));
        }

        if (test_count >= 50000) break;
    }

    test_file.close();

    PN_INFOF(("========================================"));
    PN_INFOF((" FINAL RESULT: %d / %d PASSED", pass_count, test_count));
    PN_INFOF(("========================================"));
}

void add_tests() {
    PN_INFOF(("STARTING ADD TESTS"));
    PN_INFOF(("---"));
    rounding_mode_in = 0b111;

#ifdef PN_TEST_DIR
    // Create standard string paths dynamically
    std::string test_dir = PN_TEST_DIR;
    std::string rnear_even  = test_dir + "/f32_add_rnear_even_tests.txt";
    std::string rminMag     = test_dir + "/f32_add_rminMag_tests.txt";
    std::string rmin        = test_dir + "/f32_add_rmin_tests.txt";
    std::string rmax        = test_dir + "/f32_add_rmax_tests.txt";
    std::string rnear_maxMag = test_dir + "/f32_add_rnear_maxMag_tests.txt";

    PN_INFOF(("Test 1: Round to Nearest, ties to Even"));
    PN_INFOF(("Using Tests in: %s", rnear_even.c_str()));
    berkeley_tests(rnear_even.c_str(), 0x00100153); // fadd.s f2, f0, f1, rne
    PN_INFOF(("---"));

    PN_INFOF(("Test 2: Round towards Zero"));
    PN_INFOF(("Using Tests in: %s", rminMag.c_str()));
    berkeley_tests(rminMag.c_str(), 0x00101153); // fadd.s f2, f0, f1, rtz
    PN_INFOF(("---"));

    PN_INFOF(("Test 3: Round Down"));
    PN_INFOF(("Using Tests in: %s", rmin.c_str()));
    berkeley_tests(rmin.c_str(), 0x00102153); // fadd.s f2, f0, f1, rdn
    PN_INFOF(("---"));

    PN_INFOF(("Test 4: Round Up"));
    PN_INFOF(("Using Tests in: %s", rmax.c_str()));
    berkeley_tests(rmax.c_str(), 0x00103153); // fadd.s f2, f0, f1, rup
    PN_INFOF(("---"));

    PN_INFOF(("Test 5: Round to Nearest, ties to Max Magnitude"));
    PN_INFOF(("Using Tests in: %s", rnear_maxMag.c_str()));
    berkeley_tests(rnear_maxMag.c_str(), 0x00104153); // fadd.s f2, f0, f1, rmm
    PN_INFOF(("---"));        
#else
    PN_INFOF(("ERROR: PN_TEST_DIR is not defined! Cannot run tests."));
#endif
}

void multiply_tests() {
    PN_INFOF(("STARTING MULTIPLY TESTS"));
    PN_INFOF(("---"));
    rounding_mode_in = 0b111;

#ifdef PN_TEST_DIR
    // Create standard string paths dynamically
    std::string test_dir = PN_TEST_DIR;
    std::string rnear_even   = test_dir + "/f32_mul_rnear_even_tests.txt";
    std::string rminMag      = test_dir + "/f32_mul_rminMag_tests.txt";
    std::string rmin         = test_dir + "/f32_mul_rmin_tests.txt";
    std::string rmax         = test_dir + "/f32_mul_rmax_tests.txt";
    std::string rnear_maxMag = test_dir + "/f32_mul_rnear_maxMag_tests.txt";

    PN_INFOF(("Test 1: Round to Nearest, ties to Even"));
    PN_INFOF(("Using Tests in: %s", rnear_even.c_str()));
    berkeley_tests(rnear_even.c_str(), 0x10100153); // fmul.s f2, f0, f1, rne
    PN_INFOF(("---"));

    PN_INFOF(("Test 2: Round towards Zero"));
    PN_INFOF(("Using Tests in: %s", rminMag.c_str()));
    berkeley_tests(rminMag.c_str(), 0x10101153); // fmul.s f2, f0, f1, rtz
    PN_INFOF(("---"));

    PN_INFOF(("Test 3: Round Down"));
    PN_INFOF(("Using Tests in: %s", rmin.c_str()));
    berkeley_tests(rmin.c_str(), 0x10102153); // fmul.s f2, f0, f1, rdn
    PN_INFOF(("---"));

    PN_INFOF(("Test 4: Round Up"));
    PN_INFOF(("Using Tests in: %s", rmax.c_str()));
    berkeley_tests(rmax.c_str(), 0x10103153); // fmul.s f2, f0, f1, rup
    PN_INFOF(("---"));

    PN_INFOF(("Test 5: Round to Nearest, ties to Max Magnitude"));
    PN_INFOF(("Using Tests in: %s", rnear_maxMag.c_str()));
    berkeley_tests(rnear_maxMag.c_str(), 0x10104153); // fmul.s f2, f0, f1, rmm
    PN_INFOF(("---"));        
#else
    PN_INFOF(("ERROR: PN_TEST_DIR is not defined! Cannot run tests."));
#endif
}

void divide_tests() {
    PN_INFOF(("STARTING DIVIDE TESTS"));
    PN_INFOF(("---"));
    rounding_mode_in = 0b111;

#ifdef PN_TEST_DIR
    // Create standard string paths dynamically
    std::string test_dir = PN_TEST_DIR;
    std::string rnear_even   = test_dir + "/f32_div_rnear_even_tests.txt";
    std::string rminMag      = test_dir + "/f32_div_rminMag_tests.txt";
    std::string rmin         = test_dir + "/f32_div_rmin_tests.txt";
    std::string rmax         = test_dir + "/f32_div_rmax_tests.txt";
    std::string rnear_maxMag = test_dir + "/f32_div_rnear_maxMag_tests.txt";

    PN_INFOF(("Test 1: Round to Nearest, ties to Even"));
    PN_INFOF(("Using Tests in: %s", rnear_even.c_str()));
    berkeley_tests(rnear_even.c_str(), 0x18100153); // fdiv.s f2, f0, f1, rne
    PN_INFOF(("---"));

    PN_INFOF(("Test 2: Round towards Zero"));
    PN_INFOF(("Using Tests in: %s", rminMag.c_str()));
    berkeley_tests(rminMag.c_str(), 0x18101153); // fdiv.s f2, f0, f1, rtz
    PN_INFOF(("---"));

    PN_INFOF(("Test 3: Round Down"));
    PN_INFOF(("Using Tests in: %s", rmin.c_str()));
    berkeley_tests(rmin.c_str(), 0x18102153); // fdiv.s f2, f0, f1, rdn
    PN_INFOF(("---"));

    PN_INFOF(("Test 4: Round Up"));
    PN_INFOF(("Using Tests in: %s", rmax.c_str()));
    berkeley_tests(rmax.c_str(), 0x18103153); // fdiv.s f2, f0, f1, rup
    PN_INFOF(("---"));

    PN_INFOF(("Test 5: Round to Nearest, ties to Max Magnitude"));
    PN_INFOF(("Using Tests in: %s", rnear_maxMag.c_str()));
    berkeley_tests(rnear_maxMag.c_str(), 0x18104153); // fdiv.s f2, f0, f1, rmm
    PN_INFOF(("---"));        
#else
    PN_INFOF(("ERROR: PN_TEST_DIR is not defined! Cannot run tests."));
#endif
}

void compare_tests() {
    PN_INFOF(("STARTING COMPARE TESTS"));
    PN_INFOF(("---"));
    
    load_reg(0.0f, 0);
    load_reg(2.0f, 2);
    load_reg(3.0f, 3);

    unsigned int result;

    PN_INFOF(("Test 1: 2 is less than 3"));
    result = run_instruction(0xa03110d3); // flt.s x1, f2, f3
    PN_INFOF(("        Expected: 1"));
    PN_INFOF(("        Result:   %d", result));
    PN_ASSERTM(result == 1, "Test 1: Failed");
    PN_INFOF(("Test 1: Success"));

    PN_INFO("Test 2: 3 is NOT less than 2");
    result = run_instruction(0xa02190d3); // flt.s x1, f3, f2
    PN_INFOF(("        Expected: 0"));
    PN_INFOF(("        Result:   %d", result));
    PN_ASSERTM(result == 0, "Test 2: Failed");
    PN_INFOF(("Test 2: Success"));

    PN_INFO("Test 3: 3 is NOT less than 3");
    result = run_instruction(0xa03190d3); // flt.s x1, f3, f3
    PN_INFOF(("        Expected: 0"));
    PN_INFOF(("        Result:   %d", result));
    PN_ASSERTM(result == 0, "Test 3: Failed");
    PN_INFOF(("Test 3: Success"));
}

void convert_tests() {
    PN_INFOF(("STARTING CONVERT TESTS"));
    PN_INFOF(("---"));
    rounding_mode_in = 0b111;

#ifdef PN_TEST_DIR
    // Create standard string paths dynamically
    std::string test_dir = PN_TEST_DIR;

    // F32 to UI32 Paths
    std::string f32_to_ui32_rne = test_dir + "/f32_to_ui32_rnear_even_tests.txt";
    std::string f32_to_ui32_rtz = test_dir + "/f32_to_ui32_rminMag_tests.txt";
    std::string f32_to_ui32_rdn = test_dir + "/f32_to_ui32_rmin_tests.txt";
    std::string f32_to_ui32_rup = test_dir + "/f32_to_ui32_rmax_tests.txt";
    std::string f32_to_ui32_rmm = test_dir + "/f32_to_ui32_rnear_maxMag_tests.txt";

    // F32 to I32 Paths
    std::string f32_to_i32_rne  = test_dir + "/f32_to_i32_rnear_even_tests.txt";
    std::string f32_to_i32_rtz  = test_dir + "/f32_to_i32_rminMag_tests.txt";
    std::string f32_to_i32_rdn  = test_dir + "/f32_to_i32_rmin_tests.txt";
    std::string f32_to_i32_rup  = test_dir + "/f32_to_i32_rmax_tests.txt";
    std::string f32_to_i32_rmm  = test_dir + "/f32_to_i32_rnear_maxMag_tests.txt";

    // UI32 to F32 Paths
    std::string ui32_to_f32_rne = test_dir + "/ui32_to_f32_rnear_even_tests.txt";
    std::string ui32_to_f32_rtz = test_dir + "/ui32_to_f32_rminMag_tests.txt";
    std::string ui32_to_f32_rdn = test_dir + "/ui32_to_f32_rmin_tests.txt";
    std::string ui32_to_f32_rup = test_dir + "/ui32_to_f32_rmax_tests.txt";
    std::string ui32_to_f32_rmm = test_dir + "/ui32_to_f32_rnear_maxMag_tests.txt";

    // I32 to F32 Paths
    std::string i32_to_f32_rne  = test_dir + "/i32_to_f32_rnear_even_tests.txt";
    std::string i32_to_f32_rtz  = test_dir + "/i32_to_f32_rminMag_tests.txt";
    std::string i32_to_f32_rdn  = test_dir + "/i32_to_f32_rmin_tests.txt";
    std::string i32_to_f32_rup  = test_dir + "/i32_to_f32_rmax_tests.txt";
    std::string i32_to_f32_rmm  = test_dir + "/i32_to_f32_rnear_maxMag_tests.txt";

    // ==========================================
    // 1. F32 to UI32
    // ==========================================
    PN_INFOF(("*****Testing F32 to UI32, Round to Nearest, ties to Even*****"));
    berkeley_single_argument_tests(f32_to_ui32_rne.c_str(), 0xc01000d3); // fcvt.wu.s x1, f0, rne

    PN_INFOF(("*****Testing F32 to UI32, Round towards Zero*****"));
    berkeley_single_argument_tests(f32_to_ui32_rtz.c_str(), 0xc01010d3); // fcvt.wu.s x1, f0, rtz

    PN_INFOF(("*****Testing F32 to UI32, Round Down*****"));
    berkeley_single_argument_tests(f32_to_ui32_rdn.c_str(), 0xc01020d3); // fcvt.wu.s x1, f0, rdn

    PN_INFOF(("*****Testing F32 to UI32, Round Up*****"));
    berkeley_single_argument_tests(f32_to_ui32_rup.c_str(), 0xc01030d3); // fcvt.wu.s x1, f0, rup

    PN_INFOF(("*****Testing F32 to UI32, Round to Nearest, ties to Max Magnitude*****"));
    berkeley_single_argument_tests(f32_to_ui32_rmm.c_str(), 0xc01040d3); // fcvt.wu.s x1, f0, rmm

    // ==========================================
    // 2. F32 to I32
    // ==========================================
    PN_INFOF(("*****Testing F32 to I32, Round to Nearest, ties to Even*****"));
    berkeley_single_argument_tests(f32_to_i32_rne.c_str(), 0xc00000d3); // fcvt.w.s x1, f0, rne

    PN_INFOF(("*****Testing F32 to I32, Round towards Zero*****"));
    berkeley_single_argument_tests(f32_to_i32_rtz.c_str(), 0xc00010d3); // fcvt.w.s x1, f0, rtz

    PN_INFOF(("*****Testing F32 to I32, Round Down*****"));
    berkeley_single_argument_tests(f32_to_i32_rdn.c_str(), 0xc00020d3); // fcvt.w.s x1, f0, rdn

    PN_INFOF(("*****Testing F32 to I32, Round Up*****"));
    berkeley_single_argument_tests(f32_to_i32_rup.c_str(), 0xc00030d3); // fcvt.w.s x1, f0, rup

    PN_INFOF(("*****Testing F32 to I32, Round to Nearest, ties to Max Magnitude*****"));
    berkeley_single_argument_tests(f32_to_i32_rmm.c_str(), 0xc00040d3); // fcvt.w.s x1, f0, rmm

    // ==========================================
    // 3. UI32 to F32
    // ==========================================
    PN_INFOF(("*****Testing UI32 to F32, Round to Nearest, ties to Even*****"));
    berkeley_single_argument_tests(ui32_to_f32_rne.c_str(), 0xd0108053); // fcvt.s.wu f0, x1, rne

    PN_INFOF(("*****Testing UI32 to F32, Round towards Zero*****"));
    berkeley_single_argument_tests(ui32_to_f32_rtz.c_str(), 0xd0109053); // fcvt.s.wu f0, x1, rtz

    PN_INFOF(("*****Testing UI32 to F32, Round Down*****"));
    berkeley_single_argument_tests(ui32_to_f32_rdn.c_str(), 0xd010a053); // fcvt.s.wu f0, x1, rdn

    PN_INFOF(("*****Testing UI32 to F32, Round Up*****"));
    berkeley_single_argument_tests(ui32_to_f32_rup.c_str(), 0xd010b053); // fcvt.s.wu f0, x1, rup

    PN_INFOF(("*****Testing UI32 to F32, Round to Nearest, ties to Max Magnitude*****"));
    berkeley_single_argument_tests(ui32_to_f32_rmm.c_str(), 0xd010c053); // fcvt.s.wu f0, x1, rmm

    // ==========================================
    // 4. I32 to F32
    // ==========================================
    PN_INFOF(("*****Testing I32 to F32, Round to Nearest, ties to Even*****"));
    berkeley_single_argument_tests(i32_to_f32_rne.c_str(), 0xd0008053); // fcvt.s.w f0, x1, rne

    PN_INFOF(("*****Testing I32 to F32, Round towards Zero*****"));
    berkeley_single_argument_tests(i32_to_f32_rtz.c_str(), 0xd0009053); // fcvt.s.w f0, x1, rtz

    PN_INFOF(("*****Testing I32 to F32, Round Down*****"));
    berkeley_single_argument_tests(i32_to_f32_rdn.c_str(), 0xd000a053); // fcvt.s.w f0, x1, rdn

    PN_INFOF(("*****Testing I32 to F32, Round Up*****"));
    berkeley_single_argument_tests(i32_to_f32_rup.c_str(), 0xd000b053); // fcvt.s.w f0, x1, rup

    PN_INFOF(("*****Testing I32 to F32, Round to Nearest, ties to Max Magnitude*****"));
    berkeley_single_argument_tests(i32_to_f32_rmm.c_str(), 0xd000c053); // fcvt.s.w f0, x1, rmm
#else
    PN_INFOF(("ERROR: PN_TEST_DIR is not defined! Cannot run tests."));
#endif
}

void sign_inject_tests() {
    PN_INFOF(("STARTING SIGN INJECT TESTS"));
    PN_INFOF(("---"));

    load_reg(0x3f800000, 0); //  1.0
    load_reg(0xbf800000, 1); // -1.0

    unsigned int result;

    
    PN_INFO("Test 1: FSGNJ");
    result = run_instruction(0x20100153); // fsgnj.s f2, f0, f1
    PN_INFOF(("        Expected: 0xbf800000"));
    PN_INFOF(("        Result:   0x%x", result));
    PN_ASSERTM(result == 0xbf800000, "Test 1: Failed");
    PN_INFOF(("Test 1: Success"));

    PN_INFO("Test 2: FSGNJN");
    result = run_instruction(0x20101153); // fsgnjn.s f2, f0, f1
    PN_INFOF(("        Expected: 0x3f800000"));
    PN_INFOF(("        Result:   0x%x", result));
    PN_ASSERTM(result == 0x3f800000, "Test 2: Failed");
    PN_INFOF(("Test 2: Success"));

    PN_INFO("Test 3: FSGNJX");
    result = run_instruction(0x20102153); // fsgnjx.s f2, f0, f1
    PN_INFOF(("        Expected: 0xbf800000"));
    PN_INFOF(("        Result:   0x%x", result));
    PN_ASSERTM(result == 0xbf800000, "Test 3: Failed");
    PN_INFOF(("TEST 3: Success"));
}

void sqrt_tests() {
    PN_INFOF(("STARTING SQRT TESTS"));
    PN_INFOF(("---"));
    rounding_mode_in = 0b111;

#ifdef PN_TEST_DIR
    // Create standard string paths dynamically
    std::string test_dir = PN_TEST_DIR;
    std::string rnear_even   = test_dir + "/f32_sqrt_rnear_even_tests.txt";
    std::string rminMag      = test_dir + "/f32_sqrt_rminMag_tests.txt";
    std::string rmin         = test_dir + "/f32_sqrt_rmin_tests.txt";
    std::string rmax         = test_dir + "/f32_sqrt_rmax_tests.txt";
    std::string rnear_maxMag = test_dir + "/f32_sqrt_rnear_maxMag_tests.txt";

    PN_INFOF(("Test 1: Round to Nearest, ties to Even"));
    PN_INFOF(("Using Tests in: %s", rnear_even.c_str()));
    berkeley_single_argument_tests(rnear_even.c_str(), 0x580000d3); // fsqrt.s f1, f0, rne
    PN_INFOF(("---"));

    PN_INFOF(("Test 2: Round towards Zero"));
    PN_INFOF(("Using Tests in: %s", rminMag.c_str()));
    berkeley_single_argument_tests(rminMag.c_str(), 0x580010d3); // fsqrt.s f1, f0, rtz
    PN_INFOF(("---"));

    PN_INFOF(("Test 3: Round Down"));
    PN_INFOF(("Using Tests in: %s", rmin.c_str()));
    berkeley_single_argument_tests(rmin.c_str(), 0x580020d3); // fsqrt.s f1, f0, rdn
    PN_INFOF(("---"));

    PN_INFOF(("Test 4: Round Up"));
    PN_INFOF(("Using Tests in: %s", rmax.c_str()));
    berkeley_single_argument_tests(rmax.c_str(), 0x580030d3); // fsqrt.s f1, f0, rup
    PN_INFOF(("---"));

    PN_INFOF(("Test 5: Round to Nearest, ties to Max Magnitude"));
    PN_INFOF(("Using Tests in: %s", rnear_maxMag.c_str()));
    berkeley_single_argument_tests(rnear_maxMag.c_str(), 0x580040d3); // fsqrt.s f1, f0, rmm
    PN_INFOF(("---"));        
#else
    PN_INFOF(("ERROR: PN_TEST_DIR is not defined! Cannot run tests."));
#endif
}

void mul_add_tests() {
    PN_INFOF(("STARTING MUL_ADD TESTS"));
    PN_INFOF(("---"));
    rounding_mode_in = 0b111;

#ifdef PN_TEST_DIR
    // Create standard string paths dynamically
    std::string test_dir = PN_TEST_DIR;
    std::string rnear_even   = test_dir + "/f32_mulAdd_rnear_even_tests.txt";
    std::string rminMag      = test_dir + "/f32_mulAdd_rminMag_tests.txt";
    std::string rmin         = test_dir + "/f32_mulAdd_rmin_tests.txt";
    std::string rmax         = test_dir + "/f32_mulAdd_rmax_tests.txt";
    std::string rnear_maxMag = test_dir + "/f32_mulAdd_rnear_maxMag_tests.txt";

    PN_INFOF(("Test 1: Round to Nearest, ties to Even"));
    PN_INFOF(("Using Tests in: %s", rnear_even.c_str()));
    berkeley_triple_argument_tests(rnear_even.c_str(), 0x101001c3); // fmadd.s f3, f0, f1, f2, rne
    PN_INFOF(("---"));

    PN_INFOF(("Test 2: Round towards Zero"));
    PN_INFOF(("Using Tests in: %s", rminMag.c_str()));
    berkeley_triple_argument_tests(rminMag.c_str(), 0x101011c3); // fmadd.s f3, f0, f1, f2, rtz
    PN_INFOF(("---"));

    PN_INFOF(("Test 3: Round Down"));
    PN_INFOF(("Using Tests in: %s", rmin.c_str()));
    berkeley_triple_argument_tests(rmin.c_str(), 0x101021c3); // fmadd.s f3, f0, f1, f2, rdn
    PN_INFOF(("---"));

    PN_INFOF(("Test 4: Round Up"));
    PN_INFOF(("Using Tests in: %s", rmax.c_str()));
    berkeley_triple_argument_tests(rmax.c_str(), 0x101031c3); // fmadd.s f3, f0, f1, f2, rup
    PN_INFOF(("---"));

    PN_INFOF(("Test 5: Round to Nearest, ties to Max Magnitude"));
    PN_INFOF(("Using Tests in: %s", rnear_maxMag.c_str()));
    berkeley_triple_argument_tests(rnear_maxMag.c_str(), 0x101041c3); // fmadd.s f3, f0, f1, f2, rmm
    PN_INFOF(("---"));        
#else
    PN_INFOF(("ERROR: PN_TEST_DIR is not defined! Cannot run tests."));
#endif
}

int sc_main(int argc, char** argv)
{
    sc_trace_file* tf = sc_create_vcd_trace_file("f_extension_tb");
    m_f_extension i_dut{"i_dut"};

    i_dut.clk(clk);
    i_dut.reset(reset);
    i_dut.data_in(data_in);
    i_dut.select_in(select_in);
    i_dut.rs1_select_in(rs1_select_in);
    i_dut.rs2_select_in(rs2_select_in);
    i_dut.rs3_select_in(rs3_select_in);
    i_dut.en_load_in(en_load_in);

    i_dut.stb_in(stb_in);
    i_dut.instruction_in(instruction_in);
    i_dut.rounding_mode_in(rounding_mode_in);
    i_dut.ready_out(ready_out);
    i_dut.f_out(f_out);
    i_dut.rs2_out(rs2_out);

    i_dut.pn_trace(tf, 2);
    if(tf) {
        sc_trace(tf, clk, "clk");
        sc_trace(tf, reset, "reset");
        sc_trace(tf, stb_in, "stb_in");
        sc_trace(tf, instruction_in, "instruction_in");
        sc_trace(tf, f_out, "f_out");
    }

    cout << "\n\t\t*****Simulation started*****" << endl;

    stb_in = 0;
    reset = 1;
    run_cycle(2);
    reset = 0;
    run_cycle(1);

    classify_tests();
    PN_INFOF((""));
    add_tests();
    PN_INFOF((""));
    multiply_tests();
    PN_INFOF((""));
    divide_tests();
    PN_INFOF((""));
    compare_tests();
    PN_INFOF((""));
    convert_tests();
    PN_INFOF((""));
    sign_inject_tests();
    PN_INFOF((""));
    sqrt_tests();
    PN_INFOF((""));
    mul_add_tests();

    if (tf) sc_close_vcd_trace_file(tf);

    cout << "\n\t\t*****Simulation complete*****" << endl;
    return 0;
}