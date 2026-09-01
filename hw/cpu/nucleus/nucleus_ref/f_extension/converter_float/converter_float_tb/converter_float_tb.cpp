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

#include "../converter_float.h"

#include <systemc.h>
#include <stdint.h>
#include <piconut.h>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

sc_signal<bool> PN_NAME(stb_in);

sc_signal<sc_uint<32>> PN_NAME(a_in);
sc_signal<sc_uint<32>> PN_NAME(b_in);
sc_signal<sc_uint<3>> PN_NAME(cvt_mode_in);
sc_signal<sc_uint<3>> PN_NAME(rounding_mode_in);

sc_signal<sc_uint<32>> PN_NAME(result_out);

sc_signal<bool> PN_NAME(ready_out);
sc_signal<bool> PN_NAME(done_out);

void run_cycle(int cycles = 1) {
    for(int i = 0; i < cycles; i++) {
        clk = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

int convert(int a, int b, int cm, int rm) {
    a_in = a;
    b_in = b;
    stb_in = 1;
    cvt_mode_in = cm;
    rounding_mode_in = rm;
    while(!ready_out) {
        run_cycle();
    }
    while (done_out == 0) {
        run_cycle();
        stb_in = 0;
    };
    stb_in = 0;
    
    return result_out.read();
}

void berkeley_tests(std::string filename, int cm, int rm) {
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
        
        uint32_t actual = convert(a, 0x0, cm, rm);

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

int sc_main(int argc, char** argv) {
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("converter_float_tb");

    m_converter_float i_dut{"i_dut"};

    i_dut.clk(clk);
    i_dut.reset(reset);

    i_dut.stb_in(stb_in);

    i_dut.a_in(a_in);
    i_dut.b_in(b_in);
    i_dut.cvt_mode_in(cvt_mode_in);
    i_dut.rounding_mode_in(rounding_mode_in);

    i_dut.result_out(result_out);

    i_dut.ready_out(ready_out);
    i_dut.done_out(done_out);

    i_dut.pn_trace(tf, pn_cfg_vcd_level);

    sc_start(SC_ZERO_TIME);

    cout << "\n\t\t*****Simulation started*****" << endl;

    run_cycle();
    reset = 1;
    run_cycle();
    reset = 0;
    run_cycle();

    PN_INFOF(("*****Testing F32 to UI32, Round to Nearest, ties to Even*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/converter_float/converter_float_tb/f32_to_ui32_rnear_even_tests.txt", F32_TO_UI32, 0);
    PN_INFOF(("*****Testing F32 to UI32, Round towards Zero*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/converter_float/converter_float_tb/f32_to_ui32_rminMag_tests.txt", F32_TO_UI32, 1);
    PN_INFOF(("*****Testing F32 to UI32, Round Down*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/converter_float/converter_float_tb/f32_to_ui32_rmin_tests.txt", F32_TO_UI32, 2);
    PN_INFOF(("*****Testing F32 to UI32, Round Up*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/converter_float/converter_float_tb/f32_to_ui32_rmax_tests.txt", F32_TO_UI32, 3);
    PN_INFOF(("*****Testing F32 to UI32, Round to Nearest, ties to Max Magnitude*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/converter_float/converter_float_tb/f32_to_ui32_rnear_maxMag_tests.txt", F32_TO_UI32, 4);

    PN_INFOF(("*****Testing F32 to I32, Round to Nearest, ties to Even*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/converter_float/converter_float_tb/f32_to_i32_rnear_even_tests.txt", F32_TO_I32, 0);
    PN_INFOF(("*****Testing F32 to I32, Round towards Zero*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/converter_float/converter_float_tb/f32_to_i32_rminMag_tests.txt", F32_TO_I32, 1);
    PN_INFOF(("*****Testing F32 to I32, Round Down*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/converter_float/converter_float_tb/f32_to_i32_rmin_tests.txt", F32_TO_I32, 2);
    PN_INFOF(("*****Testing F32 to I32, Round Up*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/converter_float/converter_float_tb/f32_to_i32_rmax_tests.txt", F32_TO_I32, 3);
    PN_INFOF(("*****Testing F32 to I32, Round to Nearest, ties to Max Magnitude*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/converter_float/converter_float_tb/f32_to_i32_rnear_maxMag_tests.txt", F32_TO_I32, 4);

    PN_INFOF(("*****Testing UI32 to F32, Round to Nearest, ties to Even*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/converter_float/converter_float_tb/ui32_to_f32_rnear_even_tests.txt", UI32_TO_F32, 0);
    PN_INFOF(("*****Testing UI32 to F32, Round towards Zero*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/converter_float/converter_float_tb/ui32_to_f32_rminMag_tests.txt", UI32_TO_F32, 1);
    PN_INFOF(("*****Testing UI32 to F32, Round Down*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/converter_float/converter_float_tb/ui32_to_f32_rmin_tests.txt", UI32_TO_F32, 2);
    PN_INFOF(("*****Testing UI32 to F32, Round Up*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/converter_float/converter_float_tb/ui32_to_f32_rmax_tests.txt", UI32_TO_F32, 3);
    PN_INFOF(("*****Testing UI32 to F32, Round to Nearest, ties to Max Magnitude*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/converter_float/converter_float_tb/ui32_to_f32_rnear_maxMag_tests.txt", UI32_TO_F32, 4);

    PN_INFOF(("*****Testing I32 to F32, Round to Nearest, ties to Even*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/converter_float/converter_float_tb/i32_to_f32_rnear_even_tests.txt", I32_TO_F32, 0);
    PN_INFOF(("*****Testing I32 to F32, Round towards Zero*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/converter_float/converter_float_tb/i32_to_f32_rminMag_tests.txt", I32_TO_F32, 1);
    PN_INFOF(("*****Testing I32 to F32, Round Down*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/converter_float/converter_float_tb/i32_to_f32_rmin_tests.txt", I32_TO_F32, 2);
    PN_INFOF(("*****Testing I32 to F32, Round Up*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/converter_float/converter_float_tb/i32_to_f32_rmax_tests.txt", I32_TO_F32, 3);
    PN_INFOF(("*****Testing I32 to F32, Round to Nearest, ties to Max Magnitude*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/converter_float/converter_float_tb/i32_to_f32_rnear_maxMag_tests.txt", I32_TO_F32, 4);

    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}