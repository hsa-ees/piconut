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

#include "../divisor_float.h"

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

int divide(int a, int b, int rm) {
    a_in = a;
    b_in = b;
    stb_in = 1;
    rounding_mode_in = rm;
    while(!ready_out) {
        run_cycle();
    }
    do {
        run_cycle();
        stb_in = 0;
    } while (done_out == 0);

    return result_out.read();
}

void berkeley_tests(std::string filename, int rm) {
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
        
        uint32_t actual = divide(a, b, rm);

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
    PN_INFOF(("========================================\n"));
}

int sc_main(int argc, char** argv) {
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("divisor_float_tb");

    m_divisor_float i_dut{"i_dut"};

    i_dut.clk(clk);
    i_dut.reset(reset);

    i_dut.stb_in(stb_in);

    i_dut.a_in(a_in);
    i_dut.b_in(b_in);
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

    // https://www.jhauser.us/arithmetic/TestFloat.html
    PN_INFOF(("*****Testing Round to Nearest, ties to Even*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/divisor_float/divisor_float_tb/f32_div_rnear_even_tests.txt", 0);
    PN_INFOF(("*****Testing Round towards Zero*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/divisor_float/divisor_float_tb/f32_div_rminMag_tests.txt", 1);
    PN_INFOF(("*****Testing Round Down*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/divisor_float/divisor_float_tb/f32_div_rmin_tests.txt", 2);
    PN_INFOF(("*****Testing Round Up*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/divisor_float/divisor_float_tb/f32_div_rmax_tests.txt", 3);
    PN_INFOF(("*****Testing Round to Nearest, ties to Max Magnitude*****"));
    berkeley_tests("/home/piconut-user/piconut/hw/cpu/nucleus/nucleus_ref/f_extension/divisor_float/divisor_float_tb/f32_div_rnear_maxMag_tests.txt", 4);

    //PN_INFOF(("%08X", divide(0xac7f0000, 0x80ffffff, 0x0)));
    
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}