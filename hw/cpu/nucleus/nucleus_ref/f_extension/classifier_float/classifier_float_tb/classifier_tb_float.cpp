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

#include "../classifier_float.h"

#include <systemc.h>
#include <cstdint>
#include <piconut.h>

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

sc_signal<sc_uint<32>> PN_NAME(data_in);

sc_signal<sc_uint<32>> PN_NAME(data_out);

void run_cycle(int cycles = 1) {
    for(int i = 0; i < cycles; i++) {
        clk = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

int sc_main(int argc, char** argv) {
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("classifier_float_tb");

    m_classifier_float i_dut{"i_dut"};

    i_dut.clk(clk);
    i_dut.reset(reset);

    i_dut.data_in(data_in);
    i_dut.data_out(data_out);

    i_dut.pn_trace(tf, pn_cfg_vcd_level);

    sc_start(SC_ZERO_TIME);

    cout << "\n\t\t*****Simulation started*****" << endl;

    run_cycle();
    reset = 1;
    run_cycle();
    reset = 0;
    run_cycle();

    // --- Special Cases: Zero and Infinity ---
    
    PN_INFO("TEST 1: Check negative infinity");
    data_in = 0b1'11111111'00000000000000000000000;
    run_cycle();
    PN_ASSERTM(data_out.read() == NEGATIVE_INFINITY, "TEST 1 failed");
    run_cycle();

    PN_INFO("TEST 2: Check positive infinity");
    data_in = 0b0'11111111'00000000000000000000000;
    run_cycle();
    PN_ASSERTM(data_out.read() == POSITIVE_INFINITY, "TEST 2 failed");
    run_cycle();

    PN_INFO("TEST 3: Check negative zero");
    data_in = 0b1'00000000'00000000000000000000000;
    run_cycle();
    PN_ASSERTM(data_out.read() == NEGATIVE_ZERO, "TEST 3 failed");
    run_cycle();

    PN_INFO("TEST 4: Check positive zero");
    data_in = 0b0'00000000'00000000000000000000000;
    run_cycle();
    PN_ASSERTM(data_out.read() == POSITIVE_ZERO, "TEST 4 failed");
    run_cycle();

    // --- Normal Numbers ---

    PN_INFO("TEST 5: Check negative normal");
    // Exponent is neither 0 nor 255 (e.g., 127), Sign is 1
    data_in = 0b1'01111111'00000000000000000000000; 
    run_cycle();
    PN_ASSERTM(data_out.read() == NEGATIVE_NORMAL, "TEST 5 failed");
    run_cycle();

    PN_INFO("TEST 6: Check positive normal");
    // Exponent is neither 0 nor 255 (e.g., 127), Sign is 0
    data_in = 0b0'01111111'00000000000000000000000;
    run_cycle();
    PN_ASSERTM(data_out.read() == POSITIVE_NORMAL, "TEST 6 failed");
    run_cycle();

    // --- Subnormal Numbers ---

    PN_INFO("TEST 7: Check negative subnormal");
    // Exponent is 0, Mantissa is non-zero, Sign is 1
    data_in = 0b1'00000000'00000000000000000000001;
    run_cycle();
    PN_ASSERTM(data_out.read() == NEGATIVE_SUBNORMAL, "TEST 7 failed");
    run_cycle();

    PN_INFO("TEST 8: Check positive subnormal");
    // Exponent is 0, Mantissa is non-zero, Sign is 0
    data_in = 0b0'00000000'00000000000000000000001;
    run_cycle();
    PN_ASSERTM(data_out.read() == POSITIVE_SUBNORMAL, "TEST 8 failed");
    run_cycle();

    // --- NaNs (Not a Number) ---

    PN_INFO("TEST 9: Check signaling NaN (sNaN)");
    // Exponent is 255, Mantissa MSB is 0, but rest is non-zero
    data_in = 0b0'11111111'00000000000000000000001;
    run_cycle();
    PN_ASSERTM(data_out.read() == SIGNALING_NAN, "TEST 9 failed");
    run_cycle();

    PN_INFO("TEST 10: Check quiet NaN (qNaN)");
    // Exponent is 255, Mantissa MSB is 1
    data_in = 0b0'11111111'10000000000000000000000;
    run_cycle();
    PN_ASSERTM(data_out.read() == QUIET_NAN, "TEST 10 failed");
    run_cycle();

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}