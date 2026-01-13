/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Martin Erichsen <martin.erichsen@tha.de>
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

#include "demo_image_source.h"
#include <systemc.h>

#define PERIOD_NS 10.0

// Testbench Signals
sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

sc_signal<bool> PN_NAME(c_enable);
sc_signal<sc_uint<H_COUNTER_WIDTH>> PN_NAME(c_column);
sc_signal<sc_uint<V_COUNTER_WIDTH>> PN_NAME(c_line);
sc_signal<sc_uint<32>> PN_NAME(vid_output);

void run_cycle(int cycles = 1)
{
    for(int i = 0; i < cycles; i++)
    {
        clk = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

void test_pixel(int x, int y, uint32_t expected)
{
    PN_INFOF(("Testing pixel at (%d, %d)", x, y));
    c_column = x;
    c_line = y;
    run_cycle();
    PN_ASSERTF(vid_output.read() == expected, ("Failed checking pixel at (%d, %d): Output: %08X; Expected: %08X", x, y, (uint32_t)vid_output.read(), expected));
}

int sc_main(int argc, char** argv)
{
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("demo_image_source");

    m_demo_image_source dut_inst{"dut_inst"};
    dut_inst.clk(clk);
    dut_inst.reset(reset);
    dut_inst.vid_enable(c_enable);
    dut_inst.vid_column(c_column);
    dut_inst.vid_line(c_line);
    dut_inst.vid_output(vid_output);

    dut_inst.pn_trace(tf, pn_cfg_vcd_level);

    // Start simulation
    sc_start(SC_ZERO_TIME);
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Reset simulation
    PN_INFO("Resetting");
    c_enable = 0;

    reset = 1;
    run_cycle(1);
    reset = 0;
    run_cycle(1);

    c_enable = 1;

    // Test pixels
    test_pixel(0, 64, 0xE50000);
    test_pixel(64, 0, 0xFFFFFF);
    test_pixel(64, 64, 0x000000);
    test_pixel(8 * 64, 7 * 64, 0xFF0350);

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}
