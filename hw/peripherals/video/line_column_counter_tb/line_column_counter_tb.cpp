/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Niklas Sirch <niklas.sirch1@tha.de>
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

#include "../line_column_counter.h"

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);
sc_signal<bool> PN_NAME(c_enable);

sc_signal<sc_uint<H_COUNTER_WIDTH>> PN_NAME(column_end);
sc_signal<sc_uint<V_COUNTER_WIDTH>> PN_NAME(line_end);
sc_signal<sc_uint<H_COUNTER_WIDTH>> PN_NAME(column_active);
sc_signal<sc_uint<V_COUNTER_WIDTH>> PN_NAME(line_active);
sc_signal<sc_uint<H_COUNTER_WIDTH>> PN_NAME(c_column);
sc_signal<sc_uint<V_COUNTER_WIDTH>> PN_NAME(c_line);
sc_signal<bool> PN_NAME(blanking_enable);

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

int sc_main(int argc, char** argv)
{
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("vga_sync_generator");

    m_line_column_counter i_dut{"i_dut"};
    i_dut.clk(clk);
    i_dut.reset(reset);
    i_dut.enable_in(c_enable);
    i_dut.column_end_in(column_end);
    i_dut.line_end_in(line_end);
    i_dut.column_active_in(column_active);
    i_dut.line_active_in(line_active);
    i_dut.vid_column_out(c_column);
    i_dut.vid_line_out(c_line);
    i_dut.blanking_enable_out(blanking_enable);

    i_dut.pn_trace(tf, pn_cfg_vcd_level);

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Reset
    PN_INFO("Resetting");
    c_enable = 0;
    reset = 1;
    run_cycle(1);

    PN_ASSERTM(c_column.read() == 0, "Column counter not reset");
    PN_ASSERTM(c_line.read() == 0, "Line counter not reset");

    reset = 0;
    run_cycle(1);

    PN_ASSERTM(c_column.read() == 0, "Column counter not reset after release");
    PN_ASSERTM(c_line.read() == 0, "Line counter not reset after release");

    PN_INFO("Setting timings and enable");

    // --- Tests ---
    // Configure a small timing to exercise wrap and blanking logic
    column_end = 2;
    line_end = 1;
    column_active = 1;
    line_active = 0;

    // Test 1: not counting while disabled
    c_enable = 0;
    run_cycle(4);
    PN_ASSERTM(c_column.read() == 0, "Column counter advanced while disabled");
    PN_ASSERTM(c_line.read() == 0, "Line counter advanced while disabled");

    // Test 2: counting when enabled
    PN_INFO("Enabling counting");
    c_enable = 1;
    run_cycle(1);
    PN_ASSERTM(c_column.read() == 1, "Column did not increment when enabled");
    PN_ASSERTM(c_line.read() == 0, "Line incremented unexpectedly when stepping column");

    // Blanking should be asserted for our chosen active boundary when column==1
    PN_ASSERTM(blanking_enable.read(), "Blanking not asserted in expected column");

    // Test 3: wraparound increments line
    run_cycle(2); // advance to force column wrap and line increment
    PN_ASSERTM(c_column.read() == 0, "Column did not wrap to 0 on end");
    PN_ASSERTM(c_line.read() == 1, "Line did not increment on column wrap");

    // Test 4: line wraparound (line should wrap back to 0)
    // Advance another full column cycle to trigger line wrap
    run_cycle(3);
    PN_ASSERTM(c_column.read() == 0, "Column did not wrap to 0 on second end");
    PN_ASSERTM(c_line.read() == 0, "Line did not wrap back to 0 on line end");

    // Test 4: zero-length timings (no space) should keep counters at 0
    PN_INFO("Testing zero-length timings");
    column_end = 0;
    line_end = 0;
    column_active = 0;
    line_active = 0;
    c_enable = 1;
    run_cycle(3);
    PN_ASSERTM(c_column.read() == 0, "Column should remain 0 with zero end");
    PN_ASSERTM(c_line.read() == 0, "Line should remain 0 with zero end");

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}
