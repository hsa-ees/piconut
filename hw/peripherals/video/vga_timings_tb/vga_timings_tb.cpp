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

#include "../vga_timings.h"

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<sc_uint<5>> PN_NAME(resolution_mode);

sc_signal<sc_uint<H_COUNTER_WIDTH>> PN_NAME(column_end);
sc_signal<sc_uint<V_COUNTER_WIDTH>> PN_NAME(line_end);
sc_signal<sc_uint<H_COUNTER_WIDTH>> PN_NAME(column_active);
sc_signal<sc_uint<V_COUNTER_WIDTH>> PN_NAME(line_active);
sc_signal<sc_uint<H_COUNTER_WIDTH>> PN_NAME(hsync_begin);
sc_signal<sc_uint<H_COUNTER_WIDTH>> PN_NAME(hsync_end);
sc_signal<sc_uint<V_COUNTER_WIDTH>> PN_NAME(vsync_begin);
sc_signal<sc_uint<V_COUNTER_WIDTH>> PN_NAME(vsync_end);

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

    m_vga_timings i_dut{"i_dut"};
    i_dut.resolution_mode_in(resolution_mode);
    i_dut.column_end_out(column_end);
    i_dut.line_end_out(line_end);
    i_dut.column_active_out(column_active);
    i_dut.line_active_out(line_active);
    i_dut.vga_hsync_begin_out(hsync_begin);
    i_dut.vga_hsync_end_out(hsync_end);
    i_dut.vga_vsync_begin_out(vsync_begin);
    i_dut.vga_vsync_end_out(vsync_end);

    i_dut.pn_trace(tf, pn_cfg_vcd_level);

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    resolution_mode = 0; // RESOLUTION_MODE_640x480
    run_cycle(1);
    PN_ASSERTM(column_active.read() == 640, "Incorrect column_active for 640x480");
    PN_ASSERTM(line_active.read() == 480, "Incorrect line_active for 640x480");
    PN_ASSERTM(column_end.read() == 800, "Incorrect column_end for 640x480");
    PN_ASSERTM(line_end.read() == 524, "Incorrect line_end for 640x480");
    PN_ASSERTM(hsync_begin.read() == 656, "Incorrect hsync_begin for 640x480");
    PN_ASSERTM(hsync_end.read() == 752, "Incorrect hsync_end for 640x480");
    PN_ASSERTM(vsync_begin.read() == 490, "Incorrect vsync_begin for 640x480");
    PN_ASSERTM(vsync_end.read() == 492, "Incorrect vsync_end for 640x480");

    run_cycle(1);

    resolution_mode = -1; // Invalid mode
    run_cycle(1);
    PN_ASSERTM(column_active.read() == 0, "Incorrect column_active for invalid mode");
    PN_ASSERTM(line_active.read() == 0, "Incorrect line_active for invalid mode");
    PN_ASSERTM(column_end.read() == 0, "Incorrect column_end for invalid mode");
    PN_ASSERTM(line_end.read() == 0, "Incorrect line_end for invalid mode");
    PN_ASSERTM(hsync_begin.read() == 0, "Incorrect hsync_begin for invalid mode");
    PN_ASSERTM(hsync_end.read() == 0, "Incorrect hsync_end for invalid mode");
    PN_ASSERTM(vsync_begin.read() == 0, "Incorrect vsync_begin for invalid mode");
    PN_ASSERTM(vsync_end.read() == 0, "Incorrect vsync_end for invalid mode");

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}
