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

#include "vga_sync_generator.h"
#include <systemc.h>

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

sc_signal<sc_uint<H_COUNTER_WIDTH>> PN_NAME(vga_hsync_begin);
sc_signal<sc_uint<H_COUNTER_WIDTH>> PN_NAME(vga_hsync_end);
sc_signal<sc_uint<V_COUNTER_WIDTH>> PN_NAME(vga_vsync_begin);
sc_signal<sc_uint<V_COUNTER_WIDTH>> PN_NAME(vga_vsync_end);
sc_signal<bool> PN_NAME(c_enable);
sc_signal<sc_uint<H_COUNTER_WIDTH>> PN_NAME(c_column);
sc_signal<sc_uint<V_COUNTER_WIDTH>> PN_NAME(c_line);
sc_signal<bool> PN_NAME(vga_hsync);
sc_signal<bool> PN_NAME(vga_vsync);

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

    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, c_enable);
    PN_TRACE(tf, c_line);
    PN_TRACE(tf, c_column);
    PN_TRACE(tf, vga_hsync_begin);
    PN_TRACE(tf, vga_hsync_end);
    PN_TRACE(tf, vga_vsync_begin);
    PN_TRACE(tf, vga_vsync_end);

    m_vga_sync_generator dut_inst{"dut_inst"};
    dut_inst.clk(clk);
    dut_inst.reset(reset);
    dut_inst.vga_hsync_begin(vga_hsync_begin);
    dut_inst.vga_hsync_end(vga_hsync_end);
    dut_inst.vga_vsync_begin(vga_vsync_begin);
    dut_inst.vga_vsync_end(vga_vsync_end);
    dut_inst.enable(c_enable);
    dut_inst.column(c_column);
    dut_inst.line(c_line);
    dut_inst.vga_hsync(vga_hsync);
    dut_inst.vga_vsync(vga_vsync);

    dut_inst.pn_trace(tf, pn_cfg_vcd_level);

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Reset
    PN_INFO("Resetting");
    c_enable = 0;
    reset = 1;
    run_cycle(1);

    PN_ASSERTM(vga_hsync.read(), "HSYNC not deasserted");
    PN_ASSERTM(vga_vsync.read(), "VSYNC not deasserted");

    reset = 0;
    run_cycle(1);

    PN_ASSERTM(vga_hsync.read(), "HSYNC not deasserted");
    PN_ASSERTM(vga_vsync.read(), "VSYNC not deasserted");

    PN_INFO("Setting timings and enable");
    vga_hsync_begin = 640 + 23;
    vga_hsync_end = 640 + 23 + 96;
    vga_vsync_begin = 480 + 10;
    vga_vsync_end = 480 + 10 + 2;
    c_enable = 1;

    PN_INFO("Testing hsync and vsync");

    for(int y = 0; y < 525; y++)
    {
        c_line.write(y);
        for(int x = 0; x < 800; x++)
        {
            c_column.write(x);
            run_cycle();
            PN_ASSERTF(vga_hsync.read() == (x < 663 || x >= 759), ("HSYNC failed at (%d, %d)", x, y));
            PN_ASSERTF(vga_vsync.read() == (y < 490 || y >= 492), ("VSYNC failed at (%d, %d)", x, y));
        }
    }

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}
