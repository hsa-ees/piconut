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

#include "../video.h"

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

// signals for wishbone bus
sc_signal<pn_wb_adr_t> PN_NAME(wb_adr_i);
sc_signal<pn_wb_dat_t> PN_NAME(wb_dat_o);
sc_signal<pn_wb_dat_t> PN_NAME(wb_dat_i);
sc_signal<pn_wb_sel_t> PN_NAME(wb_sel_i);
sc_signal<bool> PN_NAME(wb_we_i);
sc_signal<bool> PN_NAME(wb_stb_i);
sc_signal<bool> PN_NAME(wb_ack_o);
sc_signal<bool> PN_NAME(wb_cyc_i);

sc_signal<bool> PN_NAME(wb_rty_o);
sc_signal<bool> PN_NAME(wb_err_o);

sc_signal<sc_uint<4>> PN_NAME(vga_red_out);
sc_signal<sc_uint<4>> PN_NAME(vga_green_out);
sc_signal<sc_uint<4>> PN_NAME(vga_blue_out);
sc_signal<bool> PN_NAME(vga_hsync_out);
sc_signal<bool> PN_NAME(vga_vsync_out);

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

void wishbone_write(uint32_t adr, uint32_t data, uint8_t sel = 0xF)
{

    int count = 0;

    // set the signals for wb write setup
    wb_stb_i = 1;
    wb_cyc_i = 1;
    wb_we_i = 1;
    wb_sel_i = sel;
    wb_adr_i = adr;
    wb_dat_i = data;

    // wait for acknowledgement of the write
    while(wb_ack_o.read() == 0)
    {
        run_cycle();
        count++;

        if(count > 100)
        {
            PN_ERROR("WB Write Timeout");
        }
    }

    run_cycle();

    // clear the signals at the wb
    wb_stb_i = 0;
    wb_cyc_i = 0;
    wb_we_i = 0;
    wb_sel_i = 0;
    wb_adr_i = 0;
    wb_dat_i = 0;

    run_cycle();
}

uint32_t wishbone_read(uint32_t adr, uint8_t sel = 0xF)
{

    int cycles = 0;

    // set the signals for wb read setup
    wb_stb_i = 1;
    wb_cyc_i = 1;
    wb_we_i = 0;
    wb_sel_i = sel;
    wb_adr_i = adr;
    wb_dat_i = 0;

    // wait for acknowledgement of the read
    while(wb_ack_o.read() == 0)
    {
        run_cycle();
        cycles++;

        if(cycles > 100)
        {
            PN_ERROR("WB Read Timeout");
        }
    }

    // getting the data from the wb
    uint32_t data = (uint32_t)wb_dat_o.read();

    // clear the signals at the wb
    wb_stb_i = 0;
    wb_cyc_i = 0;
    wb_we_i = 0;
    wb_sel_i = 0;
    wb_adr_i = 0;

    run_cycle();

    return data;
}

int sc_main(int argc, char** argv)
{
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("vga_sync_generator");

    m_video i_dut{"i_dut", PN_CFG_VIDEO_BASE_ADDRESS};
    i_dut.clk(clk);
    i_dut.reset(reset);

    i_dut.wb_slave.adr_i(wb_adr_i);
    i_dut.wb_slave.dat_o(wb_dat_o);
    i_dut.wb_slave.dat_i(wb_dat_i);
    i_dut.wb_slave.sel_i(wb_sel_i);
    i_dut.wb_slave.we_i(wb_we_i);
    i_dut.wb_slave.stb_i(wb_stb_i);
    i_dut.wb_slave.ack_o(wb_ack_o);
    i_dut.wb_slave.cyc_i(wb_cyc_i);
    i_dut.wb_slave.rty_o(wb_rty_o);
    i_dut.wb_slave.err_o(wb_err_o);

    i_dut.vga_red_out(vga_red_out);
    i_dut.vga_green_out(vga_green_out);
    i_dut.vga_blue_out(vga_blue_out);
    i_dut.vga_hsync_out(vga_hsync_out);
    i_dut.vga_vsync_out(vga_vsync_out);

    i_dut.pn_trace(tf, pn_cfg_vcd_level);

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Reset
    PN_INFO("Resetting");
    reset = 1;
    run_cycle(1);

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}
