
/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Claus Janicher <claus.janicher@tha.de>
                2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
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

#include <stdint.h>
#include <systemc.h>
#include "top.h"

#define PERIOD_NS 10.0

// initialize TB signals
sc_signal<bool> PN_NAME(clk_25);
sc_signal<bool> PN_NAME(reset);

sc_signal<bool> PN_NAME(rx_i);
sc_signal<bool> PN_NAME(tx_o);

sc_signal<bool> PN_NAME(gpio_up);
sc_signal<bool> PN_NAME(gpio_down);
sc_signal<bool> PN_NAME(gpio_left);
sc_signal<bool> PN_NAME(gpio_right);
sc_signal<bool> PN_NAME(gpio_fire);

sc_signal<sc_uint<4>> PN_NAME(vga_red_o);
sc_signal<sc_uint<4>> PN_NAME(vga_green_o);
sc_signal<sc_uint<4>> PN_NAME(vga_blue_o);
sc_signal<bool> PN_NAME(vga_hsync_o);
sc_signal<bool> PN_NAME(vga_vsync_o);

sc_signal<bool> PN_NAME(sda_i);
sc_signal<bool> PN_NAME(sda_o);
sc_signal<bool> PN_NAME(sda_oe);

sc_signal<bool> PN_NAME(scl_i);
sc_signal<bool> PN_NAME(scl_o);
sc_signal<bool> PN_NAME(scl_oe);

void run_cycle(int cycles = 1)
{
    for(int i = 0; i < cycles; i++)
    {
        clk_25 = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk_25 = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

int sc_main(int argc, char** argv)
{
    pn_cfg_enable_application_path = 1;               // enable application path in program args
    PN_PARSE_CMD_ARGS(argc, argv);                    // parse command line arguments
    sc_trace_file* tf = PN_BEGIN_TRACE("piconut_tb"); // create trace file

    // Initialliaze the Design under Testing (DUT)
    m_refdesign_demonstrator i_dut{"i_dut"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    i_dut.clk(clk_25);
    i_dut.reset(reset);
    i_dut.rx_i(rx_i);
    i_dut.tx_o(tx_o);

    i_dut.gpio_up(gpio_up);
    i_dut.gpio_down(gpio_down);
    i_dut.gpio_left(gpio_left);
    i_dut.gpio_right(gpio_right);
    i_dut.gpio_fire(gpio_fire);

    i_dut.vga_red_o(vga_red_o);
    i_dut.vga_green_o(vga_green_o);
    i_dut.vga_blue_o(vga_blue_o);
    i_dut.vga_hsync_o(vga_hsync_o);
    i_dut.vga_vsync_o(vga_vsync_o);

    i_dut.sda_i(sda_i);
    i_dut.sda_o(sda_o);
    i_dut.sda_oe(sda_oe);

    i_dut.scl_o(scl_o);
    i_dut.scl_i(scl_i);
    i_dut.scl_oe(scl_oe);

    i_dut.pn_trace(tf, pn_cfg_vcd_level); // trace signals of DUT

    sc_start(SC_ZERO_TIME); // start simulation

    return 0;
}
