/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2023-2024 Lukas Bauer <lukas.bauer1@hs-augsburg.de
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    This is a test bench for the memory unit (MEMU) of the PicoNut.

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

#include <systemc.h>
#include <stdint.h>
#include "bootram.h"

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk_i); // clock input
sc_signal<bool> PN_NAME(rst_i); // reset

sc_signal<bool> PN_NAME(stb_i);            // strobe input
sc_signal<bool> PN_NAME(cyc_i);            // cycle valid input
sc_signal<bool> PN_NAME(we_i);             // indicates write transfer
sc_signal<sc_uint<32 / 8>> PN_NAME(sel_i); // byte select inputs

sc_signal<bool> PN_NAME(ack_o); // normal termination
sc_signal<bool> PN_NAME(err_o); // termination w/ error
sc_signal<bool> PN_NAME(rty_o); // termination w/ retry

sc_signal<sc_uint<32>> PN_NAME(adr_i);  // address bus inputs
sc_signal<sc_uint<32>> PN_NAME(dat_i);  // input data bus
sc_signal<sc_uint<32>> PN_NAME(dat_o);  // output data bus
sc_signal<sc_uint<32>> PN_NAME(addr_o); // output data bus

void run_cycle(int cycles = 1)
{
    for (int i = 0; i < cycles; i++)
    {
        clk_i = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk_i = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

int sc_main(int argc, char **argv)
{

    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file *tf = PN_BEGIN_TRACE("bootram_tb");

    // MWBMemory memory;
    m_boot_ram bootRam{"bootRam"};

    // Connect signals
    bootRam.clk_i(clk_i);
    bootRam.rst_i(rst_i);
    bootRam.stb_i(stb_i);
    bootRam.cyc_i(cyc_i);
    bootRam.we_i(we_i);
    bootRam.sel_i(sel_i);
    bootRam.ack_o(ack_o);
    bootRam.err_o(err_o);
    bootRam.rty_o(rty_o);
    bootRam.adr_i(adr_i);
    bootRam.dat_i(dat_i);
    bootRam.dat_o(dat_o);

    // Local signal traces
    PN_TRACE(tf, clk_i);
    PN_TRACE(tf, rst_i);
    PN_TRACE(tf, stb_i);
    PN_TRACE(tf, cyc_i);
    PN_TRACE(tf, we_i);
    PN_TRACE(tf, sel_i);
    PN_TRACE(tf, ack_o);
    PN_TRACE(tf, err_o);
    PN_TRACE(tf, rty_o);
    PN_TRACE(tf, adr_i);
    PN_TRACE(tf, dat_i);
    PN_TRACE(tf, dat_o);

    // Traces of Module instances
    bootRam.Trace(tf, pn_cfg_vcd_level);

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Reset the testbench
    PN_INFO("Resetting.....");
    rst_i = 1;
    run_cycle(5);
    rst_i = 0;
    run_cycle(5);

    // check if data can be read should be zero
    PN_INFO("Testing a read access to the block ram at address 0xC0000004, expecting 0x00000000");
    adr_i = 0xC0000004;
    cyc_i = 1;
    stb_i = 1;
    sel_i = 0xf;
    we_i = 0;
    while (ack_o == 0)
        run_cycle();

    PN_INFOF(("dat_o: 0x%08x", (uint32_t)dat_o.read()));
    PN_ASSERTM(0x0 == dat_o.read(), "Data was not correct");

    // reseting the wb interface
    cyc_i = 0;
    stb_i = 0;
    sel_i = 0;
    adr_i = 0x00000000;
    run_cycle(5);

    // check if data can be written to address
    PN_INFO("Write access to the block ram at address 0xC0000004, writing 0x12345678");
    adr_i = 0xC0000004;
    dat_i = 0x12345678;
    cyc_i = 1;
    stb_i = 1;
    sel_i = 0xf;
    we_i = 1;
    while (ack_o == 0)
        run_cycle();

    // reseting the wb interface
    cyc_i = 0;
    stb_i = 0;
    sel_i = 0;
    adr_i = 0x00000000;
    run_cycle(5);

    // check if data was writen correctly by reading it again
    PN_INFO("Testing a read access to the block ram at address 0xC0000004, expecting 0x12345678");
    adr_i = 0xC0000004;
    cyc_i = 1;
    stb_i = 1;
    sel_i = 0xf;
    we_i = 0;
    while (ack_o == 0)
        run_cycle();

    PN_INFOF(("dat_o: 0x%08x", (uint32_t)dat_o.read()));
    PN_ASSERTM(0x12345678 == dat_o.read(), "Data was not correct");

    // reseting the wb interface
    cyc_i = 0;
    stb_i = 0;
    sel_i = 0;
    adr_i = 0x00000000;
    run_cycle(5);

    // writing different data
    PN_INFO("Write access to the block ram at address 0xC0000004, writing 0x11111111");
    adr_i = 0xC0000004;
    dat_i = 0x11111111;
    cyc_i = 1;
    stb_i = 1;
    sel_i = 0b0001;
    we_i = 1;
    while (ack_o == 0)
        run_cycle();

    // reseting the wb interface
    cyc_i = 0;
    stb_i = 0;
    sel_i = 0;
    adr_i = 0x00000000;
    run_cycle(5);

    // checking it again
    PN_INFO("Testing a read access to the block ram at address 0xC0000004, expecting 0x11111111");
    adr_i = 0xC0000004;
    cyc_i = 1;
    stb_i = 1;
    sel_i = 0xf;
    we_i = 0;
    while (ack_o == 0)
        run_cycle();

    PN_INFOF(("dat_o: 0x%08x", (uint32_t)dat_o.read()));
    PN_ASSERTM(0x12345611 == dat_o.read(), "Data was not correct");

    // reseting the wb interface
    cyc_i = 0;
    stb_i = 0;
    sel_i = 0;
    adr_i = 0x00000000;
    run_cycle(5);

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}