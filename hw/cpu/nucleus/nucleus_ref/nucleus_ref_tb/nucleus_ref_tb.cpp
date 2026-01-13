/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lorenz Sommer <lorenz.sommer@tha.de>
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
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR a_in PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *************************************************************************/

#include <systemc.h>
#include <piconut.h>

#include <stdint.h>

#include "../nucleus_ref.h"


#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

/* ----------------- Input signals -----------------  */
/* IPort */
sc_signal<sc_uint<32>> PN_NAME(iport_rdata_in);
sc_signal<bool> PN_NAME(iport_ack_in);

/* DPort */
sc_signal<sc_uint<32>> PN_NAME(dport_rdata_in);
sc_signal<bool> PN_NAME(dport_ack_in);

/* Debug */
sc_signal<bool> PN_NAME(debug_haltrequest_in);

/* Interrupts */
sc_signal<bool> PN_NAME(msip_in); // Software Interrupt signal
sc_signal<bool> PN_NAME(mtip_in); // Timer Interrupt signal
sc_signal<bool> PN_NAME(meip_in); // External interrupt signal

/* ----------------- Output signals -----------------  */
/* IPort */
sc_signal<sc_uint<32>> PN_NAME(iport_adr_out);
sc_signal<sc_uint<4>> PN_NAME(iport_bsel_out);

sc_signal<bool> PN_NAME(iport_stb_out);

/* DPort */
sc_signal<sc_uint<32>> PN_NAME(dport_adr_out);
sc_signal<sc_uint<32>> PN_NAME(dport_wdata_out);
sc_signal<sc_uint<4>> PN_NAME(dport_bsel_out);
sc_signal<bool> PN_NAME(dport_we_out);
sc_signal<bool> PN_NAME(dport_stb_out);
sc_signal<bool> PN_NAME(dport_lrsc_out);
sc_signal<bool> PN_NAME(dport_amo_out);

/* Debug */
sc_signal<bool> PN_NAME(debug_haltrequest_ack_out);

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

void run_instruction(sc_uint<32> instruction, int cycles = 4)
{
    iport_rdata_in = instruction;
    iport_ack_in = 1;
    run_cycle(); // AWAIT_IPORT_ACK
    iport_ack_in = 0;
    for(int i = 0; i < cycles - 1; i++)
    {
        run_cycle(); // decode, ...
    }
    PN_ASSERTF(iport_stb_out.read() == 1, ("Wrong number of cycles: %d, for instruction 0x%08x, iport_stb_out should be 1 but is %d", cycles, instruction, (int)iport_stb_out.read()));
    run_cycle(); // IPORT_STB
}

int sc_main(int argc, char** argv)
{

    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("nucleus_tb");

    m_nucleus_ref i_dut{"i_dut"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB

    i_dut.clk(clk);
    i_dut.reset(reset);
    i_dut.iport_rdata_in(iport_rdata_in);
    i_dut.iport_ack_in(iport_ack_in);
    i_dut.dport_rdata_in(dport_rdata_in);
    i_dut.dport_ack_in(dport_ack_in);
    i_dut.debug_haltrequest_in(debug_haltrequest_in);
    i_dut.msip_in(msip_in);
    i_dut.mtip_in(mtip_in);
    i_dut.meip_in(meip_in);

    i_dut.iport_adr_out(iport_adr_out);
    i_dut.iport_bsel_out(iport_bsel_out);
    i_dut.iport_stb_out(iport_stb_out);
    i_dut.dport_adr_out(dport_adr_out);
    i_dut.dport_wdata_out(dport_wdata_out);
    i_dut.dport_bsel_out(dport_bsel_out);
    i_dut.dport_we_out(dport_we_out);
    i_dut.dport_stb_out(dport_stb_out);
    i_dut.dport_lrsc_out(dport_lrsc_out);
    i_dut.dport_amo_out(dport_amo_out);
    i_dut.debug_haltrequest_ack_out(debug_haltrequest_ack_out);

    i_dut.pn_trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT

    sc_start(SC_ZERO_TIME); // start simulation

    cout << "\n\t\t*****Simulation started*****" << endl;

    while(iport_stb_out == 0)
    {
        run_cycle();
    }

    // instructions from https://luplab.gitlab.io/rvcodecjs/
    run_instruction(0x00a10193); // addi x3, x0, 10

    run_instruction(0x00014397); // auipc x4, 20

    run_instruction(0x00006fb7); // lui x31, 6

    iport_rdata_in = 0x000faf03; // lw x30, 0(x31)
    iport_ack_in = 1;
    run_cycle(); // AWAIT_IPORT_ACK
    iport_ack_in = 0;
    run_cycle(); // decode
    dport_ack_in = 0;
    run_cycle(); // LOAD1
    run_cycle(); // LOAD2
    PN_ASSERTM(dport_stb_out.read() == 1, "Test #1 failed: Should strobe dport");
    PN_ASSERTM(dport_bsel_out.read() == 0xf, "Test #1 failed: Should select all bytes");
    PN_ASSERTF(dport_adr_out.read() == 0x00006000, ("Test #1 failed: dport_adr_out should be 0x00006000 but is 0x%08x", (uint32_t)dport_adr_out.read()));
    dport_ack_in = 1;
    dport_rdata_in = 0x0000006f; // dummy value
    run_cycle();                 // LOAD3
    run_cycle();                 // LOAD4
    run_cycle();                 // IPORT_STB

    iport_rdata_in = 0x01ffafaf; // amoadd.w x31, x31, (x31)
    iport_ack_in = 1;
    run_cycle(); // AWAIT_IPORT_ACK
    dport_ack_in = 0;
    run_cycle(); // DECODE
    run_cycle(); // A_LOAD1
    run_cycle(); // A_LOAD2
    PN_ASSERTM(dport_stb_out.read() == 1, "Test #2 failed: Should strobe dport");
    PN_ASSERTM(dport_bsel_out.read() == 0xf, "Test #2 failed: Should select all bytes");
    dport_rdata_in = 0xFF00; // dummy value
    dport_ack_in = 1;
    PN_ASSERTF(dport_adr_out.read() == 0x00006000, ("Test #2 failed: dport_adr_out should be 0x00006000 but is 0x%08x", (uint32_t)dport_adr_out.read()));
    run_cycle(); // A_LOAD3
    run_cycle(); // A_DECODE
    run_cycle(); // A_ALU_OP
    dport_ack_in = 0;
    run_cycle(); // A_STORE1
    PN_ASSERTM(dport_adr_out.read() == 0x00006000, "Test #2 failed: dport_adr_out should have held the address in register");
    run_cycle(); // A_STORE2
    dport_ack_in = 1;
    run_cycle(); // A_STORE3
    run_cycle(); // A_WRITE_OLD
    run_cycle(); // IPORT_STB

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}
