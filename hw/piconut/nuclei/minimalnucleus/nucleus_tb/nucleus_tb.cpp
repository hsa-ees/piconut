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



#include "nucleus.h"
#include <systemc.h>
#include <stdint.h>
#include <base.h>

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

int sc_main(int argc, char** argv)
{

    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("nucleus_tb");

    m_nucleus dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB

    dut_inst.clk(clk);
    dut_inst.reset(reset);
    dut_inst.iport_rdata_in(iport_rdata_in);
    dut_inst.iport_ack_in(iport_ack_in);
    dut_inst.dport_rdata_in(dport_rdata_in);
    dut_inst.dport_ack_in(dport_ack_in);
    dut_inst.debug_haltrequest_in(debug_haltrequest_in);

    dut_inst.iport_adr_out(iport_adr_out);
    dut_inst.iport_bsel_out(iport_bsel_out);
    dut_inst.iport_stb_out(iport_stb_out);
    dut_inst.dport_adr_out(dport_adr_out);
    dut_inst.dport_wdata_out(dport_wdata_out);
    dut_inst.dport_bsel_out(dport_bsel_out);
    dut_inst.dport_we_out(dport_we_out);
    dut_inst.dport_stb_out(dport_stb_out);
    dut_inst.debug_haltrequest_ack_out(debug_haltrequest_ack_out);

    dut_inst.Trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT

    sc_start(SC_ZERO_TIME); // start simulation

    cout << "\n\t\t*****Simulation started*****" << endl;

    while(iport_stb_out == 0)
    {
        run_cycle();
    }

    iport_rdata_in = 0x00a10193; // addi x3, x0, 10
    iport_ack_in = 1;
    run_cycle();
    iport_ack_in = 0;
    run_cycle();
    run_cycle();
    run_cycle();
    run_cycle();

    iport_rdata_in = 0x00014397; // auipc x4, 20
    iport_ack_in = 1;
    run_cycle();
    iport_ack_in = 0;
    run_cycle();
    run_cycle();
    run_cycle();
    run_cycle();


    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}