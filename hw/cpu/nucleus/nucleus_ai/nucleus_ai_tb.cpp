/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    Synthesis-only testbench of the Nucleus AI.

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


#include "nucleus_ai.h"





// ********************** Ports ************************************************


// General ...
sc_signal<bool>         PN_NAME(clk);
sc_signal<bool>         PN_NAME(reset);
sc_signal<bool>         PN_NAME(pwroff);

// IPort ...
sc_signal<bool>         PN_NAME(iport_stb);
sc_signal<sc_uint<32>>  PN_NAME(iport_adr);
sc_signal<sc_uint<4>>   PN_NAME(iport_bsel);
sc_signal<sc_uint<32>>  PN_NAME(iport_rdata);
sc_signal<bool>         PN_NAME(iport_ack);

// DPort ...
sc_signal<bool>         PN_NAME(dport_stb);
sc_signal<bool>         PN_NAME(dport_we);
sc_signal<sc_uint<32>>  PN_NAME(dport_adr);
sc_signal<sc_uint<32>>  PN_NAME(dport_wdata);
sc_signal<sc_uint<4>>   PN_NAME(dport_bsel);
sc_signal<bool>         PN_NAME(dport_amo);
sc_signal<bool>         PN_NAME(dport_lrsc);
sc_signal<sc_uint<32>>  PN_NAME(dport_rdata);
sc_signal<bool>         PN_NAME(dport_ack);

// VPort ...
sc_signal<bool>            PN_NAME(vport_stb);
sc_signal<bool>            PN_NAME(vport_we);
sc_signal<bool>            PN_NAME(vport_ack);
sc_signal<bool>            PN_NAME(vport_lrsc);
sc_signal<bool>            PN_NAME(vport_amo);
sc_signal<pn_dport_adr_t>  PN_NAME(vport_adr);
sc_signal<pn_dport_bsel_t> PN_NAME(vport_bsel);
sc_signal<pn_dport_dat_t>  PN_NAME(vport_rdata);
sc_signal<pn_dport_dat_t>  PN_NAME(vport_wdata);

// Debugging ...
sc_signal<bool>         PN_NAME(debug_halt_req);
sc_signal<bool>         PN_NAME(debug_halt_ack);

// Interrupts ...
sc_signal<bool>         PN_NAME(msip_in); // Software Interrupt input
sc_signal<bool>         PN_NAME(mtip_in); // Timer Interrupt input
sc_signal<bool>         PN_NAME(meip_in); // External interrupt input





// ********************** Helpers **********************************************


#define PERIOD_NS 10.0


static int cycles = 0;


void run_cycle (int n = 1) {
  for (int i = 0; i < n; i++) {
    clk = 1;
    sc_start (PERIOD_NS / 2, SC_NS);
    clk = 0;
    sc_start (PERIOD_NS / 2, SC_NS);
  }
  clk = 1;
  sc_start (0, SC_NS);    // Trigger DUT events to let them happen before new inputs are set by testbench

  cycles += n;
}


static inline int get_cycles () { return cycles; }





// ********************** Main *************************************************


int sc_main (int argc, char** argv) {
  int i;

  // Parse args ...
  pn_cfg_itrace_level = 1;
  pn_tb_parse_cmd_args (argc, argv);

  // Instantiate and connect DUT ...
  m_nucleus_ai i_dut{"i_dut"}; // this is the Design name needed by the svc_tool

  i_dut.clk         (clk);
  i_dut.reset       (reset);
  i_dut.pwroff      (pwroff);

  i_dut.iport_adr   (iport_adr);
  i_dut.iport_bsel  (iport_bsel);
  i_dut.iport_rdata (iport_rdata);
  i_dut.iport_stb   (iport_stb);
  i_dut.iport_ack   (iport_ack);

  i_dut.dport_adr   (dport_adr);
  i_dut.dport_bsel  (dport_bsel);
  i_dut.dport_rdata (dport_rdata);
  i_dut.dport_wdata (dport_wdata);
  i_dut.dport_stb   (dport_stb);
  i_dut.dport_we    (dport_we);
  i_dut.dport_ack   (dport_ack);
  i_dut.dport_lrsc  (dport_lrsc);
  i_dut.dport_amo   (dport_amo);

  i_dut.vport_stb   (vport_stb);
  i_dut.vport_we    (vport_we);
  i_dut.vport_ack   (vport_ack);
  i_dut.vport_lrsc  (vport_lrsc);
  i_dut.vport_amo   (vport_amo);
  i_dut.vport_adr   (vport_adr);
  i_dut.vport_bsel  (vport_bsel);
  i_dut.vport_rdata (vport_rdata);
  i_dut.vport_wdata (vport_wdata);

  i_dut.debug_halt_req (debug_halt_req);
  i_dut.debug_halt_ack (debug_halt_ack);

  i_dut.msip_in (msip_in);
  i_dut.mtip_in (mtip_in);
  i_dut.meip_in (meip_in);

  // Open trace file ...
  sc_trace_file *tf = pn_tb_start_trace ();
  i_dut.pn_trace (tf, pn_cfg_vcd_level);

  // End of elaboration, start (very rudimentary) simulation ...
  sc_start();

  // ... reset ...
  reset = 0;
  run_cycle (4);
  reset = 1;
  run_cycle ();
  reset = 0;
  run_cycle ();
  PN_ASSERT (pwroff.read () == 0);

  // ... deliver: li a7, 93 ...
  PN_INFO ("Executing 'li a7, 93' ...");
  for (i = 0; iport_stb.read () == 0; run_cycle (), i++)
    PN_ASSERT(i < 100);
  PN_ASSERT ((iport_adr.read () == PN_CFG_CPU_RESET_ADR));
  iport_rdata = 0x05d00893;  // ADDI X17, X0, 93 (000001011101 00000 000 10001 0010011
  iport_ack = 1;
  run_cycle ();
  iport_ack = 0;

  // ... deliver: ecall ...
  PN_INFO ("Executing 'ecall' ...");
  for (i = 0; iport_stb.read () == 0; run_cycle (), i++)
    PN_ASSERT(i < 100);
  PN_ASSERT ((iport_adr.read () == PN_CFG_CPU_RESET_ADR + 4));
  iport_rdata = 0x00000073;  // ECALL
  iport_ack = 1;
  run_cycle ();
  iport_ack = 0;

  // ... wait and assert power off ...
  PN_INFO ("Waiting for power off...");
  run_cycle (10);
  PN_ASSERT (pwroff.read () == 1);

  // Done ...
  PN_INFO("Simulation completed.");
  pn_tb_end_trace ();
  return 0;
}
