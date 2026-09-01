/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Marco Milenkovic <Marco.Milenkovic@hs-augsburg.de>
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


#include "demo_ai.h"





// ********************** Ports ************************************************


// General ...
sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);
sc_signal<bool> PN_NAME(pwroff);





// ********************** Helpers **********************************************


#define PERIOD_NS 10.0


static unsigned long long cycles = 0;


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


static inline unsigned long long get_cycles () { return cycles; }





// ********************** Main *************************************************


int sc_main (int argc, char** argv) {
  int c0, c1;
  
  // Parse args ...
  pn_cfg_enable_application_path = 1;   // enable ELF file in args
  pn_parse_enable_trace_core = 1;       // enable '-c' option
  pn_tb_parse_cmd_args (argc, argv);

  // Instantiate and connect DUT ...
  m_demo_ai i_dut{"i_dut"}; // this is the Design name needed by the svc_tool

  i_dut.clk(clk);
  i_dut.reset(reset);
  i_dut.pwroff(pwroff);

  // Load software ...
  i_dut.cpu->membrana->load_elf (pn_cfg_application_path.c_str ());

  // Open trace file ...
  sc_trace_file *tf = pn_tb_start_trace ();
  i_dut.pn_trace (tf, pn_cfg_vcd_level);

  // Start simulation ...
  sc_start();
  printf ("\n********** Simulation started **********\n\n");

  // ... reset ...
  reset = 0;
  run_cycle ();
  reset = 1;
  run_cycle ();
  reset = 0;
  run_cycle ();

  // ... main loop until power off condition is detected ...
  while (i_dut.pwroff.read () == 0)
    run_cycle(1);
  
  delete i_dut.cpu->membrana;

  // Done ...
  pn_tb_end_trace ();
  printf ("\n********** Simulation finished (%llu cycles) **********\n", get_cycles ());
  return 0;
}
