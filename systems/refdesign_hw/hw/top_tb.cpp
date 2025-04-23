
/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Claus Janicher <claus.janicher@tha.de>
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
sc_signal<bool> PN_NAME(test_o);

void run_cycle(int cycles = 1)
{
    for (int i = 0; i < cycles; i++)
    {
        clk_25 = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk_25 = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

int sc_main(int argc, char **argv)
{
    pn_cfg_enable_application_path = 1;               // enable application path in program args
    PN_PARSE_CMD_ARGS(argc, argv);                    // parse command line arguments
    sc_trace_file *tf = PN_BEGIN_TRACE("piconut_tb"); // create trace file

    // Initialliaze the Design under Testing (DUT)
    m_top dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.clk_25(clk_25);
    dut_inst.reset(reset);
    dut_inst.rx_i(rx_i);
    dut_inst.tx_o(tx_o);
    dut_inst.test_o(test_o);

    dut_inst.Trace(tf, pn_cfg_vcd_level); // trace signals of DUT

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Testbench code here

    reset = 1;
    run_cycle(2);
    reset = 0;

    while (dut_inst.piconut->state_is_not_halt()) {
        run_cycle();
    }

    // Testing if resetting the Nucleus works after completing the Hello World program

    reset = 1;
    run_cycle(10);
    reset = 0;
    run_cycle(10);

    cout << "\n\t\t*****Simulation waiting reset and repeat*****" << endl;

    while (dut_inst.piconut->state_is_not_halt()) {
        run_cycle();
    }

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}