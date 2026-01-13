/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lukas Bauer <lukas.bauer1@tha.de>
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


// Design template
#include "baudgen.h"
#include <stdint.h>
#include <systemc.h>

#define PERIOD_NS 10.0

// initialize TB signals
sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

sc_signal<bool> PN_NAME(en_i);
sc_signal<bool> PN_NAME(clear_i);
sc_signal<sc_uint<16>> PN_NAME(div_i);
sc_signal<bool> PN_NAME(baudtick_o);


void run_cycle(int cycles = 1){
    for (int i = 0; i < cycles; i++){
        clk = 0;
        sc_start(PERIOD_NS/2, SC_NS);
        clk = 1;
        sc_start(PERIOD_NS/2, SC_NS);
    }

}

int sc_main(int argc, char **argv)
{

    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file *tf = PN_BEGIN_TRACE("baudgen_tb");


    // Initialliaze the Design under Testing (DUT)
    m_baudgen dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.clk(clk);
    dut_inst.reset(reset);
    dut_inst.en_i(en_i);
    dut_inst.clear_i(clear_i);
    dut_inst.div_i(div_i);
    dut_inst.baudtick_o(baudtick_o);

    // Traces of signals here

    dut_inst.pn_trace(tf, pn_cfg_vcd_level); // trace signals of

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Testbench code here

    // Reset
    PN_INFO("Resetting ...");
    reset = 1;
    run_cycle();
    reset = 0;
    run_cycle();

    // Test divider for 10 cycles this should exceed the divider value and set the baudtick high
    PN_INFO("Testing divider (10 cycles) ...");
    div_i = 10;
    en_i = 1;
    run_cycle(11);
    PN_INFOF(("Baudtick: %d", baudtick_o.read()));
    PN_ASSERTM(baudtick_o.read() == 1, "Baudtick should be high after 10 cycles");

    // Test divider for 10 cycles again this to check if the next
    // baudtick is enabled
    run_cycle(11);
    PN_INFOF(("Baudtick: %d", baudtick_o.read()));
    PN_ASSERTM(baudtick_o.read() == 1, "Baudtick should be low after 10 cycles");

    // Change div to 20 and run for 10 cycles the divider should not be reached
    // and baudtick should be low
    PN_INFO("Testing divider (20 cycles) ...");
    div_i = 20;
    run_cycle(11);
    PN_INFOF(("Baudtick: %d", baudtick_o.read()));
    PN_ASSERTM(baudtick_o.read() == 0, "Baudtick should not be high after 10 cycles");

    // after 10 more cycles the divider should be reached and baudtick should be high
    run_cycle(10);
    PN_INFOF(("Baudtick: %d", baudtick_o.read()));
    PN_ASSERTM(baudtick_o.read() == 1, "Baudtick should be high after 20 cycles");

    // Test holding the value by disabling the en_i signal
    // the baudtick should be low
    PN_INFO("Testing holding the value 0 ...");
    en_i = 0;
    run_cycle(21);
    PN_INFOF(("Baudtick: %d", baudtick_o.read()));
    PN_ASSERTM(baudtick_o.read() == 0, "Baudtick should be low after 21 cycles. Because en_i is low");

    // Test holding the value 1 by enabling the en_i signal
    // the baudtick should be high
    PN_INFO("Testing holding the value 1 ...");
    en_i = 1;
    run_cycle(21);
    PN_INFOF(("Baudtick: %d", baudtick_o.read()));

    // Test clearing the counter
    // clearing the counter and waiting 20 cycles should set the baudtick to high
    // again because the divider is reached
    PN_INFO("Testing clearing the counter ...");
    clear_i = 1;
    run_cycle(21);
    PN_INFOF(("Baudtick: %d", baudtick_o.read()));
    PN_ASSERTM(baudtick_o.read() == 0, "Baudtick should be low. Because clear_i is high");

    // Test if setting en_i to 0 after reaching the divider value
    // this should set the baudtick to high and stay high for 1 cycle and then stay low

    PN_INFO("Testing setting en_i to 0 after reaching the divider value ...");
    clear_i = 0;
    en_i = 1;
    run_cycle(20);
    en_i = 0;
    run_cycle(1);
    PN_INFOF(("Baudtick: %d", baudtick_o.read()));
    PN_ASSERTM(baudtick_o.read() == 1, "Baudtick should be high. For 1 cycle");

    run_cycle();
    PN_INFOF(("Baudtick: %d", baudtick_o.read()));
    PN_ASSERTM(baudtick_o.read() == 0, "Baudtick should be low. After being high for 1 cycle");

    run_cycle(); // end with a wait

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}