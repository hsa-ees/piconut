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
#include "majority_filter.h"
#include <stdint.h>
#include <systemc.h>

#define PERIOD_NS 10.0

// initialize TB signals
sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

// filter signals

sc_signal<bool> PN_NAME(filter_i); // Signal to be filtered
sc_signal<bool> PN_NAME(capture_i); // Enable Signal to capture the filter signal
sc_signal<bool> PN_NAME(clear_i); // Clear the filter signal
sc_signal<bool> PN_NAME(filter_o); // Filtered Signal


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
    sc_trace_file *tf = PN_BEGIN_TRACE("majority_filter_tb");


    // Initialliaze the Design under Testing (DUT)
    m_majority_filter dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.clk(clk);
    dut_inst.reset(reset);
    dut_inst.filter_i(filter_i);
    dut_inst.capture_i(capture_i);
    dut_inst.clear_i(clear_i);
    dut_inst.filter_o(filter_o);

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

    // Test that no output happens if no samples are captured
    // after 10 cycles the output should be still low
    // because the capture is disabled
    PN_INFO("Testing disabled capture ...");
    filter_i = 0;
    run_cycle(10);
    PN_INFOF(("filter_o: %d", filter_o.read()));
    PN_ASSERTM(filter_o.read() == 1, "No samples captured, output should be high");

    // Test that the output stays high  if capture is enabled but the input is high
    PN_INFO("Testing disabled capture with input high...");
    filter_i = 1;
    capture_i = 1;
    run_cycle(10);
    PN_INFOF(("filter_o: %d", filter_o.read()));
    PN_ASSERTM(filter_o.read() == 1, "No samples captured, output should be high");

    // Test that the output is low if the majority threshold is reached
    // for this the input needs to be low
    PN_INFO("Testing majority threshold reached...");
    capture_i = 1;
    filter_i = 0;
    run_cycle(11);
    PN_INFOF(("filter_o: %d", filter_o.read()));
    PN_ASSERTM(filter_o.read() == 0, "Majority threshold reached, output should be low");

    // Test if counting further does not overflow the counter
    // because the counter stoped when the threshold is reached
    PN_INFO("Testing if counter can overflow...");
    run_cycle(10);
    PN_INFOF(("filter_o: %d", filter_o.read()));
    PN_ASSERTM(filter_o.read() == 0, "Majority threshold reached, output should be low");

    capture_i = 0;
    filter_i = 0;


    // Test if the counter is reset if clear is high
    // it should revet the output to high
    PN_INFO("Testing reseting filter with clear...");
    clear_i = 1;
    run_cycle(2);
    PN_INFOF(("filter_o: %d", filter_o.read()));
    PN_ASSERTM(filter_o.read() == 1, "Clear is high, output should be high");

    // Test if counting to a majority then clearing then counting up again works
    // the output should be low after reaching the threshold and high after clearing the clear signal
    PN_INFO("Testing majority threshold reached after clearing...");
    clear_i = 0;
    filter_i = 0;
    capture_i = 1;
    run_cycle(11);
    PN_INFOF(("filter_o: %d", filter_o.read()));
    PN_ASSERTM(filter_o.read() == 0, "Majority threshold reached, output should be low");

    // doing the same again
    clear_i = 1;
    run_cycle();
    clear_i = 0;
    run_cycle(11);
    PN_INFOF(("filter_o: %d", filter_o.read()));
    PN_ASSERTM(filter_o.read() == 0, "Majority threshold reached, output should be low");

    filter_i = 0;
    capture_i = 0;

    // Test if reset clears the module if the reset is enabled
    PN_INFO("Testing reset...");
    reset = 1;
    run_cycle();
    reset = 0;
    PN_INFOF(("filter_o: %d", filter_o.read()));
    PN_ASSERTM(filter_o.read() == 1, "Reset is high, output should be high");

    run_cycle(); // end with a wait

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}