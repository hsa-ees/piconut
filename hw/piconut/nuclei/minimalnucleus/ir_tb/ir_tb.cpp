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



#include "ir.h"
#include <systemc.h>
#include <stdint.h>
#include <base.h>

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

sc_signal<sc_uint<32>> PN_NAME(ir_in);
sc_signal<sc_uint<32>> PN_NAME(ir_out);
sc_signal<bool> PN_NAME(en_load_in);

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
    sc_trace_file* tf = PN_BEGIN_TRACE("ir_tb");

    m_ir dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.clk(clk);
    dut_inst.reset(reset);

    dut_inst.ir_in(ir_in);
    dut_inst.ir_out(ir_out);
    dut_inst.en_load_in(en_load_in);

    dut_inst.Trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT

    sc_start(SC_ZERO_TIME); // start simulation

    cout << "\n\t\t*****Simulation started*****" << endl;

    run_cycle();
    PN_INFOF(("Set reset high"));
    reset = 1;
    run_cycle();
    PN_INFOF(("Set reset low"));
    reset = 0;
    run_cycle();
    PN_INFOF(("Test 1: Check if reset value is 0x0"));
    PN_INFOF(("ir_out: 0x%x", (uint32_t)ir_out.read()));
    PN_ASSERTM(ir_out.read() == 0, "Test 1 failed");
    PN_INFOF(("Test 1: Passed"));
    PN_INFOF(("\n"));

    PN_INFOF(("Test 2: Load value into IR, check if value is loaded"));
    ir_in = 0xffe0;
    en_load_in = 1;
    run_cycle();
    PN_INFOF(("ir_in: 0x%x", (uint32_t)ir_in.read()));
    PN_INFOF(("ir_out: 0x%x", (uint32_t)ir_out.read()));
    PN_ASSERTM(ir_out.read() == 0xffe0, "Test 2 failed");
    PN_INFOF(("Test 2: Passed"));
    PN_INFOF(("\n"));

    PN_INFOF(("Test 3: Disable load enable, check if value is held"));
    en_load_in = 0;
    run_cycle();
    PN_INFOF(("ir_out: 0x%x", (uint32_t)ir_out.read()));
    PN_ASSERTM(ir_out.read() == 0xffe0, "Test 3 failed");
    PN_INFOF(("Test 3: Passed"));
    PN_INFOF(("\n"));

    PN_INFOF(("Test 4: Set reset low then high, check if IR is reset to 0"));
    reset = 0;
    run_cycle();
    reset = 1;
    run_cycle();
    PN_INFOF(("ir_out: 0x%x", (uint32_t)ir_out.read()));
    PN_ASSERTM(ir_out.read() == 0, "Test 4 failed");
    PN_INFOF(("Test 4: Passed"));

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}