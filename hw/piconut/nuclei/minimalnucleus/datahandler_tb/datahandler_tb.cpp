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
#include "datahandler.h"
#include <systemc.h>
#include <stdint.h>
#include <base.h>

#define PERIOD_NS 10.0

sc_signal<sc_uint<32>> PN_NAME(data_in);
sc_signal<sc_uint<4>> PN_NAME(bsel_in);
sc_signal<sc_uint<32>> PN_NAME(data_out);

void run_cycle(int cycles = 1)
{
    for(int i = 0; i < cycles; i++)
    {

        sc_start(PERIOD_NS, SC_NS);
    }
}

int sc_main(int argc, char** argv)
{
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("datahandler_tb");

    m_datahandler dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.data_in(data_in);
    dut_inst.data_out(data_out);
    dut_inst.bsel_in(bsel_in);

    dut_inst.Trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT

    sc_start(SC_ZERO_TIME); // start simulation

    cout << "\n\t\t*****Simulation started*****" << endl;
    run_cycle(1);
    // Test 1
    data_in = 0x12345678;
    bsel_in.write(BYTE_0);
    PN_INFOF(("------- Test 1 -------"));
    run_cycle();
    PN_INFOF(("bsel     = 0x%x", (uint32_t)bsel_in.read()));
    PN_INFOF(("data_in  = 0x%08x", (uint32_t)data_in.read()));
    PN_INFOF(("data_out = 0x%08x", (uint32_t)data_out.read()));
    PN_ASSERTM(data_out.read() == 0x78, "Test 1 failed");

    bsel_in.write(BYTE_1);
    PN_INFOF(("------- Test 2 -------"));
    run_cycle();
    PN_INFOF(("bsel     = 0x%x", (uint32_t)bsel_in.read()));
    PN_INFOF(("data_in  = 0x%08x", (uint32_t)data_in.read()));
    PN_INFOF(("data_out = 0x%08x", (uint32_t)data_out.read()));
    PN_ASSERTM(data_out.read() == 0x7800, "Test 2 failed");

    bsel_in.write(BYTE_2);
    PN_INFOF(("------- Test 3 -------"));
    run_cycle();
    PN_INFOF(("bsel     = 0x%x", (uint32_t)bsel_in.read()));
    PN_INFOF(("data_in  = 0x%08x", (uint32_t)data_in.read()));
    PN_INFOF(("data_out = 0x%08x", (uint32_t)data_out.read()));
    PN_ASSERTM(data_out.read() == 0x780000, "Test 3 failed");

    bsel_in.write(BYTE_3);
    PN_INFOF(("------- Test 4 -------"));
    run_cycle();
    PN_INFOF(("bsel     = 0x%x", (uint32_t)bsel_in.read()));
    PN_INFOF(("data_in  = 0x%08x", (uint32_t)data_in.read()));
    PN_INFOF(("data_out = 0x%08x", (uint32_t)data_out.read()));
    PN_ASSERTM(data_out.read() == 0x78000000, "Test 4 failed");

    bsel_in.write(HALF_LOWER);
    PN_INFOF(("------- Test 5 -------"));
    run_cycle();
    PN_INFOF(("bsel     = 0x%x", (uint32_t)bsel_in.read()));
    PN_INFOF(("data_in  = 0x%08x", (uint32_t)data_in.read()));
    PN_INFOF(("data_out = 0x%08x", (uint32_t)data_out.read()));
    PN_ASSERTM(data_out.read() == 0x5678, "Test 5 failed");

    bsel_in.write(HALF_UPPER);
    PN_INFOF(("------- Test 6 -------"));
    run_cycle();
    PN_INFOF(("bsel     = 0x%x", (uint32_t)bsel_in.read()));
    PN_INFOF(("data_in  = 0x%08x", (uint32_t)data_in.read()));
    PN_INFOF(("data_out = 0x%08x", (uint32_t)data_out.read()));
    PN_ASSERTM(data_out.read() == 0x56780000, "Test 6 failed");

    bsel_in.write(WORD);
    PN_INFOF(("------- Test 7 -------"));
    run_cycle();
    PN_INFOF(("bsel     = 0x%x", (uint32_t)bsel_in.read()));
    PN_INFOF(("data_in  = 0x%08x", (uint32_t)data_in.read()));
    PN_INFOF(("data_out = 0x%08x", (uint32_t)data_out.read()));
    PN_ASSERTM(data_out.read() == 0x12345678, "Test 7 failed");

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;
    return 0;
}