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

#include "extender.h"
#include <systemc.h>
#include <stdint.h>
#include <piconut.h>

#define PERIOD_NS 10.0

sc_signal<sc_uint<32>> PN_NAME(data_in);
sc_signal<sc_uint<4>> PN_NAME(bsel_in);
sc_signal<sc_uint<3>> PN_NAME(funct3_in);
sc_signal<sc_uint<32>> PN_NAME(extend_out);

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
    sc_trace_file* tf = PN_BEGIN_TRACE("immgen_tb");

    m_extender dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.data_in(data_in);
    dut_inst.extend_out(extend_out);
    dut_inst.funct3_in(funct3_in);
    dut_inst.bsel_in(bsel_in);

    dut_inst.pn_trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT

    sc_start(SC_ZERO_TIME); // start simulation

    cout << "\n\t\t*****Simulation started*****" << endl;
    run_cycle();

    PN_INFOF(("-------- Testing lb/lh --------"));

    data_in = 0x12345678;
    bsel_in = BYTE_3;
    funct3_in = 0x0;
    PN_INFOF(("Test 1: Read upper byte from 0x12345678, expecting 0x00000012"));
    run_cycle(1);
    PN_INFOF(("Test 1 bystelect: 0x%08x", (uint32_t)bsel_in.read()));
    PN_INFOF(("Test 1 result: 0x%08x", (uint32_t)extend_out.read()));
    PN_ASSERTM((uint32_t)extend_out.read() == 0x00000012, "Test 1 failed");

    bsel_in = BYTE_2;
    PN_INFOF(("Test 2: Read third byte from 0x12345678, expecting 0x00000034"));
    run_cycle(1);
    PN_INFOF(("Test 2 bystelect: 0x%08x", (uint32_t)bsel_in.read()));
    PN_INFOF(("Test 2 result: 0x%08x", (uint32_t)extend_out.read()));
    PN_ASSERTM((uint32_t)extend_out.read() == 0x00000034, "Test 2 failed");

    bsel_in = BYTE_1;
    PN_INFOF(("Test 3: Read second byte from 0x12345678, expecting 0x00000056"));
    run_cycle(1);
    PN_INFOF(("Test 3 bystelect: 0x%08x", (uint32_t)bsel_in.read()));
    PN_INFOF(("Test 3 result: 0x%08x", (uint32_t)extend_out.read()));
    PN_ASSERTM((uint32_t)extend_out.read() == 0x00000056, "Test 3 failed");

    bsel_in = BYTE_0;
    PN_INFOF(("Test 4: Read lowest byte from 0x12345678, expecting 0x00000078"));
    run_cycle(1);
    PN_INFOF(("Test 4 bystelect: 0x%08x", (uint32_t)bsel_in.read()));
    PN_INFOF(("Test 4 result: 0x%08x", (uint32_t)extend_out.read()));
    PN_ASSERTM((uint32_t)extend_out.read() == 0x00000078, "Test 4 failed");

    bsel_in = HALF_UPPER;
    funct3_in = 0x1;
    PN_INFOF(("Test 5: Read upper halfword from 0x12345678, expecting 0x00001234"));
    run_cycle(1);
    PN_INFOF(("Test 5 bystelect: 0x%08x", (uint32_t)bsel_in.read()));
    PN_INFOF(("Test 5 result: 0x%08x", (uint32_t)extend_out.read()));
    PN_ASSERTM((uint32_t)extend_out.read() == 0x00001234, "Test 5 failed");

    bsel_in = HALF_LOWER;
    PN_INFOF(("Test 6: Read lower halfword from 0x12345678, expecting 0x00005678"));
    run_cycle(1);
    PN_INFOF(("Test 6 bystelect: 0x%08x", (uint32_t)bsel_in.read()));
    PN_INFOF(("Test 6 result: 0x%08x", (uint32_t)extend_out.read()));
    PN_ASSERTM((uint32_t)extend_out.read() == 0x00005678, "Test 6 failed");

    PN_INFOF(("\n\n"));
    PN_INFOF(("-------- Testing unsigned lbu/lhu --------"));

    data_in = 0xFF835DC2;
    bsel_in = BYTE_3;
    funct3_in = FUNCT3_LB;
    PN_INFOF(("Test 7: Read upper byte from 0xFF835DC2, expecting 0xFFFFFFFF"));
    run_cycle(1);
    PN_INFOF(("Test 7 result: 0x%08x", (uint32_t)extend_out.read()));
    PN_ASSERTM((uint32_t)extend_out.read() == 0xFFFFFFFF, "Test 7 failed");

    bsel_in = BYTE_2;
    PN_INFOF(("Test 8: Read third byte from 0xFF835DC2, expecting 0xFFFFFF83"));
    run_cycle(1);
    PN_INFOF(("Test 8 result: 0x%08x", (uint32_t)extend_out.read()));
    PN_ASSERTM((uint32_t)extend_out.read() == 0xFFFFFF83, "Test 8 failed");

    bsel_in = BYTE_1;
    PN_INFOF(("Test 9: Read second byte from 0xFF835DC2, expecting 0x0000005D"));
    run_cycle(1);
    PN_INFOF(("Test 9 result: 0x%08x", (uint32_t)extend_out.read()));
    PN_ASSERTM((uint32_t)extend_out.read() == 0x0000005D, "Test 9 failed");

    bsel_in = BYTE_0;
    PN_INFOF(("Test 10: Read lowest byte from 0xFF835DC2, expecting 0xFFFFFFC2"));
    run_cycle(1);
    PN_INFOF(("Test 10 result: 0x%08x", (uint32_t)extend_out.read()));
    PN_ASSERTM((uint32_t)extend_out.read() == 0xFFFFFFC2, "Test 10 failed");

    bsel_in = HALF_UPPER;
    funct3_in = FUNCT3_LH;
    PN_INFOF(("Test 11: Read upper halfword from 0xFF835DC2, expecting 0xFFFFFF83"));
    run_cycle(1);
    PN_INFOF(("Test 11 result: 0x%08x", (uint32_t)extend_out.read()));
    PN_ASSERTM((uint32_t)extend_out.read() == 0xFFFFFF83, "Test 11 failed");

    bsel_in = HALF_LOWER;
    PN_INFOF(("Test 12: Read lower halfword from 0xFF835DC2, expecting 0x00005DC2"));
    run_cycle(1);
    PN_INFOF(("Test 12 result: 0x%08x", (uint32_t)extend_out.read()));
    PN_ASSERTM((uint32_t)extend_out.read() == 0x00005DC2, "Test 12 failed");

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;
    return 0;
}