
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


#include "byteselector.h"
#include <systemc.h>
#include <stdint.h>
#include <base.h>

#define PERIOD_NS 10.0

sc_signal<sc_uint<2>> PN_NAME(adr);
sc_signal<sc_uint<3>> PN_NAME(funct3);
sc_signal<sc_uint<4>> PN_NAME(bsel);
sc_signal<bool> PN_NAME(invalid_out);

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
    sc_trace_file* tf = PN_BEGIN_TRACE("byteselector_tb");

    m_byteselector dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.bsel_out(bsel);
    dut_inst.funct3_in(funct3);
    dut_inst.adr_in(adr);
    dut_inst.invalid_out(invalid_out);

    dut_inst.Trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT

    sc_start(SC_ZERO_TIME); // start simulation

    cout << "\n\t\t*****Simulation started*****" << endl;

    run_cycle(1);

    PN_INFOF(("-------- Testing lb/lh --------"));

    adr = 0x0;
    funct3 = 0x0;

    PN_INFOF(("Test 1: Read byte 0, expecting bsel '0001'"));
    run_cycle(1);
    PN_INFOF(("Test 1 result: '%x'", (uint32_t)bsel.read()));
    PN_ASSERTM((uint32_t)bsel.read() == 0x1, "Test 1 failed");

    adr = 0x1;
    PN_INFOF(("Test 2: Read byte 1, expecting bsel '0010'"));
    run_cycle(1);
    PN_INFOF(("Test 2 result: '%x'", (uint32_t)bsel.read()));
    PN_ASSERTM((uint32_t)bsel.read() == 0x2, "Test 2 failed");

    adr = 0x2;
    PN_INFOF(("Test 3: Read byte 2, expecting bsel '0100'"));
    run_cycle(1);
    PN_INFOF(("Test 3 result: '%x'", (uint32_t)bsel.read()));
    PN_ASSERTM((uint32_t)bsel.read() == 0x4, "Test 3 failed");

    adr = 0x3;
    PN_INFOF(("Test 4: Read byte 3, expecting bsel '1000'"));
    run_cycle(1);
    PN_INFOF(("Test 4 result: '%x'", (uint32_t)bsel.read()));
    PN_ASSERTM((uint32_t)bsel.read() == 0x8, "Test 4 failed");

    adr = 0x0;
    funct3 = 0x1;
    PN_INFOF(("Test 5: Read lower halfword, expecting bsel '0011'"));
    run_cycle(1);
    PN_INFOF(("Test 5 result: '%x'", (uint32_t)bsel.read()));
    PN_ASSERTM((uint32_t)bsel.read() == 0x3, "Test 5 failed");

    adr = 0x2;
    PN_INFOF(("Test 6: Read upper halfword, expecting bsel '1100'"));
    run_cycle(1);
    PN_INFOF(("Test 6 result: '%x'", (uint32_t)bsel.read()));
    PN_ASSERTM((uint32_t)bsel.read() == 0xc, "Test 6 failed");

    PN_INFOF(("\n\n"));
    PN_INFOF(("-------- Testing lbu/lhu --------"));

    adr = 0x0;
    funct3 = 0x4;
    PN_INFOF(("Test 8: Read byte 0, expecting bsel '0001'"));
    run_cycle(1);
    PN_INFOF(("Test 8 result: '%x'", (uint32_t)bsel.read()));
    PN_ASSERTM((uint32_t)bsel.read() == 0x1, "Test 8 failed");

    adr = 0x1;
    PN_INFOF(("Test 9: Read byte 1, expecting bsel '0010'"));
    run_cycle(1);
    PN_INFOF(("Test 9 result: '%x'", (uint32_t)bsel.read()));
    PN_ASSERTM((uint32_t)bsel.read() == 0x2, "Test 9 failed");

    adr = 0x2;
    PN_INFOF(("Test 10: Read byte 2, expecting bsel '0100'"));
    run_cycle(1);
    PN_INFOF(("Test 10 result: '%x'", (uint32_t)bsel.read()));
    PN_ASSERTM((uint32_t)bsel.read() == 0x4, "Test 10 failed");

    adr = 0x3;
    PN_INFOF(("Test 11: Read byte 3, expecting bsel '1000'"));
    run_cycle(1);
    PN_INFOF(("Test 11 result: '%x'", (uint32_t)bsel.read()));
    PN_ASSERTM((uint32_t)bsel.read() == 0x8, "Test 11 failed");

    adr = 0x0;
    funct3 = 0x5;
    PN_INFOF(("Test 12: Read lower halfword, expecting bsel '0011'"));
    run_cycle(1);
    PN_INFOF(("Test 12 result: '%x'", (uint32_t)bsel.read()));
    PN_ASSERTM((uint32_t)bsel.read() == 0x3, "Test 12 failed");

    adr = 0x2;
    PN_INFOF(("Test 13: Read upper halfword, expecting bsel '1100'"));
    run_cycle(1);
    PN_INFOF(("Test 13 result: '%x'", (uint32_t)bsel.read()));
    PN_ASSERTM((uint32_t)bsel.read() == 0xC, "Test 13 failed");

    PN_INFOF(("\n\n"));
    PN_INFOF(("-------- Testing lw --------"));
    adr = 0x0;
    funct3 = 0x2;
    PN_INFOF(("Test 14: Read word, expecting bsel '1111'"));
    run_cycle(1);
    PN_INFOF(("Test 14 result: '%x'", (uint32_t)bsel.read()));
    PN_ASSERTM((uint32_t)bsel.read() == 0xF, "Test 14 failed");

    PN_INFOF(("\n\n"));
    PN_INFOF(("-------- Testing invalid operations --------"));

    PN_INFOF(("\n\n"));
    adr = 0x1;
    funct3 = 0x1;
    PN_INFOF(("Test 15: Invalid byte alignment in lh/sh, expecting bsel 0 and invalid_out '1'"));
    run_cycle(1);
    PN_INFOF(("Test 15 result: '%x'", (uint32_t)bsel.read()));
    PN_INFOF(("Test 15 invalid_out: '%x'", (uint32_t)dut_inst.invalid_out.read()));
    PN_ASSERTM((uint32_t)bsel.read() == 0x0, "Test 16 failed");
    PN_ASSERTM((uint32_t)dut_inst.invalid_out.read() == 0x1, "Test 15 failed");

    PN_INFOF(("\n\n"));
    adr = 0x3;
    funct3 = 0x2;
    PN_INFOF(("Test 16: Invalid byte alignment in lw, expecting bsel 0 and invalid_out '1'"));
    run_cycle(1);
    PN_INFOF(("Test 16 result: '%x'", (uint32_t)bsel.read()));
    PN_INFOF(("Test 16 invalid_out: '%x'", (uint32_t)dut_inst.invalid_out.read()));
    PN_ASSERTM((uint32_t)bsel.read() == 0x0, "Test 17 failed");
    PN_ASSERTM((uint32_t)dut_inst.invalid_out.read() == 0x1, "Test 16 failed");

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}