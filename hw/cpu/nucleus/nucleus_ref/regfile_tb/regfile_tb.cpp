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



#include "regfile.h"
#include <systemc.h>
#include <stdint.h>
#include <piconut.h>

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

sc_signal<bool> PN_NAME(en_load_in);
sc_signal<sc_uint<32>> PN_NAME(data_in);
sc_signal<sc_uint<5>> PN_NAME(select_in);
sc_signal<sc_uint<5>> PN_NAME(rs1_select_in);
sc_signal<sc_uint<5>> PN_NAME(rs2_select_in);

sc_signal<sc_uint<32>> PN_NAME(rs1_out);
sc_signal<sc_uint<32>> PN_NAME(rs2_out);

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
    sc_trace_file* tf = PN_BEGIN_TRACE("regfile_tb");

    m_regfile dut_inst{"dut_inst"};

    dut_inst.clk(clk);
    dut_inst.reset(reset);
    dut_inst.en_load_in(en_load_in);
    dut_inst.data_in(data_in);
    dut_inst.select_in(select_in);
    dut_inst.rs1_select_in(rs1_select_in);
    dut_inst.rs2_select_in(rs2_select_in);
    dut_inst.rs1_out(rs1_out);
    dut_inst.rs2_out(rs2_out);

    dut_inst.pn_trace(tf, pn_cfg_vcd_level);

    sc_start(SC_ZERO_TIME);

    cout << "\n\t\t*****Simulation started*****" << endl;

    run_cycle();
    reset = 1;
    run_cycle();
    reset = 0;
    run_cycle();
    en_load_in = 1;
    PN_INFOF(("Iterate through all 32 registers and load iterator value (starting at 1)"));
    for(size_t i = 1; i < 32; i++)
    {
        data_in = i;
        select_in = i;
        rs1_select_in = i;
        rs2_select_in = i;
        run_cycle();
    }
    run_cycle();
    PN_INFOF(("Test 1: Check if x0 has value 0x0"));
    rs1_select_in = 0;
    rs2_select_in = 0;
    run_cycle();
    PN_INFOF(("rs1_select_in: 0x%x", (uint32_t)rs1_select_in.read()));
    PN_INFOF(("rs2_select_in: 0x%x", (uint32_t)rs2_select_in.read()));
    PN_INFOF(("rs1_out: %d", (uint32_t)rs1_out.read()));
    PN_INFOF(("rs2_out: %d", (uint32_t)rs2_out.read()));
    PN_ASSERTM(rs1_out.read() == 0, "Test 1 failed");
    PN_ASSERTM(rs2_out.read() == 0, "Test 1 failed");
    PN_INFOF(("Test 1: Passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("Test 2: Check if all registers have the correct value"));
    for(size_t i = 1; i < 32; i++)
    {
        PN_INFOF(("Checking register x%d", i));
        rs1_select_in = i;
        rs2_select_in = i;
        run_cycle();
        PN_INFOF(("rs1_select_in: 0x%x", (uint32_t)rs1_select_in.read()));
        PN_INFOF(("rs2_select_in: 0x%x", (uint32_t)rs2_select_in.read()));
        PN_INFOF(("rs1_out: %d", (uint32_t)rs1_out.read()));
        PN_INFOF(("rs2_out: %d", (uint32_t)rs2_out.read()));
        PN_ASSERTM(rs1_out.read() == i, "Test 2 failed");
        PN_ASSERTM(rs2_out.read() == i, "Test 2 failed");
        PN_INFOF(("\n"));
    }
    PN_INFOF(("Test 2: Passed"));
    PN_INFOF(("\n"));

    PN_INFOF(("Test 3: Attempt loading value into x0, check value is still 0x0"));
    select_in = 0;
    data_in = 0x12345678;
    en_load_in = 1;
    rs1_select_in = 0;
    rs2_select_in = 0;
    PN_INFOF(("select_in: %d", (uint32_t)select_in.read()));
    PN_INFOF(("data_in: 0x%x", (uint32_t)data_in.read()));
    PN_INFOF(("rs1_select_in: %d", (uint32_t)rs1_select_in.read()));
    PN_INFOF(("rs2_select_in: %d", (uint32_t)rs2_select_in.read()));
    run_cycle();
    PN_INFOF(("rs1_out: %d", (uint32_t)rs1_out.read()));
    PN_INFOF(("rs2_out: %d", (uint32_t)rs2_out.read()));
    PN_ASSERTM(rs1_out.read() == 0, "Test 3 failed");
    PN_ASSERTM(rs2_out.read() == 0, "Test 3 failed");
    PN_INFOF(("Test 3: Passed"));
    PN_INFOF(("\n"));

    PN_INFOF(("Test 4: Set reset to 1, check if all registers are reset to 0"));
    reset = 1;
    run_cycle();
    reset = 0;
    run_cycle();
    for(size_t i = 0; i < 32; i++)
    {
        rs1_select_in = i;
        PN_INFOF(("rs1_select_in: 0x%x", (uint32_t)rs1_select_in.read()));
        PN_INFOF(("rs2_select_in: 0x%x", (uint32_t)rs2_select_in.read()));
        rs2_select_in = i;
        run_cycle();
        PN_INFOF(("rs1_out: %d", (uint32_t)rs1_out.read()));
        PN_INFOF(("rs2_out: %d", (uint32_t)rs2_out.read()));
        PN_ASSERTM(rs1_out.read() == 0, "Test 4 failed");
        PN_ASSERTM(rs2_out.read() == 0, "Test 4 failed");
        PN_INFOF(("\n"));
    }
    PN_INFOF(("Test 4: Passed"));
    PN_INFOF(("\n"));

    PN_INFOF(("Test 5: Load value into x1, attempt to write new value without setting load enable, check if value is still the same"));
    PN_INFOF(("Load single value into x1"));
    en_load_in = 1;
    data_in = 0x12345678;
    select_in = 1;
    rs1_select_in = 1;
    rs2_select_in = 1;
    run_cycle();
    PN_INFOF(("rs1_out: 0x%x", (uint32_t)rs1_out.read()));
    PN_INFOF(("Attempt to write new value, dont set en_load_in"));
    en_load_in = 0;
    data_in = 0x87654321;
    run_cycle();
    PN_INFOF(("rs1_out: 0x%x", (uint32_t)rs1_out.read()));
    PN_ASSERTM(rs1_out.read() == 0x12345678, "Test 5 failed");
    PN_INFOF(("Test 5: Passed"));
    PN_INFOF(("\n"));

    PN_INFOF(("Test 6: Write new value to x1 with load enable, check if value is updated"));
    PN_INFOF(("Load single value into x1"));
    en_load_in = 1;
    data_in = 0xFFFFFFFF;
    select_in = 1;
    rs1_select_in = 1;
    rs2_select_in = 1;
    run_cycle();
    PN_INFOF(("rs1_out: 0x%x", (uint32_t)rs1_out.read()));
    PN_ASSERTM(rs1_out.read() == 0xFFFFFFFF, "Test 6 failed");
    PN_INFOF(("Test 6: Passed"));
    PN_INFOF(("\n"));

    PN_INFOF(("Test 7: Set reset low then high, check if all registers are reset to 0"));
    reset = 0;
    run_cycle();
    reset = 1;
    run_cycle();
    for(size_t i = 0; i < 32; i++)
    {
        rs1_select_in = i;
        rs2_select_in = i;
        run_cycle();
        PN_ASSERTM(rs1_out.read() == 0, "Test 7 failed");
        PN_ASSERTM(rs2_out.read() == 0, "Test 7 failed");
    }
    PN_INFOF(("Test 7: Passed"));

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}