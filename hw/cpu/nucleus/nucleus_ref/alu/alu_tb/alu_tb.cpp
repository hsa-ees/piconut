/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lorenz Sommer <lorenz.sommer@tha.de>
                     2025 Niklas Sirch  <niklas.sirch1@tha.de>
                     2026 Tristan Kundrat <tristan.kundrat@tha.de>
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

#include "../alu.h"

#include <cstdint>
#include <sysc/datatypes/int/sc_nbdefs.h>
#include <systemc.h>
#include <stdint.h>
#include <piconut.h>

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);
sc_signal<sc_uint<32>> PN_NAME(a);          // Operand A
sc_signal<sc_uint<32>> PN_NAME(b);          // Operand B
sc_signal<sc_uint<3>> PN_NAME(funct3);      // selects operation
sc_signal<sc_uint<32>> PN_NAME(y);          // Result Y (actual)
sc_signal<sc_uint<32>> PN_NAME(y_expected); // Result Y (expected)
sc_signal<bool> PN_NAME(force_add);
sc_signal<bool> PN_NAME(equal);        // Equal flag
sc_signal<bool> PN_NAME(less);         // Less flag
sc_signal<bool> PN_NAME(lessu);        // Less unsigned flag
sc_signal<bool> PN_NAME(valid);        // Valid flag
sc_signal<sc_uint<7>> PN_NAME(funct7); // funct7 flag - This flag is derived from IR[31:25]
sc_signal<sc_uint<3>> PN_NAME(alu_mode);
sc_signal<bool> PN_NAME(force_amo);

void run_cycle(int cycles = 1)
{
    for(int i = 0; i < cycles; i++)
    {
        clk = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

int sc_main(int argc, char** argv)
{

    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("alu_tb");

    m_alu i_dut{"i_dut"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB

    i_dut.clk(clk);
    i_dut.reset(reset);
    i_dut.a_in(a);
    i_dut.b_in(b);
    i_dut.funct7_in(funct7);
    i_dut.funct3_in(funct3);
    i_dut.y_out(y);
    i_dut.force_add_in(force_add);
    i_dut.equal_out(equal);
    i_dut.less_out(less);
    i_dut.lessu_out(lessu);
    i_dut.valid_out(valid);
    i_dut.alu_mode_in(alu_mode);
    i_dut.force_amo_in(force_amo); // Connect force_amo signal

    i_dut.pn_trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT

    sc_start(SC_ZERO_TIME); // start simulation

    cout << "\n\t\t*****Simulation started*****" << endl;
    PN_INFOF(("Resetting for 10 cycles..."));
    reset = 1;
    run_cycle(10);
    reset = 0;
    PN_INFOF(("Done resetting."));
    run_cycle();

    PN_INFOF(("------- Test 1 -------"));
    PN_INFOF(("Test 1: 0x5 + 0x5, expecting 0xA"));
    funct7 = 0x0;
    a = 0x5;
    b = 0x5;
    y_expected = 0xA;
    alu_mode = ALU_MODE_REG_REG;
    funct7 = FUNCT7_RV32I_BASE;
    funct3 = FUNCT3_ADD_SUB_MUL;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 1 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 1 failed");
    PN_INFOF(("Test 1 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 2 -------"));
    PN_INFOF(("Test 2: Subtract 0xB - 0x5, expecting 0x6"));
    funct7 = 0x20;
    a = 0xB;
    b = 0x5;
    y_expected = 0x6;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 2 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 2 failed");
    PN_INFOF(("Test 2 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 3 -------"));
    PN_INFOF(("Test 3: 0x2 - 0x3, expecting underflow 0xFFFFFFFF"));
    funct7 = 0x20;
    a = 0x2;
    b = 0x3;
    y_expected = (unsigned)0x2 - (unsigned)0x3;
    funct3 = FUNCT3_ADD_SUB_MUL;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 3 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 3 failed.");
    PN_INFOF(("Test 3 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 4 -------"));
    PN_INFOF(("Test 4: '10101' & '00101', expecting '00001'"));
    funct3 = FUNCT3_AND_REMU;
    funct7 = 0x0;
    a = 0b10101;
    b = 0b00101;
    y_expected = 0b10101 & 0b00101;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 4 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 4 failed.");
    PN_INFOF(("Test 4 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 5 -------"));
    PN_INFOF(("Test 5: '110' | '001', expecting '111'"));
    funct3 = FUNCT3_OR_REM;
    a = 0b110;
    b = 0b001;
    y_expected = 0b110 | 0b001;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 5 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 5 failed.");
    PN_INFOF(("Test 5 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 6 -------"));
    PN_INFOF(("Test 6: '10101' ^ '10100', expecting '00001'"));
    funct3 = FUNCT3_XOR_DIV;
    a = 0b10101;
    b = 0b10100;
    y_expected = 0b10101 ^ 0b10100;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 6 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 6 failed.");
    PN_INFOF(("Test 6 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 7 -------"));
    PN_INFOF(("Test 7: (-5) < 10 ? 1:0, expecting 1, expecting less_out = 1"));
    a = -5;
    b = 10;
    y_expected = ((int32_t)-5 < (int32_t)10) ? 1 : 0;
    funct3 = FUNCT3_SLT_MULHSU;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 7 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: %d", (int32_t)a.read()));
    PN_INFOF(("Operand B: %d", (int32_t)b.read()));
    PN_INFOF(("Result Y: %d", (int32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 7 failed.");
    PN_ASSERTM((uint32_t)less.read() == 1, "Test 7 failed.");
    PN_INFOF(("Test 7 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 8 -------"));
    PN_INFOF(("Test 8: 600 < (-10) ? 1:0, expecting 0, expecting less_out = 0"));
    a = 600;
    b = -10;
    y_expected = ((int32_t)600 < (int32_t)-10) ? 1 : 0;
    funct3 = FUNCT3_SLT_MULHSU;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 8 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: %d", (int32_t)a.read()));
    PN_INFOF(("Operand B: %d", (int32_t)b.read()));
    PN_INFOF(("Result Y: %d", (int32_t)y.read()));
    PN_INFOF(("Less flag: %d", (uint32_t)less.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 8 failed.");
    PN_ASSERTM((uint32_t)less.read() == 0, "Test 8 failed.");
    PN_INFOF(("Test 8 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 9 -------"));
    PN_INFOF(("Test 9: (-100) < (-10) ? 1:0, expecting 1, expecting less_out = 1"));
    a = -100;
    b = -10;
    y_expected = ((int32_t)-100 < (int32_t)-10) ? 1 : 0;
    funct3 = FUNCT3_SLT_MULHSU;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 9 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: %d", (int32_t)a.read()));
    PN_INFOF(("Operand B: %d", (int32_t)b.read()));
    PN_INFOF(("Result Y: %d", (int32_t)y.read()));
    PN_INFOF(("Less flag: %d", (uint32_t)less.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 9 failed.");
    PN_ASSERTM((uint32_t)less.read() == 1, "Test 9 failed.");
    PN_INFOF(("Test 9 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 10 -------"));
    PN_INFOF(("Test 10: (unsigned)(-5) < 10 ? 1:0 , expecting 0, expecting lessu_out = 0"));
    a = -5;
    b = 10;
    y_expected = ((uint32_t)-5 < (uint32_t)10) ? 1 : 0;
    funct3 = FUNCT3_SLTU_MULHU;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 10 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: 0x%x", (int32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (int32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (int32_t)y.read()));
    PN_INFOF(("Lessu flag: %d", (uint32_t)lessu.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 10 failed.");
    PN_ASSERTM((uint32_t)lessu.read() == 0, "Test 10 failed.");
    PN_INFOF(("Test 10 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 11 -------"));
    PN_INFOF(("Test 11: 600 < (unsigned)(-10) ? 1:0, expecting 1, expecting lessu_out = 1"));
    a = 600;
    b = -10;
    y_expected = ((uint32_t)600 < (uint32_t)-10) ? 1 : 0;
    funct3 = FUNCT3_SLTU_MULHU;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 11 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: 0x%x", (int32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (int32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (int32_t)y.read()));
    PN_INFOF(("Lessu flag: %d", (uint32_t)lessu.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 11 failed.");
    PN_ASSERTM((uint32_t)lessu.read() == 1, "Test 11 failed.");
    PN_INFOF(("Test 11 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 12 -------"));
    PN_INFOF(("Test 12: 10 < 10 ? 1:0, expecting 0, expecting equal_out = 1"));
    a = 10;
    b = 10;
    y_expected = ((int32_t)10 < (int32_t)10) ? 1 : 0;
    funct3 = FUNCT3_SLT_MULHSU;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 12 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: %d", (int32_t)a.read()));
    PN_INFOF(("Operand B: %d", (int32_t)b.read()));
    PN_INFOF(("Result Y: %d", (int32_t)y.read()));
    PN_INFOF(("Equal flag: %d", (uint32_t)equal.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 12 failed.");
    PN_ASSERTM((uint32_t)equal.read() == 1, "Test 12 failed.");
    PN_INFOF(("Test 12 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 13 -------"));
    PN_INFOF(("Test 12: '0001000' << 1, expecting '0010000'"));
    a = 0x0001000;
    b = 0x1;
    y_expected = 0x0001000 << 0x1;
    funct3 = FUNCT3_SLL_MULH;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 13 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 13 failed.");
    PN_INFOF(("Test 13 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 14 -------"));
    PN_INFOF(("Test 14: '1000' >> 3, expecting '0001', logical shift zero extends"));
    a = 8;
    b = 3;
    y_expected = 8 >> 3;
    funct3 = FUNCT3_SRL_SRA_DIVU;
    funct7 = 0x20;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 14 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: %d", (uint32_t)a.read()));
    PN_INFOF(("Operand B: %d", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 14 failed.");
    PN_INFOF(("Test 14 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 15 -------"));
    PN_INFOF(("Test 15: '1' >> 1, expecting '0x0', logical shift zero extends"));
    a = 0x1;
    b = 0x1;
    y_expected = 0x1 >> 0x1;
    funct7 = 0;
    funct3 = FUNCT3_SRL_SRA_DIVU;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 15 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: %d", (uint32_t)a.read()));
    PN_INFOF(("Operand B: %d", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 15 failed.");
    PN_INFOF(("Test 15 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 16 -------"));
    PN_INFOF(("Test 16: '0xFAFAFAFA' >> 2, expecting '0xFEBEBEBE', arithmetic shift sign extends"));
    a = 0xFAFAFAFA;
    b = 0x2;
    y_expected = 0xFEBEBEBE;
    funct3 = FUNCT3_SRL_SRA_DIVU;
    funct7 = 0x20;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 16 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: %x", (int32_t)a.read()));
    PN_INFOF(("Operand B: %d", (int32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (int32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 16 failed.");
    PN_INFOF(("Test 16 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 17 -------"));
    PN_INFOF(("Test 17: force_amo MIN (signed) (-5, 10), expecting -5"));
    force_amo = 1;
    force_add = 0;
    alu_mode = 0x0; // Should be ignored
    funct3 = 0x0;   // Should be ignored
    funct7 = 0x40;  // FUNCT5A_MIN (0b10000) << 2
    a = -5;
    b = 10;
    y_expected = -5;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 17 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: %d", (int32_t)a.read()));
    PN_INFOF(("Operand B: %d", (int32_t)b.read()));
    PN_INFOF(("Result Y: %d", (int32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 17a failed");

    PN_INFOF(("Test 17: force_amo MIN (signed) (10, -5), expecting -5"));
    a = 10;
    b = -5;
    y_expected = -5;
    run_cycle();
    PN_INFOF(("Operand A: %d", (int32_t)a.read()));
    PN_INFOF(("Operand B: %d", (int32_t)b.read()));
    PN_INFOF(("Result Y: %d", (int32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 17b failed");
    PN_INFOF(("Test 17 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 18 -------"));
    PN_INFOF(("Test 18: force_amo MAX (signed) (-5, 10), expecting 10"));
    force_amo = 1;
    force_add = 0;
    alu_mode = 0x0;
    funct3 = 0x0;
    funct7 = 0x50; // FUNCT5A_MAX (0b10100) << 2
    a = -5;
    b = 10;
    y_expected = 10;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 18 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: %d", (int32_t)a.read()));
    PN_INFOF(("Operand B: %d", (int32_t)b.read()));
    PN_INFOF(("Result Y: %d", (int32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 18a failed");

    PN_INFOF(("Test 18: force_amo MAX (signed) (10, -5), expecting 10"));
    a = 10;
    b = -5;
    y_expected = 10;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 18 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: %d", (int32_t)a.read()));
    PN_INFOF(("Operand B: %d", (int32_t)b.read()));
    PN_INFOF(("Result Y: %d", (int32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 18b failed");
    PN_INFOF(("Test 18 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 19 -------"));
    PN_INFOF(("Test 19: force_amo MINU (unsigned) (5, 10), expecting 5"));
    force_amo = 1;
    force_add = 0;
    alu_mode = 0x0;
    funct3 = 0x0;
    funct7 = 0x60; // FUNCT5A_MINU (0b11000) << 2
    a = 5;
    b = 10;
    y_expected = 5;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 19 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 19a failed");

    PN_INFOF(("Test 19: force_amo MINU (unsigned) (0xFFFFFFFB, 10), expecting 10"));
    a = 0xFFFFFFFB; // Unsigned large value
    b = 10;
    y_expected = 10;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 19 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 19b failed");
    PN_INFOF(("Test 19 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 20 -------"));
    PN_INFOF(("Test 20: force_amo MAXU (unsigned) (5, 10), expecting 10"));
    force_amo = 1;
    force_add = 0;
    alu_mode = 0x0;
    funct3 = 0x0;
    funct7 = 0x70; // FUNCT5A_MAXU (0b11100) << 2
    a = 5;
    b = 10;
    y_expected = 10;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 20 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 20a failed");

    PN_INFOF(("Test 20: force_amo MAXU (unsigned) (0xFFFFFFFB, 10), expecting 0xFFFFFFFB"));
    a = 0xFFFFFFFB; // Unsigned large value
    b = 10;
    y_expected = 0xFFFFFFFB;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 20 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 20b failed");
    PN_INFOF(("Test 20 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 21 -------"));
    PN_INFOF(("Test 21: force_amo ADD (5, 10), expecting 15"));
    force_amo = 1;
    force_add = 0;
    alu_mode = 0x0;            // Should be ignored
    funct3 = 0x0;              // Should be ignored
    funct7 = FUNCT5A_ADD << 2; // FUNCT5A_ADD (0b00000) << 2
    a = 5;
    b = 10;
    y_expected = 15;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 21 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 21 failed");
    PN_INFOF(("Test 21 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 22 -------"));
    PN_INFOF(("Test 22: force_amo AND (0b1010, 0b1100), expecting 0b1000"));
    force_amo = 1;
    force_add = 0;
    alu_mode = 0x0;
    funct3 = 0x0;
    funct7 = FUNCT5A_AND << 2; // FUNCT5A_AND (0b01100) << 2
    a = 0b1010;
    b = 0b1100;
    y_expected = 0b1010 & 0b1100;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 22 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 22 failed");
    PN_INFOF(("Test 22 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 23 -------"));
    PN_INFOF(("Test 23: force_amo OR (0b1010, 0b1100), expecting 0b1110"));
    force_amo = 1;
    force_add = 0;
    alu_mode = 0x0;
    funct3 = 0x0;
    funct7 = FUNCT5A_OR << 2; // FUNCT5A_OR (0b01000) << 2
    a = 0b1010;
    b = 0b1100;
    y_expected = 0b1010 | 0b1100;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 23 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 23 failed");
    PN_INFOF(("Test 23 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 24 -------"));
    PN_INFOF(("Test 24: force_amo XOR (0b1010, 0b1100), expecting 0b0110"));
    force_amo = 1;
    force_add = 0;
    alu_mode = 0x0;
    funct3 = 0x0;
    funct7 = FUNCT5A_XOR << 2; // FUNCT5A_XOR (0b00100) << 2
    a = 0b1010;
    b = 0b1100;
    y_expected = 0b1010 ^ 0b1100;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 24 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 24 failed");
    PN_INFOF(("Test 24 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 25 -------"));
    PN_INFOF(("Test 25: force_amo  (0b1111, 0b0101), expecting 0b0101"));
    force_amo = 1;
    force_add = 0;
    alu_mode = 0x0;
    funct3 = 0x0;
    funct7 = FUNCT5A_SWAP << 2;
    a = 0b1111;
    b = 0b0101;
    y_expected = 0b0101;
    run_cycle();
    PN_ASSERTM(valid.read() == 1, "Test 25 timed out (signal valid not 1 after 1 cycle)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 25 failed");
    PN_INFOF(("Test 25 passed"));

#if PN_CFG_ALU_ENABLE_ZMMUL_EXTENSION == 1
    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 26 -------"));
    PN_INFOF(("Test 26: mul  (-777, +21)"));
    force_amo = 0;
    alu_mode = ALU_MODE_REG_REG;
    funct3 = FUNCT3_ADD_SUB_MUL;
    funct7 = FUNCT7_M_EXTENSION;
    a = -777;
    b = 21;
    y_expected = -777 * 21;
    for(int i = 0; i < 100; i++)
    {
        run_cycle();
        if(valid.read() == 1)
            break;
    }
    PN_ASSERTM(valid.read() == 1, "Test 26 timed out (signal valid not 1 after 100 cycles)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 26 failed");
    PN_INFOF(("Test 26 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 27 -------"));
    PN_INFOF(("Test 27a: mulh  (500000, 400000)"));
    alu_mode = ALU_MODE_REG_REG;
    funct3 = FUNCT3_SLL_MULH;
    funct7 = FUNCT7_M_EXTENSION;
    a = 500000;
    b = 400000;
    y_expected = (uint64_t)((int64_t)500000 * (int64_t)400000) >> 32;
    for(int i = 0; i < 100; i++)
    {
        run_cycle();
        if(valid.read() == 1)
            break;
    }
    PN_ASSERTM(valid.read() == 1, "Test 27a timed out (signal valid not 1 after 100 cycles)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 27a failed");
    PN_INFOF(("Test 27a passed"));

    // reset alu_mode, so start_in for mul/div is zero
    alu_mode = ALU_MODE_IDLE;
    run_cycle();

    PN_INFOF(("\n"));
    PN_INFOF(("Test 27b: mulh  (-500000, -400000)"));
    alu_mode = ALU_MODE_REG_REG;
    funct3 = FUNCT3_SLL_MULH;
    funct7 = FUNCT7_M_EXTENSION;
    a = -500000;
    b = -400000;
    y_expected = (uint64_t)((int64_t)-500000 * (int64_t)-400000) >> 32;
    for(int i = 0; i < 100; i++)
    {
        run_cycle();
        if(valid.read() == 1)
            break;
    }
    PN_ASSERTM(valid.read() == 1, "Test 27b timed out (signal valid not 1 after 100 cycles)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 27b failed");
    PN_INFOF(("Test 27b passed"));

    // reset alu_mode, so start_in for mul/div is zero
    alu_mode = ALU_MODE_IDLE;
    run_cycle();

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 28 -------"));
    PN_INFOF(("Test 28: mulhu  (500000, 400000)"));
    alu_mode = ALU_MODE_REG_REG;
    funct3 = FUNCT3_SLTU_MULHU;
    funct7 = FUNCT7_M_EXTENSION;
    a = 500000;
    b = 400000;
    y_expected = (uint64_t)((uint64_t)500000 * (uint64_t)400000) >> 32;
    for(int i = 0; i < 100; i++)
    {
        run_cycle();
        if(valid.read() == 1)
            break;
    }
    PN_ASSERTM(valid.read() == 1, "Test 28 timed out (signal valid not 1 after 100 cycles)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 28 failed");
    PN_INFOF(("Test 28 passed"));

    // reset alu_mode, so start_in for mul/div is zero
    alu_mode = ALU_MODE_IDLE;
    run_cycle();

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 29 -------"));
    PN_INFOF(("Test 29: mulhsu  (-500000, 400000)"));
    alu_mode = ALU_MODE_REG_REG;
    funct3 = FUNCT3_SLT_MULHSU;
    funct7 = FUNCT7_M_EXTENSION;
    a = -500000;
    b = 400000;
    y_expected = (uint64_t)((int64_t)-500000 * (uint64_t)400000) >> 32;
    for(int i = 0; i < 100; i++)
    {
        run_cycle();
        if(valid.read() == 1)
            break;
    }
    PN_ASSERTM(valid.read() == 1, "Test 29 timed out (signal valid not 1 after 100 cycles)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 29 failed");
    PN_INFOF(("Test 29 passed"));
#endif // PN_CFG_ALU_ENABLE_ZMMUL_EXTENSION == 1

#if PN_CFG_ALU_ENABLE_M_EXTENSION == 1
    // reset alu_mode, so start_in for mul/div is zero
    alu_mode = ALU_MODE_IDLE;
    run_cycle();

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 30 -------"));
    PN_INFOF(("Test 30: div  (-5000, 5)"));
    alu_mode = ALU_MODE_REG_REG;
    funct3 = FUNCT3_XOR_DIV;
    funct7 = FUNCT7_M_EXTENSION;
    a = -5000;
    b = 5;
    y_expected = (uint32_t)(-5000 / 5);
    for(int i = 0; i < 100; i++)
    {
        run_cycle();
        if(valid.read() == 1)
            break;
    }
    PN_ASSERTM(valid.read() == 1, "Test 30 timed out (signal valid not 1 after 100 cycles)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 30 failed");
    PN_INFOF(("Test 30 passed"));

    // reset alu_mode, so start_in for mul/div is zero
    alu_mode = ALU_MODE_IDLE;
    run_cycle();

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 31 -------"));
    PN_INFOF(("Test 31: divu  (5000, 5)"));
    alu_mode = ALU_MODE_REG_REG;
    funct3 = FUNCT3_SRL_SRA_DIVU;
    funct7 = FUNCT7_M_EXTENSION;
    a = 5000;
    b = 5;
    y_expected = 5000 / 5;
    for(int i = 0; i < 100; i++)
    {
        run_cycle();
        if(valid.read() == 1)
            break;
    }
    PN_ASSERTM(valid.read() == 1, "Test 31 timed out (signal valid not 1 after 100 cycles)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 31 failed");
    PN_INFOF(("Test 31 passed"));

    // reset alu_mode, so start_in for mul/div is zero
    alu_mode = ALU_MODE_IDLE;
    run_cycle();

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 32 -------"));
    PN_INFOF(("Test 32: rem  (-4501, 6)"));
    alu_mode = ALU_MODE_REG_REG;
    funct3 = FUNCT3_OR_REM;
    funct7 = FUNCT7_M_EXTENSION;
    a = -4501;
    b = 6;
    y_expected = (uint32_t)(-4501 % 6);
    for(int i = 0; i < 100; i++)
    {
        run_cycle();
        if(valid.read() == 1)
            break;
    }
    PN_ASSERTM(valid.read() == 1, "Test 32 timed out (signal valid not 1 after 100 cycles)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 32 failed");
    PN_INFOF(("Test 32 passed"));

    // reset alu_mode, so start_in for mul/div is zero
    alu_mode = ALU_MODE_IDLE;
    run_cycle();

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 33 -------"));
    PN_INFOF(("Test 33: remu  (-4501, 6)"));
    alu_mode = ALU_MODE_REG_REG;
    funct3 = FUNCT3_AND_REMU;
    funct7 = FUNCT7_M_EXTENSION;
    a = 4501;
    b = 6;
    y_expected = 4501 % 6;
    for(int i = 0; i < 100; i++)
    {
        run_cycle();
        if(valid.read() == 1)
            break;
    }
    PN_ASSERTM(valid.read() == 1, "Test 33 timed out (signal valid not 1 after 100 cycles)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 33 failed");
    PN_INFOF(("Test 33 passed"));

    alu_mode = ALU_MODE_IDLE;
    run_cycle();
#endif // PN_CFG_ALU_ENABLE_M_EXTENSION == 1

#if PN_CFG_ENABLE_ZKNE_ZKND == 1
    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 34 -------"));
    PN_INFOF(("Test 34: aes32dsi"));
    alu_mode = ALU_MODE_REG_REG;
    funct3 = 0;
    funct7 = FUNCT7L_AES32DSI;
    a = 0xDEADBEEF;
    b = 0x12345678;
    y_expected = 0XDEADBE2E;
    for(int i = 0; i < 100; i++)
    {
        run_cycle();
        if(valid.read() == 1)
            break;
    }
    PN_ASSERTM(valid.read() == 1, "Test 34 timed out (signal valid not 1 after 100 cycles)");
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM(y.read() == y_expected.read(), "Test 34 failed");
    PN_INFOF(("Test 34 passed"));
#endif // PN_CFG_ENABLE_ZKNE_ZKND == 1

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}