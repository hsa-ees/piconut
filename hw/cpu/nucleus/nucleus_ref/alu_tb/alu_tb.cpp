/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lorenz Sommer <lorenz.sommer@tha.de>
                     2025 Niklas Sirch  <niklas.sirch1@tha.de
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

#include "alu.h"
#include <systemc.h>
#include <stdint.h>
#include <piconut.h>

#define PERIOD_NS 10.0

sc_signal<sc_uint<32>> PN_NAME(a);     // Operand A
sc_signal<sc_uint<32>> PN_NAME(b);     // Operand B
sc_signal<sc_uint<3>> PN_NAME(funct3); // selects operation
sc_signal<sc_uint<32>> PN_NAME(y);     // Result Y
sc_signal<bool> PN_NAME(force_add);
sc_signal<bool> PN_NAME(equal);        // Equal flag
sc_signal<bool> PN_NAME(less);         // Less flag
sc_signal<bool> PN_NAME(lessu);        // Less unsigned flag
sc_signal<sc_uint<7>> PN_NAME(funct7); // funct7 flag - This flag is derived from IR[31:25]
sc_signal<sc_uint<3>> PN_NAME(alu_mode);
sc_signal<bool> PN_NAME(force_amo);

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
    sc_trace_file* tf = PN_BEGIN_TRACE("alu_tb");

    m_alu dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB

    dut_inst.a_in(a);
    dut_inst.b_in(b);
    dut_inst.funct7_in(funct7);
    dut_inst.funct3_in(funct3);
    dut_inst.y_out(y);
    dut_inst.force_add_in(force_add);
    dut_inst.equal_out(equal);
    dut_inst.less_out(less);
    dut_inst.lessu_out(lessu);
    dut_inst.alu_mode_in(alu_mode);
    dut_inst.force_amo_in(force_amo); // Connect force_amo signal

    dut_inst.pn_trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT

    sc_start(SC_ZERO_TIME); // start simulation

    cout << "\n\t\t*****Simulation started*****" << endl;
    run_cycle();

    PN_INFOF(("------- Test 1 -------"));
    PN_INFOF(("Test 1: 0x5 + 0x5, expecting 0xA"));
    funct7 = 0x0;
    a = 0x5;
    b = 0x5;
    alu_mode = 0x0;
    funct7 = FUNCT7_RV32I_BASE;
    funct3 = FUNCT3_ADD_SUB;
    run_cycle(1);
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM((uint32_t)y.read() == 0xA, "Test 1 failed");
    PN_INFOF(("Test 1 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 2 -------"));
    PN_INFOF(("Test 2: Subtract 0xB - 0x5, expecting 0x6"));
    funct7 = 0x20;
    a = 0xB;
    b = 0x5;
    run_cycle(1);
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM((uint32_t)y.read() == 0x6, "Test 2 failed");
    PN_INFOF(("Test 2 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 3 -------"));
    PN_INFOF(("Test 3: 0x2 - 0x3, expecting underflow 0xFFFFFFFF"));
    funct7 = 0x20;
    a = 0x2;
    b = 0x3;
    funct3 = FUNCT3_ADD_SUB;
    run_cycle();
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM((int32_t)y.read() == ((unsigned)a.read() - (unsigned)b.read()), "Test 3 failed.");
    PN_INFOF(("Test 3 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 4 -------"));
    PN_INFOF(("Test 4: '10101' & '00101', expecting '00001'"));
    funct3 = FUNCT3_AND;
    funct7 = 0x0;
    a = 0b10101;
    b = 0b00101;
    run_cycle();
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM((int32_t)y.read() == (a.read() & b.read()), "Test 4 failed.");
    PN_INFOF(("Test 4 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 5 -------"));
    PN_INFOF(("Test 5: '110' | '001', expecting '111'"));
    funct3 = FUNCT3_OR;
    a = 0b110;
    b = 0b001;
    run_cycle();
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM((int32_t)y.read() == (a.read() | b.read()), "Test 5 failed.");
    PN_INFOF(("Test 5 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 6 -------"));
    PN_INFOF(("Test 6: '10101' ^ '10100', expecting '00001'"));
    funct3 = FUNCT3_XOR;
    a = 0b10101;
    b = 0b10100;
    run_cycle();
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM((int32_t)y.read() == (a.read() ^ b.read()), "Test 6 failed.");
    PN_INFOF(("Test 6 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 7 -------"));
    PN_INFOF(("Test 7: (-5) < 10 ? 1:0, expecting 1, expecting less_out = 1"));
    a = -5;
    b = 10;
    funct3 = FUNCT3_SLT;
    run_cycle();
    PN_INFOF(("Operand A: %d", (int32_t)a.read()));
    PN_INFOF(("Operand B: %d", (int32_t)b.read()));
    PN_INFOF(("Result Y: %d", (int32_t)y.read()));
    PN_ASSERTM((uint32_t)y.read() == (((int32_t)a.read() < (int32_t)b.read()) ? 1 : 0), "Test 7 failed.");
    PN_ASSERTM((uint32_t)less.read() == 1, "Test 7 failed.");
    PN_INFOF(("Test 7 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 8 -------"));
    PN_INFOF(("Test 8: 600 < (-10) ? 1:0, expecting 0, expecting less_out = 0"));
    a = 600;
    b = -10;
    funct3 = FUNCT3_SLT;
    run_cycle();
    PN_INFOF(("Operand A: %d", (int32_t)a.read()));
    PN_INFOF(("Operand B: %d", (int32_t)b.read()));
    PN_INFOF(("Result Y: %d", (int32_t)y.read()));
    PN_INFOF(("Less flag: %d", (uint32_t)less.read()));
    PN_ASSERTM((uint32_t)y.read() == (((int32_t)a.read() < (int32_t)b.read()) ? 1 : 0), "Test 8 failed.");
    PN_ASSERTM((uint32_t)less.read() == 0, "Test 8 failed.");
    PN_INFOF(("Test 8 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 9 -------"));
    PN_INFOF(("Test 9: (-100) < (-10) ? 1:0, expecting 1, expecting less_out = 1"));
    a = -100;
    b = -10;
    funct3 = FUNCT3_SLT;
    run_cycle();
    PN_INFOF(("Operand A: %d", (int32_t)a.read()));
    PN_INFOF(("Operand B: %d", (int32_t)b.read()));
    PN_INFOF(("Result Y: %d", (int32_t)y.read()));
    PN_INFOF(("Less flag: %d", (uint32_t)less.read()));
    PN_ASSERTM((uint32_t)y.read() == (((int32_t)a.read() < (int32_t)b.read()) ? 1 : 0), "Test 9 failed.");
    PN_ASSERTM((uint32_t)less.read() == 1, "Test 9 failed.");
    PN_INFOF(("Test 9 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 10 -------"));
    PN_INFOF(("Test 10: (unsigned)(-5) < 10 ? 1:0 , expecting 0, expecting lessu_out = 0"));
    a = -5;
    b = 10;
    funct3 = FUNCT3_SLTU;
    run_cycle();
    PN_INFOF(("Operand A: 0x%x", (int32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (int32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (int32_t)y.read()));
    PN_INFOF(("Lessu flag: %d", (uint32_t)lessu.read()));
    PN_ASSERTM((uint32_t)y.read() == (((uint32_t)a.read() < (uint32_t)b.read()) ? 1 : 0), "Test 10 failed.");
    PN_ASSERTM((uint32_t)lessu.read() == 0, "Test 10 failed.");
    PN_INFOF(("Test 10 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 11 -------"));
    PN_INFOF(("Test 11: 600 < (unsigned)(-10) ? 1:0, expecting 1, expecting lessu_out = 1"));
    a = 600;
    b = -10;
    funct3 = FUNCT3_SLTU;
    run_cycle();
    PN_INFOF(("Operand A: 0x%x", (int32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (int32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (int32_t)y.read()));
    PN_INFOF(("Lessu flag: %d", (uint32_t)lessu.read()));
    PN_ASSERTM((uint32_t)y.read() == (((uint32_t)a.read() < (uint32_t)b.read()) ? 1 : 0), "Test 11 failed.");
    PN_ASSERTM((uint32_t)lessu.read() == 1, "Test 11 failed.");
    PN_INFOF(("Test 11 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 12 -------"));
    PN_INFOF(("Test 12: 10 < 10 ? 1:0, expecting 0, expecting equal_out = 1"));
    a = 10;
    b = 10;
    funct3 = FUNCT3_SLT;
    run_cycle();
    PN_INFOF(("Operand A: %d", (int32_t)a.read()));
    PN_INFOF(("Operand B: %d", (int32_t)b.read()));
    PN_INFOF(("Result Y: %d", (int32_t)y.read()));
    PN_INFOF(("Equal flag: %d", (uint32_t)equal.read()));
    PN_ASSERTM((uint32_t)y.read() == (((int32_t)a.read() < (int32_t)b.read()) ? 1 : 0), "Test 12 failed.");
    PN_ASSERTM((uint32_t)equal.read() == 1, "Test 12 failed.");
    PN_INFOF(("Test 12 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 13 -------"));
    PN_INFOF(("Test 12: '0001000' << 1, expecting '0010000'"));
    a = 0x0001000;
    b = 0x1;
    funct3 = FUNCT3_SLL;
    run_cycle();
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM((uint32_t)y.read() == (a.read() << b.read()), "Test 13 failed.");
    PN_INFOF(("Test 13 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 14 -------"));
    PN_INFOF(("Test 14: '1000' >> 3, expecting '0001', logical shift zero extends"));
    a = 8;
    b = 3;
    funct3 = FUNCT3_SRL_SRA;
    funct7 = 0x20;
    run_cycle();
    PN_INFOF(("Operand A: %d", (uint32_t)a.read()));
    PN_INFOF(("Operand B: %d", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM((uint32_t)y.read() == (a.read() >> b.read()), "Test 14 failed.");
    PN_INFOF(("Test 14 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 15 -------"));
    PN_INFOF(("Test 15: '1' >> 1, expecting '0x0', logical shift zero extends"));
    a = 0x1;
    b = 0x1;
    funct7 = 0;
    funct3 = FUNCT3_SRL_SRA;
    run_cycle();
    PN_INFOF(("Operand A: %d", (uint32_t)a.read()));
    PN_INFOF(("Operand B: %d", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM((uint32_t)y.read() == (a.read() >> b.read()), "Test 15 failed.");
    PN_INFOF(("Test 15 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 16 -------"));
    PN_INFOF(("Test 16: '0xFAFAFAFA' >> 2, expecting '0xFEBEBEBE', arithmetic shift sign extends"));
    a = 0xFAFAFAFA;
    b = 0x2;
    funct3 = FUNCT3_SRL_SRA;
    funct7 = 0x20;
    run_cycle();
    PN_INFOF(("Operand A: %x", (int32_t)a.read()));
    PN_INFOF(("Operand B: %d", (int32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (int32_t)y.read()));
    PN_ASSERTM((int32_t)y.read() == 0xFEBEBEBE, "Test 16 failed.");
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
    run_cycle();
    PN_INFOF(("Operand A: %d", (int32_t)a.read()));
    PN_INFOF(("Operand B: %d", (int32_t)b.read()));
    PN_INFOF(("Result Y: %d", (int32_t)y.read()));
    PN_ASSERTM((int32_t)y.read() == -5, "Test 17a failed");

    PN_INFOF(("Test 17: force_amo MIN (signed) (10, -5), expecting -5"));
    a = 10;
    b = -5;
    run_cycle();
    PN_INFOF(("Operand A: %d", (int32_t)a.read()));
    PN_INFOF(("Operand B: %d", (int32_t)b.read()));
    PN_INFOF(("Result Y: %d", (int32_t)y.read()));
    PN_ASSERTM((int32_t)y.read() == -5, "Test 17b failed");
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
    run_cycle();
    PN_INFOF(("Operand A: %d", (int32_t)a.read()));
    PN_INFOF(("Operand B: %d", (int32_t)b.read()));
    PN_INFOF(("Result Y: %d", (int32_t)y.read()));
    PN_ASSERTM((int32_t)y.read() == 10, "Test 18a failed");

    PN_INFOF(("Test 18: force_amo MAX (signed) (10, -5), expecting 10"));
    a = 10;
    b = -5;
    run_cycle();
    PN_INFOF(("Operand A: %d", (int32_t)a.read()));
    PN_INFOF(("Operand B: %d", (int32_t)b.read()));
    PN_INFOF(("Result Y: %d", (int32_t)y.read()));
    PN_ASSERTM((int32_t)y.read() == 10, "Test 18b failed");
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
    run_cycle();
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM((uint32_t)y.read() == 5, "Test 19a failed");

    PN_INFOF(("Test 19: force_amo MINU (unsigned) (0xFFFFFFFB, 10), expecting 10"));
    a = 0xFFFFFFFB; // Unsigned large value
    b = 10;
    run_cycle();
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM((uint32_t)y.read() == 10, "Test 19b failed");
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
    run_cycle();
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM((uint32_t)y.read() == 10, "Test 20a failed");

    PN_INFOF(("Test 20: force_amo MAXU (unsigned) (0xFFFFFFFB, 10), expecting 0xFFFFFFFB"));
    a = 0xFFFFFFFB; // Unsigned large value
    b = 10;
    run_cycle();
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM((uint32_t)y.read() == 0xFFFFFFFB, "Test 20b failed");
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
    run_cycle();
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM((uint32_t)y.read() == 15, "Test 21 failed");
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
    run_cycle();
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM((uint32_t)y.read() == (a.read() & b.read()), "Test 22 failed");
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
    run_cycle();
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM((uint32_t)y.read() == (a.read() | b.read()), "Test 23 failed");
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
    run_cycle();
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM((uint32_t)y.read() == (a.read() ^ b.read()), "Test 24 failed");
    PN_INFOF(("Test 24 passed"));

    PN_INFOF(("\n"));
    PN_INFOF(("------- Test 25 -------"));
    PN_INFOF(("Test 24: force_amo  (0b1111, 0b0101), expecting 0b0101"));
    force_amo = 1;
    force_add = 0;
    alu_mode = 0x0;
    funct3 = 0x0;
    funct7 = FUNCT5A_SWAP << 2;
    a = 0b1111;
    b = 0b0101;
    run_cycle();
    PN_INFOF(("Operand A: 0x%x", (uint32_t)a.read()));
    PN_INFOF(("Operand B: 0x%x", (uint32_t)b.read()));
    PN_INFOF(("Result Y: 0x%x", (uint32_t)y.read()));
    PN_ASSERTM((uint32_t)y.read() == (b.read()), "Test 25 failed");
    PN_INFOF(("Test 25 passed"));

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}