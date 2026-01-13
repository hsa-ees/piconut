/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lukas Bauer
                     2025 Alexander Beck <alexander.beck1@tha.de>
                     2025 Christian Zellinger <christian.zellinger1@tha.de>
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
#include "pc.h"
#include <systemc.h>
#include <stdint.h>
#include <piconut.h>

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

sc_signal<sc_uint<32>> PN_NAME(pc_in);
sc_signal<sc_uint<32>> PN_NAME(pc_out);
sc_signal<bool> PN_NAME(inc_in);
sc_signal<bool> PN_NAME(en_load_in);

sc_signal<bool> PN_NAME(debug_level_enter_in);
sc_signal<bool> PN_NAME(debug_level_leave_in);
sc_signal<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(csr_dpc_in);
sc_signal<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(csr_mepc_in);
sc_signal<bool> mret_in;
sc_signal<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(csr_mtvec_in);
sc_signal<bool> PN_NAME(trap_handler_enter_in);

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
    sc_trace_file* tf = PN_BEGIN_TRACE("pc_tb");

    m_pc dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.clk(clk);
    dut_inst.reset(reset);

    dut_inst.pc_in(pc_in);
    dut_inst.pc_out(pc_out);
    dut_inst.inc_in(inc_in);
    dut_inst.en_load_in(en_load_in);
    dut_inst.debug_level_enter_in(debug_level_enter_in);
    dut_inst.debug_level_leave_in(debug_level_leave_in);
    dut_inst.csr_dpc_in(csr_dpc_in);
    dut_inst.csr_mepc_in(csr_mepc_in);
    dut_inst.mret_in(mret_in);
    dut_inst.csr_mtvec_in(csr_mtvec_in);
    dut_inst.trap_handler_enter_in(trap_handler_enter_in);

    dut_inst.pn_trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT

    sc_start(SC_ZERO_TIME); // start simulation

    cout << "\n\t\t*****Simulation started*****" << endl;

    run_cycle();
    PN_INFOF(("Set reset high"));
    reset = 1;
    run_cycle();
    PN_INFOF(("Set reset low"));
    reset = 0;
    run_cycle();

    /* ------------------- Test 1 ------------------- */
    PN_INFOF(("Test 1: Check if start adress is correct"));
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == 0x10000000, "Test 1 failed");
    PN_INFOF(("Test 1: Passed"));
    PN_INFOF(("\n"));

    /* ------------------- Test 2 ------------------- */
    PN_INFOF(("Test 2: Attempt loading of new value, check if value is loaded"));
    pc_in = 0x0000ffe0;
    PN_INFOF(("pc_in: 0x%x", (uint32_t)pc_in.read()));
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));

    PN_INFOF(("Test 1: Check if start adress is correct"));
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == 0x10000000, "Test 1 failed");
    PN_INFOF(("Test 1: Passed"));
    PN_INFOF(("\n"));

    PN_INFOF(("Test 2: Attempt loading of new value, check if value is loaded"));
    pc_in = 0x0000ffe0;
    PN_INFOF(("pc_in: 0x%x", (uint32_t)pc_in.read()));
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    en_load_in = 1;
    PN_INFOF(("Clock cycle"));
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM((uint32_t)pc_out.read() == (0x0000ffe0), "Test 2 failed");
    PN_INFOF(("Test 2: Passed"));
    PN_INFOF(("\n"));

    /* ------------------- Test 3 ------------------- */
    PN_INFOF(("Test 3: Disable load, enable increment, check if PC is incremented by 4"));
    en_load_in = 0;
    pc_in = 0x0;
    inc_in = 1;
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_INFOF(("Clock cycle"));
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == ((0x0000ffe0) + 0x4), "Test 3 failed");
    PN_INFOF(("Test 3: Passed"));
    PN_INFOF(("\n"));

    /* ------------------- Test 4 ------------------- */
    PN_INFOF(("Test 4: Disable load, disable increment, check if PC is held"));
    inc_in = 0;
    pc_in = 0x0;
    en_load_in = 0;
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_INFOF(("Clock cycle"));
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == ((0x0000ffe0) + 0x4), "Test 4 failed");
    PN_INFOF(("Test 4: Passed"));
    PN_INFOF(("\n"));

    /* ------------------- Test 5 ------------------- */
    PN_INFOF(("Test 5: Set pc_in to some value, don't set load enable, check if value remains"));
    pc_in = 0x11111111;
    run_cycle();
    PN_INFOF(("pc_in: 0x%x", (uint32_t)pc_in.read()));
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_INFOF(("Clock cycle"));
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == ((0x0000ffe0) + 0x4), "Test 5 failed");
    PN_INFOF(("Test 5: Passed"));
    PN_INFOF(("\n"));

    /* ------------------- Test 6 ------------------- */
    PN_INFOF(("Test 6: Perform two more increments. Check if PC is incremented correctly"));
    inc_in = 1;
    run_cycle(2);
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == ((0x0000ffe0) + 0xC), "Test 6 failed");
    PN_INFOF(("Test 6: Passed"));
    inc_in = 0;
    PN_INFOF(("\n"));

    /* ------------------- Test 7 ------------------- */
    PN_INFOF(("Test 7: Trigger reset, check if PC is reset correctly"));
    reset = 0;
    PN_INFOF(("Set reset high and advance one clock cycle"));
    run_cycle();
    PN_INFOF(("Set reset low and advance one clock cycle"));
    reset = 1;
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == 0x10000000, "Test 7 failed");
    PN_INFOF(("Test 7: Passed"));
    reset = 0;
    PN_INFOF(("\n"));

    /* ------------------- Test 8 ------------------- */
    PN_INFOF(("Test 8: Perform debug level enter. Check if PC is set correctly"));
    debug_level_enter_in = 1;
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == CFG_DEBUG_DEBUG_HANDLER_START_ADDRESS, "Test 8 failed");
    PN_INFOF(("Test 8: Passed"));
    debug_level_enter_in = 0;
    PN_INFOF(("\n"));

    /* ------------------- Test 9 ------------------- */
    PN_INFOF(("Test 9: Perform debug level leave. Check if PC is set to csr_dpc_in"));
    debug_level_leave_in = 1;
    csr_dpc_in = 0xaffeaff0;
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == 0xaffeaff0, "Test 9 failed");
    PN_INFOF(("Test 9: Passed"));
    debug_level_leave_in = 0;
    csr_dpc_in = 0x0;
    PN_INFOF(("\n"));

    /* ------------------- Test 10 ------------------- */
    PN_INFOF(("Test 10: Trap handling - Test entering trap handler"));
    // Set up a custom trap vector address
    csr_mtvec_in = 0x80001000;
    trap_handler_enter_in = 1;
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == 0x80001000, "Test 10 failed");
    PN_INFOF(("Test 10: Passed"));
    trap_handler_enter_in = 0;
    PN_INFOF(("\n"));

    /* ------------------- Test 11 ------------------- */
    PN_INFOF(("Test 11: Trap handling - Test returning from trap (MRET instruction)"));
    // Set up a return address in mepc
    csr_mepc_in = 0x20004000;
    mret_in = 1;
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    // Should match the MEPC value directly
    PN_ASSERTM(pc_out.read() == 0x20004000, "Test 11 failed - PC should be MEPC value");
    PN_INFOF(("Test 11: Passed"));
    mret_in = 0;
    PN_INFOF(("\n"));

    /* ------------------- Test 12 ------------------- */
    PN_INFOF(("Test 12: Test trap handling priority over other control signals"));
    // Set up all control signals at once - trap handler should have priority
    trap_handler_enter_in = 1;
    csr_mtvec_in = 0x80002000;
    en_load_in = 1;
    pc_in = 0x12345678;
    inc_in = 1;
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == 0x80002000, "Test 12 failed");
    PN_INFOF(("Test 12: Passed"));
    trap_handler_enter_in = 0;
    en_load_in = 0;
    inc_in = 0;
    PN_INFOF(("\n"));

    /* ------------------- Test 13 ------------------- */
    PN_INFOF(("Test 13: Test MRET priority over regular load and increment"));
    // Set up competing control signals - MRET should have priority
    mret_in = 1;
    csr_mepc_in = 0x30005000;
    en_load_in = 1;
    pc_in = 0x12345678;
    inc_in = 1;
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    // Should directly load MEPC value
    PN_ASSERTM(pc_out.read() == 0x30005000, "Test 13 failed - PC should be MEPC value");
    PN_INFOF(("Test 13: Passed"));
    mret_in = 0;
    en_load_in = 0;
    inc_in = 0;
    PN_INFOF(("\n"));

    /* ------------------- Test 14 ------------------- */
    PN_INFOF(("Test 14: Test priority between debug, trap and MRET - debug_level_enter has priority"));
    // Set all three special control signals - debug_level_enter should win
    debug_level_enter_in = 1;
    trap_handler_enter_in = 1;
    csr_mtvec_in = 0x80003000;
    mret_in = 1;
    csr_mepc_in = 0x40006000;
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == CFG_DEBUG_DEBUG_HANDLER_START_ADDRESS, 
               "Test 14 failed - debug_level_enter should have priority");
    PN_INFOF(("Test 14: Passed"));
    debug_level_enter_in = 0;
    trap_handler_enter_in = 0;
    mret_in = 0;
    PN_INFOF(("\n"));

    /* ------------------- Test 15 ------------------- */
    PN_INFOF(("Test 15: Test priority between debug_level_leave, trap and MRET - debug_level_leave has priority"));
    // Set all three special control signals - debug_level_leave should win
    debug_level_leave_in = 1;
    csr_dpc_in = 0xbbbbbbbc;
    trap_handler_enter_in = 1;
    csr_mtvec_in = 0x80003000;
    mret_in = 1;
    csr_mepc_in = 0x40006000;
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == 0xbbbbbbbc, 
               "Test 15 failed - debug_level_leave should have priority");
    PN_INFOF(("Test 15: Passed"));
    debug_level_leave_in = 0;
    trap_handler_enter_in = 0;
    mret_in = 0;
    PN_INFOF(("\n"));

    /* ------------------- Test 16 ------------------- */
    PN_INFOF(("Test 16: Test trap handler with unaligned MTVEC address"));
    // Set an unaligned trap vector address (lowest 2 bits non-zero)
    csr_mtvec_in = 0x80004003;  // Last 2 bits are 11
    trap_handler_enter_in = 1;
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    // PC should be aligned (lowest 2 bits forced to 00)
    PN_ASSERTM(pc_out.read() == 0x80004000, 
               "Test 16 failed - PC should be aligned (lowest 2 bits are 00)");
    PN_INFOF(("Test 16: Passed"));
    trap_handler_enter_in = 0;
    PN_INFOF(("\n"));

    /* ------------------- Test 17 ------------------- */
    PN_INFOF(("Test 17: Test MRET with unaligned MEPC address"));
    // Set an unaligned return address (lowest 2 bits non-zero)
    csr_mepc_in = 0x50007003;  // Last 2 bits are 11
    mret_in = 1;
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    // PC should be aligned to word boundary only
    PN_ASSERTM(pc_out.read() == 0x50007000, 
           "Test 17 failed - PC should be aligned to word boundary");
    PN_INFOF(("Test 17: Passed"));
    mret_in = 0;
    PN_INFOF(("\n"));

    /* ------------------- Test 18 ------------------- */
    PN_INFOF(("Test 18: Test MRET with boundary condition (0xFFFFFFFC)"));
    // Set MEPC to almost the maximum 32-bit value
    csr_mepc_in = 0xFFFFFFFC;
    mret_in = 1;
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    // Should be exactly 0xFFFFFFFC (no +4 addition)
    PN_ASSERTM(pc_out.read() == 0xFFFFFFFC, 
           "Test 18 failed - PC should be 0xFFFFFFFC");
    PN_INFOF(("Test 18: Passed"));
    mret_in = 0;
    PN_INFOF(("\n"));

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}