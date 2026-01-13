/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lorenz Sommer <lorenz.sommer@tha.de>
                     2025 Alexander Beck <alexander.beck1@tha.de>
                     2025 Christian Zellinger <Christian.Zellinger1@tha.de>
                     2025 Niklas Sirch  <niklas.sirch1@tha.de>
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

#include "controller.h"

#include <piconut.h>
#include <piconut-config.h>

#include <systemc.h>

#include <cstdint>

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

/* ---------------  Status signals ---------------  */
sc_signal<bool> PN_NAME(s_alu_less);
sc_signal<bool> PN_NAME(s_alu_lessu);
sc_signal<bool> PN_NAME(s_alu_equal);
sc_signal<bool> PN_NAME(s_dport_ack);
sc_signal<bool> PN_NAME(s_iport_ack);

sc_signal<sc_uint<32>> PN_NAME(s_instruction);

/* Debug */
sc_signal<bool> PN_NAME(s_debug_haltrequest_in);
sc_signal<bool> PN_NAME(s_debug_step_in);

/* --------------- Control signals ---------------  */

/* IPort/DPort */
sc_signal<bool> PN_NAME(c_iport_stb);
sc_signal<bool> PN_NAME(c_dport_stb);
sc_signal<bool> PN_NAME(c_dport_we);

/* Port Registers*/
sc_signal<bool> PN_NAME(c_ld_en_rdata);
sc_signal<bool> PN_NAME(c_ld_en_wdata);
sc_signal<bool> PN_NAME(c_ld_en_adr_bsel);

sc_signal<bool> PN_NAME(c_dport_lrsc);
sc_signal<bool> PN_NAME(c_dport_amo);

sc_signal<bool> PN_NAME(c_rs1_adr_bsel);

sc_signal<bool> PN_NAME(c_alu_rdata_reg); // ALU operand A is rdata register output
sc_signal<bool> PN_NAME(c_alu_out_to_wdata);

/* Regfile input */
sc_signal<bool> PN_NAME(c_reg_ldpc);
sc_signal<bool> PN_NAME(c_reg_ldmem);
sc_signal<bool> PN_NAME(c_reg_ldimm);
sc_signal<bool> PN_NAME(c_reg_ldalu);
sc_signal<bool> PN_NAME(c_reg_ldcsr);

/* Regfile load enable*/
sc_signal<bool> PN_NAME(c_reg_ld_en);

/* ALU */
sc_signal<bool> PN_NAME(c_alu_pc);
sc_signal<bool> PN_NAME(c_alu_imm);
sc_signal<bool> PN_NAME(c_force_add);
sc_signal<bool> PN_NAME(c_force_amo);
sc_signal<sc_uint<3>> PN_NAME(alu_mode);

/* Program counter */
sc_signal<bool> PN_NAME(c_pc_inc4);
sc_signal<bool> PN_NAME(c_pc_ld_en);

/* Instruction register*/
sc_signal<bool> PN_NAME(c_ir_ld);

/* Debug */
sc_signal<bool> PN_NAME(c_debug_haltrequest_ack);
sc_signal<bool> PN_NAME(c_debug_level_enter_ebreak);
sc_signal<bool> PN_NAME(c_debug_level_enter_haltrequest);
sc_signal<bool> PN_NAME(c_debug_level_enter_step);
sc_signal<bool> PN_NAME(c_debug_level_leave);

/* Csr */
sc_signal<sc_uint<CFG_CSR_BUS_ADR_WIDTH>> PN_NAME(c_csr_bus_adr);
sc_signal<bool> PN_NAME(c_csr_bus_write_en);
sc_signal<bool> PN_NAME(c_csr_bus_read_en);
sc_signal<bool> PN_NAME(c_csr_imm_en);
sc_signal<sc_uint<5>> PN_NAME(c_csr_imm);
sc_signal<sc_uint<2>> PN_NAME(c_csr_write_mode);
sc_signal<bool> PN_NAME(c_csr_mret);

sc_signal<bool> PN_NAME(c_csr_interrupt);
sc_signal<bool> PN_NAME(s_interrupt_pending);
sc_signal<bool> trap_handler_enter;
sc_signal<bool> PN_NAME(c_trap_handler_enter);
sc_signal<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(c_csr_bus_wdata);

/* Interrupt specific signals */
sc_signal<bool> PN_NAME(s_mip_msip);    // Software interrupt pending
sc_signal<bool> PN_NAME(s_mie_msie);    // Software interrupt enable
sc_signal<bool> PN_NAME(s_mip_mtip);    // Timer interrupt pending
sc_signal<bool> PN_NAME(s_mie_mtie);    // Timer interrupt enable
sc_signal<bool> PN_NAME(s_mip_meip);    // External interrupt pending
sc_signal<bool> PN_NAME(s_mie_meie);    // External interrupt enable
sc_signal<bool> PN_NAME(s_mstatus_mie); // Global machine interrupt enable

///////////////// Helpers /////////////////
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
    sc_trace_file* tf = PN_BEGIN_TRACE("controller_tb");

    m_controller dut_inst{"dut_inst"};

    dut_inst.clk(clk);
    dut_inst.reset(reset);

    /* ---------------  Status signals ---------------  */
    dut_inst.s_instruction_in(s_instruction);
    dut_inst.s_alu_less_in(s_alu_less);
    dut_inst.s_alu_lessu_in(s_alu_lessu);
    dut_inst.s_alu_equal_in(s_alu_equal);
    dut_inst.s_dport_ack_in(s_dport_ack);
    dut_inst.s_iport_ack_in(s_iport_ack);

    /* Debug */
    dut_inst.s_debug_haltrequest_in(s_debug_haltrequest_in);
    dut_inst.s_debug_step_in(s_debug_step_in);

    /* --------------- Control signals ---------------  */
    dut_inst.c_iport_stb_out(c_iport_stb);
    dut_inst.c_dport_stb_out(c_dport_stb);
    dut_inst.c_dport_we_out(c_dport_we);

    dut_inst.c_dport_lrsc_out(c_dport_lrsc);
    dut_inst.c_dport_amo_out(c_dport_amo);

    dut_inst.c_ld_en_rdata_out(c_ld_en_rdata);
    dut_inst.c_ld_en_wdata_out(c_ld_en_wdata);
    dut_inst.c_ld_en_adr_bsel_out(c_ld_en_adr_bsel);

    dut_inst.c_reg_ldpc_out(c_reg_ldpc);
    dut_inst.c_reg_ldmem_out(c_reg_ldmem);
    dut_inst.c_reg_ldimm_out(c_reg_ldimm);
    dut_inst.c_reg_ldalu_out(c_reg_ldalu);
    dut_inst.c_reg_ldcsr_out(c_reg_ldcsr);

    dut_inst.c_reg_ld_en_out(c_reg_ld_en);

    dut_inst.c_alu_pc_out(c_alu_pc);
    dut_inst.c_alu_imm_out(c_alu_imm);
    dut_inst.c_force_add_out(c_force_add);
    dut_inst.c_force_amo_out(c_force_amo);
    dut_inst.c_alu_rdata_reg_out(c_alu_rdata_reg);       // ALU operand A is regfile output
    dut_inst.c_rs1_adr_bsel_out(c_rs1_adr_bsel);         // ALU operand B is zero
    dut_inst.c_alu_out_to_wdata_out(c_alu_out_to_wdata); // ALU output to wdata

    dut_inst.c_pc_inc4_out(c_pc_inc4);
    dut_inst.c_pc_ld_en_out(c_pc_ld_en);

    dut_inst.c_ir_ld_en_out(c_ir_ld);

    dut_inst.c_alu_mode_out(alu_mode);

    /* Debug */
    dut_inst.c_debug_haltrequest_ack_out(c_debug_haltrequest_ack);
    dut_inst.c_debug_level_enter_ebreak_out(c_debug_level_enter_ebreak);
    dut_inst.c_debug_level_enter_haltrequest_out(c_debug_level_enter_haltrequest);
    dut_inst.c_debug_level_enter_step_out(c_debug_level_enter_step);
    dut_inst.c_debug_level_leave_out(c_debug_level_leave);

    /* Csr */
    dut_inst.c_csr_bus_adr_out(c_csr_bus_adr);
    dut_inst.c_csr_bus_read_en_out(c_csr_bus_read_en);
    dut_inst.c_csr_bus_write_en_out(c_csr_bus_write_en);
    dut_inst.c_csr_imm_en_out(c_csr_imm_en);
    dut_inst.c_csr_imm_out(c_csr_imm);
    dut_inst.c_csr_write_mode_out(c_csr_write_mode);
    dut_inst.c_csr_mret_out(c_csr_mret);

    dut_inst.c_csr_interrupt_out(c_csr_interrupt);
    dut_inst.s_interrupt_pending_in(s_interrupt_pending);
    s_interrupt_pending = 0;

    dut_inst.trap_handler_enter_in(trap_handler_enter);
    trap_handler_enter = 0;

    dut_inst.c_trap_handler_enter_out(c_trap_handler_enter);

    // Add this after the other CSR port bindings
    dut_inst.c_csr_bus_wdata_out(c_csr_bus_wdata);

    /* Interrupt specific signal bindings */
    dut_inst.s_mip_msip_in(s_mip_msip);
    dut_inst.s_mie_msie_in(s_mie_msie);
    dut_inst.s_mip_mtip_in(s_mip_mtip);
    dut_inst.s_mie_mtie_in(s_mie_mtie);
    dut_inst.s_mip_meip_in(s_mip_meip);
    dut_inst.s_mie_meie_in(s_mie_meie);
    dut_inst.s_mstatus_mie_in(s_mstatus_mie);

    // Initialize interrupt signals
    s_mip_msip = 0;
    s_mie_msie = 0;
    s_mip_mtip = 0;
    s_mie_mtie = 0;
    s_mip_meip = 0;
    s_mie_meie = 0;
    s_mstatus_mie = 0;

    /* ---------------------------------------------- */

    dut_inst.pn_trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT

    sc_start(SC_ZERO_TIME); // start simulation

    cout << "\n\t\t*****Simulation started*****" << endl;
    run_cycle();

    /* ------------------- Test 1 ------------------- */
    reset = 1;
    PN_INFOF(("Test #1:Initial state should be reset."));
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_RESET, "Test #1 failed: Initial state should be STATE_RESET");
    PN_INFOF(("Test #1 passed!\n"));

    /* ------------------- Test 2 ------------------- */
    PN_INFOF(("Release reset."));
    reset = 0;
    PN_INFOF(("Test #2: Next state should be STATE_IPORT_STB and iport_stb should be 1."));
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #2 failed: State should be STATE_IPORT_STB");
    PN_ASSERTM((uint32_t)c_iport_stb == 1, "Test #2 failed: c_iport_stb should be 1");
    PN_INFOF(("Test #2 passed!\n"));

    /* ------------------- Test 3 ------------------- */
    PN_INFOF(("Test #3: Next state should be STATE_AWAIT_IPORT_ACK."));
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #3 failed: State should be STATE_AWAIT_IPORT_ACK");
    PN_INFOF(("Test #3 passed!\n"));

    /* ------------------- Test 4 ------------------- */
    PN_INFOF(("Test #4: Without iport_ack state should remain STATE_AWAIT_IPORT_ACK."));
    s_iport_ack = 0x0;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #4 failed: State should be STATE_AWAIT_IPORT_ACK");
    PN_ASSERTM((uint32_t)c_ir_ld == 1, "Test #4 failed: c_ir_ld should be 1");
    PN_INFOF(("Test #4 passed!\n"));

    /* ------------------- Test 5 ------------------- */
    PN_INFOF(("Test #5: With iport_ack state should be DECODE."));
    PN_INFOF(("Setting iport_ack to 1."));
    s_iport_ack = 0x1;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #5 failed: State should be DECODE");
    PN_INFOF(("Test #5 passed!\n"));

    /* ------------------- Test 6 ------------------- */
    PN_INFOF(("Test #6: From DECODE state, opcode OP_ALU should lead to state ALU."));
    s_instruction = 0x00000033; // OP_ALU
    s_iport_ack = 0x0;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_ALU, "Test #6 failed: State should be ALU");
    PN_ASSERTM((uint32_t)c_reg_ldalu == 1, "Test #6 failed: c_reg_ldalu should be 1");
    PN_ASSERTM((uint32_t)c_reg_ld_en == 1, "Test #6 failed: c_reg_ld_en should be 1");
    PN_ASSERTM((uint32_t)c_pc_inc4 == 1, "Test #6 failed: c_pc_inc4 should be 1");
    PN_INFOF(("Test #6 passed!\n"));

    /* ------------------- Test 7 ------------------- */
    PN_INFOF(("Test #7: From STATE_ALU, controller should return to STATE_IPORT_STB."));
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #7 failed: State should be STATE_IPORT_STB");
    PN_ASSERTM((uint32_t)c_iport_stb == 1, "Test #2 failed: c_iport_stb should be 1");
    PN_INFOF(("Test #7 passed!\n"));

    /* ------------------- Test 8 ------------------- */
    PN_INFOF(("Test #8: From STATE_IPORT_STB, controller should go to STATE_AWAIT_IPORT_ACK."));
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #8 failed: State should be STATE_AWAIT_IPORT_ACK");
    PN_ASSERTM((uint32_t)c_ir_ld == 1, "Test #8 failed: c_ir_ld should be 1");
    PN_INFOF(("Test #8 passed!\n"));

    /* ------------------- Test 9 ------------------- */
    PN_INFOF(("Test #9: With iport_ack state should be DECODE."));
    PN_INFOF(("Setting iport_ack to 1."));
    s_iport_ack = 0x1;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #9 failed: State should be DECODE");
    PN_INFOF(("Test #9 passed!\n"));

    /* ------------------- Test 10 ------------------- */
    PN_INFOF(("Test #10: From DECODE state, opcode OP_ALUI should lead to state ALU_IMM."));
    s_instruction = 0x00000013; // OP_ALUI
    s_iport_ack = 0x0;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_ALU_IMM, "Test #10 failed: State should be ALU_IMM");
    PN_ASSERTM((uint32_t)c_reg_ldalu == 1, "Test #10 failed: c_reg_ldalu should be 1");
    PN_ASSERTM((uint32_t)c_reg_ld_en == 1, "Test #10 failed: c_reg_ld_en should be 1");
    PN_ASSERTM((uint32_t)c_pc_inc4 == 1, "Test #10 failed: c_pc_inc4 should be 1");
    PN_ASSERTM((uint32_t)c_alu_imm == 1, "Test #10 failed: c_alu_imm should be 1");
    PN_INFOF(("Test #10 passed!\n"));

    /* ------------------- Test 11 ------------------- */
    PN_INFOF(("Test #11: From STATE_ALU_IMM, controller should return to STATE_IPORT_STB."));
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #11 failed: State should be STATE_IPORT_STB");
    PN_ASSERTM((uint32_t)c_iport_stb == 1, "Test #11 failed: c_iport_stb should be 1");
    PN_INFOF(("Test #11 passed!\n"));

    /* ------------------- Test 12 ------------------- */
    PN_INFOF(("Test #12: Testing load cycle."));
    // STATE_IPORT_STB
    s_instruction = 0x00003003; // OP_LOAD
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #12 failed: State should be STATE_IPORT_STB");
    run_cycle();
    // STATE_AWAIT_IPORT_ACK
    s_iport_ack = 0x1;
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #12 failed: State should be STATE_AWAIT_IPORT_ACK");
    run_cycle();
    // DECOE
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #12 failed: State should be DECODE");
    run_cycle();
    // LOAD1
    s_dport_ack = 0x1;
    run_cycle();
    PN_INFOF(("Stall in LOAD1 until dport_ack is 0"));
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_LOAD1, "Test #12 failed: State should be LOAD1");
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_LOAD1, "Test #12 failed: State should be LOAD1");
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_LOAD1, "Test #12 failed: State should be LOAD1");
    run_cycle();
    // Bus ready -> LOAD3
    s_dport_ack = 0x0;
    PN_INFOF(("Setting dport_ack to 0"));
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_LOAD2, "Test #12 failed: State should be LOAD2");
    PN_ASSERTM((uint32_t)c_dport_stb == 1, "Test #12 failed: c_dport_stb should be 1");
    PN_ASSERTM((uint32_t)c_reg_ld_en == 0, "Test #12 failed: c_reg_ld_en should be 0");
    PN_ASSERTM((uint32_t)c_reg_ldmem == 0, "Test #12 failed: c_reg_ldmem should be 0");
    PN_ASSERTM((uint32_t)c_force_add == 1, "Test #12 failed: c_force_add should be 1");
    PN_ASSERTM((uint32_t)c_pc_inc4 == 1, "Test #12 failed: c_pc_inc4 should be 1");
    PN_ASSERTM((uint32_t)c_alu_imm == 1, "Test #12 failed: c_alu_imm should be 1");
    run_cycle();
    // Go to LOAD3
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_LOAD3, "Test #12 failed: State should be LOAD3");
    PN_ASSERTM((uint32_t)c_dport_stb == 0, "Test #12 failed: c_dport_stb should be 0");
    PN_ASSERTM((uint32_t)c_reg_ld_en == 0, "Test #12 failed: c_reg_ld_en should be 0");
    PN_ASSERTM((uint32_t)c_reg_ldmem == 0, "Test #12 failed: c_reg_ldmem should be 1");
    run_cycle();
    // Stall in LOAD3 until dport_ack is 0
    PN_INFOF(("Stall in LOAD4 until dport_ack is 1"));
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_LOAD3, "Test #12 failed: State should be LOAD3");
    run_cycle();
    // Stall in LOAD3 until dport_ack is 1
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_LOAD3, "Test #12 failed: State should be LOAD3");
    run_cycle();
    // Stall in LOAD3 until dport_ack is 1
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_LOAD3, "Test #12 failed: State should be LOAD3");

    run_cycle();
    // Stall in LOAD3 until dport_ack is 1
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_LOAD3, "Test #12 failed: State should be LOAD3");
    PN_INFOF(("Setting dport_ack to 1"));
    s_dport_ack = 0x1;
    run_cycle();
    // Go to LOAD4
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_LOAD4, "Test #12 failed: State should be STATE_LOAD4");
    PN_ASSERTM((uint32_t)c_dport_stb == 0, "Test #12 failed: c_dport_stb should be 0");
    PN_ASSERTM((uint32_t)c_reg_ld_en == 1, "Test #12 failed: c_reg_ld_en should be 1");
    PN_ASSERTM((uint32_t)c_reg_ldmem == 1, "Test #12 failed: c_reg_ldmem should be 1");
    // got to STATE_IPORT_STB
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #12 failed: State should be STATE_IPORT_STB");
    PN_INFOF(("Test #12 passed!\n"));
    s_iport_ack = 0x1;

    /* ------------------- Test 13 ------------------- */
    run_cycle();
    s_iport_ack = 0x1;
    PN_INFOF(("Test #13: Testing NOP."));
    // should be in await iport ack
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #13 failed: State should be STATE_AWAIT_IPORT_ACK");
    s_instruction = 0x0ff0000f;
    run_cycle();
    // should be in decode
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #13 failed: State should be STATE_DECODE");
    s_iport_ack = 0x1;
    run_cycle();
    // should be in NOP

    PN_ASSERTM(dut_inst.get_state() == STATE_NOP, "Test #13 failed: State should be STATE_NOP");
    PN_INFOF(("Test #13 passed!\n"));

    /* ------------------- Test 14 ------------------- */
    run_cycle();
    PN_INFOF(("Test #14: Testing DEBUG_HALTREQUEST."));
    // should be in iport stb
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #14 failed: State should be STATE_IPORT_STB");
    // execute dummy instruction
    s_iport_ack = 0x1;
    s_instruction = 0x0ff0000f;
    run_cycle(3);
    s_debug_haltrequest_in = 1;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DEBUG_HALTREQUEST1, "Test #14 failed: State should be STATE_DEBUG_HALTREQUEST1");
    PN_ASSERT(c_debug_haltrequest_ack.read() == 1);
    PN_ASSERT(c_debug_level_enter_haltrequest.read() == 1);
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DEBUG_HALTREQUEST2, "Test #14 failed: State should be STATE_DEBUG_HALTREQUEST2");
    PN_ASSERT(c_debug_haltrequest_ack.read() == 0);
    PN_ASSERT(c_debug_level_enter_haltrequest.read() == 0);
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DEBUG_HALTREQUEST2, "Test #14 failed: State should be STATE_DEBUG_HALTREQUEST2");
    s_debug_haltrequest_in = 0;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #14 failed: State should be STATE_IPORT_STB");

    s_iport_ack = 0x0;
    s_instruction = 0x0;

    PN_INFOF(("Test #14 passed!\n"));

    /* ------------------- Test 15 ------------------- */
    PN_INFOF(("Test #15: Testing EBREAK."));

    run_cycle();
    // should be in await iport ack
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #15 failed: State should be STATE_AWAIT_IPORT_ACK");
    s_iport_ack = 0x1;
    s_instruction = 0x00100073; // ebreak

    run_cycle();
    // should be in decode
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #15 failed: State should be STATE_DECODE");

    run_cycle();
    // should be in ebreak
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_EBREAK, "Test #15 failed: State should be STATE_EBREAK");
    PN_ASSERT(c_debug_level_enter_ebreak.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #15 failed: State should be STATE_IPORT_STB");

    s_iport_ack = 0x0;
    s_instruction = 0x0;

    PN_INFOF(("Test #15 passed!\n"));

    /* ------------------- Test 16 ------------------- */
    PN_INFOF(("Test #16: Testing DRET."));

    run_cycle();
    // should be in iport stb wait
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    run_cycle();
    // should be in await iport ack start
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    run_cycle();
    // should be in await iport ack wait
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #16 failed: State should be STATE_AWAIT_IPORT_ACK");
    s_iport_ack = 0x1;
    s_instruction = 0x7b200073; // dret

    run_cycle();
    // should be in decode
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #16 failed: State should be STATE_DECODE");

    run_cycle();
    // should be in dret
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DRET, "Test #16 failed: State should be STATE_DRET");
    PN_ASSERT(c_debug_level_leave.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #16 failed: State should be STATE_IPORT_STB");

    s_iport_ack = 0x0;
    s_instruction = 0x0;

    PN_INFOF(("Test #16 passed!\n"));

    /* ------------------- Test 17 ------------------- */
    PN_INFOF(("Test #17: Testing CSRRW with rd != zero"));

    run_cycle();
    // should be in await iport ack
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #17 failed: State should be STATE_AWAIT_IPORT_ACK");
    s_iport_ack = 0x1;
    s_instruction = 0x7b2414f3; // csrrw s1, dscratch0, s0

    run_cycle();
    // should be in decode
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #17 failed: State should be STATE_DECODE");

    run_cycle();
    // should be in csrrw1
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRW1, "Test #17 failed: State should be STATE_CSRRW1");
    PN_ASSERT(c_csr_bus_read_en.read() == 1);
    PN_ASSERT(c_csr_bus_adr.read() == 0x7b2); // csr adr of dscratch0

    run_cycle();
    // should be in csrrw2
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRW2, "Test #17 failed: State should be STATE_CSRRW2");
    PN_ASSERT(c_reg_ld_en.read() == 1);
    PN_ASSERT(c_reg_ldcsr.read() == 1);

    PN_ASSERT(c_csr_bus_write_en.read() == 1);
    PN_ASSERT(c_csr_bus_adr.read() == 0x7b2); // csr adr of dscratch0
    PN_ASSERT(c_csr_write_mode.read() == e_csr_write_mode::CSR_WRITE_ALL);

    PN_ASSERT(c_pc_inc4.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #17 failed: State should be STATE_IPORT_STB");

    s_iport_ack = 0x0;
    s_instruction = 0x0;

    PN_INFOF(("Test #17 passed!\n"));

    /* ------------------- Test 18 ------------------- */
    PN_INFOF(("Test #18: Testing CSRRW with rd = zero"));

    run_cycle();
    // should be in await iport ack
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #18 failed: State should be STATE_AWAIT_IPORT_ACK");
    s_iport_ack = 0x1;
    s_instruction = 0x7b241073; // csrrw zero, dscratch0, s0

    run_cycle();
    // should be in decode
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #18 failed: State should be STATE_DECODE");

    run_cycle();
    // should be in csrrw1
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRW1, "Test #18 failed: State should be STATE_CSRRW1");
    PN_ASSERT(c_csr_bus_read_en.read() == 0);
    PN_ASSERT(c_csr_bus_adr.read() == 0x0);

    run_cycle();
    // should be in csrrw2
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRW2, "Test #18 failed: State should be STATE_CSRRW2");
    PN_ASSERT(c_reg_ld_en.read() == 0);
    PN_ASSERT(c_reg_ldcsr.read() == 0);

    PN_ASSERT(c_csr_bus_write_en.read() == 1);
    PN_ASSERT(c_csr_bus_adr.read() == 0x7b2); // csr adr of dscratch0
    PN_ASSERT(c_csr_write_mode.read() == e_csr_write_mode::CSR_WRITE_ALL);

    PN_ASSERT(c_pc_inc4.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #18 failed: State should be STATE_IPORT_STB");

    s_iport_ack = 0x0;
    s_instruction = 0x0;

    PN_INFOF(("Test #18 passed!\n"));

    /* ------------------- Test 19 ------------------- */
    PN_INFOF(("Test #19: Testing CSRRC with rs1 != zero"));

    run_cycle();
    // should be in await iport ack
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #19 failed: State should be STATE_AWAIT_IPORT_ACK");
    s_iport_ack = 0x1;
    s_instruction = 0x7b24b473; // csrrc s0, dscratch0, s1

    run_cycle();
    // should be in decode
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #19 failed: State should be STATE_DECODE");

    run_cycle();
    // should be in csrrc1
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRC1, "Test #19 failed: State should be STATE_CSRRC1");
    PN_ASSERT(c_csr_bus_read_en.read() == 1);
    PN_ASSERT(c_csr_bus_adr.read() == 0x7b2); // csr adr of dscratch0

    run_cycle();
    // should be in csrrc2
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRC2, "Test #19 failed: State should be STATE_CSRRC2");
    PN_ASSERT(c_reg_ld_en.read() == 1);
    PN_ASSERT(c_reg_ldcsr.read() == 1);

    PN_ASSERT(c_csr_bus_write_en.read() == 1);
    PN_ASSERT(c_csr_bus_adr.read() == 0x7b2); // csr adr of dscratch0
    PN_ASSERT(c_csr_write_mode.read() == e_csr_write_mode::CSR_WRITE_CLEAR);

    PN_ASSERT(c_pc_inc4.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #19 failed: State should be STATE_IPORT_STB");

    s_iport_ack = 0x0;
    s_instruction = 0x0;

    PN_INFOF(("Test #19 passed!\n"));

    /* ------------------- Test 20 ------------------- */
    PN_INFOF(("Test #20: Testing CSRRC with rs1 = zero"));

    run_cycle();
    // should be in await iport ack
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #20 failed: State should be STATE_AWAIT_IPORT_ACK");
    s_iport_ack = 0x1;
    s_instruction = 0x7b203473; // csrrc s0, dscratch0, zero

    run_cycle();
    // should be in decode
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #20 failed: State should be STATE_DECODE");

    run_cycle();
    // should be in csrrc1
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRC1, "Test #20 failed: State should be STATE_CSRRC1");
    PN_ASSERT(c_csr_bus_read_en.read() == 1);
    PN_ASSERT(c_csr_bus_adr.read() == 0x7b2); // csr adr of dscratch0

    run_cycle();
    // should be in csrrc2
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRC2, "Test #20 failed: State should be STATE_CSRRC2");
    PN_ASSERT(c_reg_ld_en.read() == 1);
    PN_ASSERT(c_reg_ldcsr.read() == 1);

    PN_ASSERT(c_csr_bus_write_en.read() == 0);
    PN_ASSERT(c_csr_bus_adr.read() == 0x0);
    PN_ASSERT(c_csr_write_mode.read() == 0);

    PN_ASSERT(c_pc_inc4.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #20 failed: State should be STATE_IPORT_STB");

    s_iport_ack = 0x0;
    s_instruction = 0x0;

    PN_INFOF(("Test #20 passed!\n"));

    /* ------------------- Test 21 ------------------- */
    PN_INFOF(("Test #21: Testing CSRRS with rs1 != zero"));

    run_cycle();
    // should be in await iport ack
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #21 failed: State should be STATE_AWAIT_IPORT_ACK");
    s_iport_ack = 0x1;
    s_instruction = 0x7b24a473; // csrrs s0, dscratch0, s1

    run_cycle();
    // should be in decode
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #21 failed: State should be STATE_DECODE");

    run_cycle();
    // should be in csrrs1
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRS1, "Test #21 failed: State should be STATE_CSRRS1");
    PN_ASSERT(c_csr_bus_read_en.read() == 1);
    PN_ASSERT(c_csr_bus_adr.read() == 0x7b2); // csr adr of dscratch0

    run_cycle();
    // should be in csrrs2
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRS2, "Test #21 failed: State should be STATE_CSRRS2");
    PN_ASSERT(c_reg_ld_en.read() == 1);
    PN_ASSERT(c_reg_ldcsr.read() == 1);

    PN_ASSERT(c_csr_bus_write_en.read() == 1);
    PN_ASSERT(c_csr_bus_adr.read() == 0x7b2); // csr adr of dscratch0
    PN_ASSERT(c_csr_write_mode.read() == e_csr_write_mode::CSR_WRITE_SET);

    PN_ASSERT(c_pc_inc4.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #21 failed: State should be STATE_IPORT_STB");

    s_iport_ack = 0x0;
    s_instruction = 0x0;

    PN_INFOF(("Test #21 passed!\n"));

    /* ------------------- Test 22 ------------------- */
    PN_INFOF(("Test #22: Testing CSRRS with rs1 = zero"));

    run_cycle();
    // should be in await iport ack
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #22 failed: State should be STATE_AWAIT_IPORT_ACK");
    s_iport_ack = 0x1;
    s_instruction = 0x7b202473; // csrrs s0, dscratch0, zero

    run_cycle();
    // should be in decode
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #22 failed: State should be STATE_DECODE");

    run_cycle();
    // should be in csrrs1
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRS1, "Test #22 failed: State should be STATE_CSRRS1");
    PN_ASSERT(c_csr_bus_read_en.read() == 1);
    PN_ASSERT(c_csr_bus_adr.read() == 0x7b2); // csr adr of dscratch0

    run_cycle();
    // should be in csrrs2
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRS2, "Test #22 failed: State should be STATE_CSRRS2");
    PN_ASSERT(c_reg_ld_en.read() == 1);
    PN_ASSERT(c_reg_ldcsr.read() == 1);

    PN_ASSERT(c_csr_bus_write_en.read() == 0);
    PN_ASSERT(c_csr_bus_adr.read() == 0x0);
    PN_ASSERT(c_csr_write_mode.read() == 0x0);

    PN_ASSERT(c_pc_inc4.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #22 failed: State should be STATE_IPORT_STB");

    s_iport_ack = 0x0;
    s_instruction = 0x0;

    PN_INFOF(("Test #22 passed!\n"));

    /* ------------------- Test 23 ------------------- */
    PN_INFOF(("Test #23: Testing CSRRWI with rd != zero"));

    run_cycle();
    // should be in await iport ack
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #23 failed: State should be STATE_AWAIT_IPORT_ACK");
    s_iport_ack = 0x1;
    s_instruction = 0x7b2ad473; // csrrwi s0, dscratch0, 0x15

    run_cycle();
    // should be in decode
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #23 failed: State should be STATE_DECODE");

    run_cycle();
    // should be in csrrwi1
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRWI1, "Test #23 failed: State should be STATE_CSRRWI1");
    PN_ASSERT(c_csr_bus_read_en.read() == 1);
    PN_ASSERT(c_csr_bus_adr.read() == 0x7b2); // csr adr of dscratch0

    run_cycle();
    // should be in csrrwi2
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRWI2, "Test #23 failed: State should be STATE_CSRRWI2");
    PN_ASSERT(c_reg_ld_en.read() == 1);
    PN_ASSERT(c_reg_ldcsr.read() == 1);

    PN_ASSERT(c_csr_bus_write_en.read() == 1);
    PN_ASSERT(c_csr_bus_adr.read() == 0x7b2); // csr adr of dscratch0
    PN_ASSERT(c_csr_imm_en.read() == 1);
    PN_ASSERT(c_csr_imm.read() == 0x15); // imm value from instruction
    PN_ASSERT(c_csr_write_mode.read() == e_csr_write_mode::CSR_WRITE_ALL);

    PN_ASSERT(c_pc_inc4.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #23 failed: State should be STATE_IPORT_STB");

    s_iport_ack = 0x0;
    s_instruction = 0x0;

    PN_INFOF(("Test #23 passed!\n"));

    /* ------------------- Test 24 ------------------- */
    PN_INFOF(("Test #24: Testing CSRRWI with rd = zero"));

    run_cycle();
    // should be in await iport ack
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #24 failed: State should be STATE_AWAIT_IPORT_ACK");
    s_iport_ack = 0x1;
    s_instruction = 0x7b2ad073; // csrrwi zero, dscratch0, 0x15

    run_cycle();
    // should be in decode
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #24 failed: State should be STATE_DECODE");

    run_cycle();
    // should be in csrrwi1
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRWI1, "Test #24 failed: State should be STATE_CSRRWI1");
    PN_ASSERT(c_csr_bus_read_en.read() == 0);
    PN_ASSERT(c_csr_bus_adr.read() == 0x0);

    run_cycle();
    // should be in csrrwi2
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRWI2, "Test #24 failed: State should be STATE_CSRRWI2");
    PN_ASSERT(c_reg_ld_en.read() == 0);
    PN_ASSERT(c_reg_ldcsr.read() == 0);

    PN_ASSERT(c_csr_bus_write_en.read() == 1);
    PN_ASSERT(c_csr_bus_adr.read() == 0x7b2); // csr adr of dscratch0
    PN_ASSERT(c_csr_imm_en.read() == 1);
    PN_ASSERT(c_csr_imm.read() == 0x15); // imm value from instruction
    PN_ASSERT(c_csr_write_mode.read() == e_csr_write_mode::CSR_WRITE_ALL);

    PN_ASSERT(c_pc_inc4.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #24 failed: State should be STATE_IPORT_STB");

    s_iport_ack = 0x0;
    s_instruction = 0x0;

    PN_INFOF(("Test #24 passed!\n"));

    /* ------------------- Test 25 ------------------- */
    PN_INFOF(("Test #25: Testing CSRRCI with imm != zero"));

    run_cycle();
    // should be in await iport ack
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #25 failed: State should be STATE_AWAIT_IPORT_ACK");
    s_iport_ack = 0x1;
    s_instruction = 0x7b2af473; // csrrci s0, dscratch0, 0x15

    run_cycle();
    // should be in decode
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #25 failed: State should be STATE_DECODE");

    run_cycle();
    // should be in csrrci1
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRCI1, "Test #25 failed: State should be STATE_CSRRCI1");
    PN_ASSERT(c_csr_bus_read_en.read() == 1);
    PN_ASSERT(c_csr_bus_adr.read() == 0x7b2); // csr adr of dscratch0

    run_cycle();
    // should be in csrrci2
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRCI2, "Test #25 failed: State should be STATE_CSRRCI2");
    PN_ASSERT(c_reg_ld_en.read() == 1);
    PN_ASSERT(c_reg_ldcsr.read() == 1);

    PN_ASSERT(c_csr_bus_write_en.read() == 1);
    PN_ASSERT(c_csr_bus_adr.read() == 0x7b2); // csr adr of dscratch0
    PN_ASSERT(c_csr_imm_en.read() == 1);
    PN_ASSERT(c_csr_imm.read() == 0x15); // imm value from instruction
    PN_ASSERT(c_csr_write_mode.read() == e_csr_write_mode::CSR_WRITE_CLEAR);

    PN_ASSERT(c_pc_inc4.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #25 failed: State should be STATE_IPORT_STB");

    s_iport_ack = 0x0;
    s_instruction = 0x0;

    PN_INFOF(("Test #25 passed!\n"));

    /* ------------------- Test 26 ------------------- */
    PN_INFOF(("Test #26: Testing CSRRCI with imm = zero"));

    run_cycle();
    // should be in await iport ack
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #26 failed: State should be STATE_AWAIT_IPORT_ACK");
    s_iport_ack = 0x1;
    s_instruction = 0x7b207473; // csrrci s0, dscratch0, 0x0

    run_cycle();
    // should be in decode
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #26 failed: State should be STATE_DECODE");

    run_cycle();
    // should be in csrrci1
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRCI1, "Test #26 failed: State should be STATE_CSRRCI1");
    PN_ASSERT(c_csr_bus_read_en.read() == 1);
    PN_ASSERT(c_csr_bus_adr.read() == 0x7b2); // csr adr of dscratch0

    run_cycle();
    // should be in csrrci2
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRCI2, "Test #26 failed: State should be STATE_CSRRCI2");
    PN_ASSERT(c_reg_ld_en.read() == 1);
    PN_ASSERT(c_reg_ldcsr.read() == 1);

    PN_ASSERT(c_csr_bus_write_en.read() == 0);
    PN_ASSERT(c_csr_bus_adr.read() == 0x0);
    PN_ASSERT(c_csr_imm_en.read() == 0);
    PN_ASSERT(c_csr_imm.read() == 0x0);
    PN_ASSERT(c_csr_write_mode.read() == 0x0);

    PN_ASSERT(c_pc_inc4.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #26 failed: State should be STATE_IPORT_STB");

    s_iport_ack = 0x0;
    s_instruction = 0x0;

    PN_INFOF(("Test #26 passed!\n"));

    /* ------------------- Test 27 ------------------- */
    PN_INFOF(("Test #27: Testing CSRRSI with imm != zero"));

    run_cycle();
    // should be in await iport ack
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #27 failed: State should be STATE_AWAIT_IPORT_ACK");
    s_iport_ack = 0x1;
    s_instruction = 0x7b2ae473; // csrrsi s0, dscratch0, 0x15

    run_cycle();
    // should be in decode
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #27 failed: State should be STATE_DECODE");

    run_cycle();
    // should be in csrrci1
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRSI1, "Test #27 failed: State should be STATE_CSRRSI1");
    PN_ASSERT(c_csr_bus_read_en.read() == 1);
    PN_ASSERT(c_csr_bus_adr.read() == 0x7b2); // csr adr of dscratch0

    run_cycle();
    // should be in csrrci2
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRSI2, "Test #27 failed: State should be STATE_CSRRSI2");
    PN_ASSERT(c_reg_ld_en.read() == 1);
    PN_ASSERT(c_reg_ldcsr.read() == 1);

    PN_ASSERT(c_csr_bus_write_en.read() == 1);
    PN_ASSERT(c_csr_bus_adr.read() == 0x7b2); // csr adr of dscratch0
    PN_ASSERT(c_csr_imm_en.read() == 1);
    PN_ASSERT(c_csr_imm.read() == 0x15); // imm value from instruction
    PN_ASSERT(c_csr_write_mode.read() == e_csr_write_mode::CSR_WRITE_SET);

    PN_ASSERT(c_pc_inc4.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #27 failed: State should be STATE_IPORT_STB");

    s_iport_ack = 0x0;
    s_instruction = 0x0;

    PN_INFOF(("Test #27 passed!\n"));

    /* ------------------- Test 28 ------------------- */
    PN_INFOF(("Test #28: Testing CSRRSI with imm = zero"));

    run_cycle();
    // should be in await iport ack
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #28 failed: State should be STATE_AWAIT_IPORT_ACK");
    s_iport_ack = 0x1;
    s_instruction = 0x7b206473; // csrrsi s0, dscratch0, zero

    run_cycle();
    // should be in decode
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #28 failed: State should be STATE_DECODE");

    run_cycle();
    // should be in csrrci1
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRSI1, "Test #28 failed: State should be STATE_CSRRSI1");
    PN_ASSERT(c_csr_bus_read_en.read() == 1);
    PN_ASSERT(c_csr_bus_adr.read() == 0x7b2); // csr adr of dscratch0

    run_cycle();
    // should be in csrrci2
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_CSRRSI2, "Test #28 failed: State should be STATE_CSRRSI2");
    PN_ASSERT(c_reg_ld_en.read() == 1);
    PN_ASSERT(c_reg_ldcsr.read() == 1);

    PN_ASSERT(c_csr_bus_write_en.read() == 0);
    PN_ASSERT(c_csr_bus_adr.read() == 0x0);
    PN_ASSERT(c_csr_imm_en.read() == 0);
    PN_ASSERT(c_csr_imm.read() == 0x0);
    PN_ASSERT(c_csr_write_mode.read() == 0x0);

    PN_ASSERT(c_pc_inc4.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #28 failed: State should be STATE_IPORT_STB");

    s_iport_ack = 0x0;
    s_instruction = 0x0;

    PN_INFOF(("Test #28 passed!\n"));

    /* ------------------- Test 29 ------------------- */
    run_cycle();
    PN_INFOF(("Test #29: Testing DEBUG_STEP."));
    // should be in iport stb
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #29 failed: State should be STATE_AWAIT_IPORT_ACK");
    // execute dummy instruction
    s_iport_ack = 0x1;
    s_instruction = 0x0ff0000f;
    run_cycle(2);
    s_debug_step_in = 1;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DEBUG_STEP, "Test #29 failed: State should be STATE_DEBUG_STEP");
    PN_ASSERT(c_debug_level_enter_step.read() == 1);
    s_debug_step_in = 0;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #29 failed: State should be STATE_IPORT_STB");

    s_iport_ack = 0x0;
    s_instruction = 0x0;

    PN_INFOF(("Test #29 passed!\n"));

    PN_INFOF(("Test 30: Interrupt handling - entry and save cause"));

    // First complete the current instruction fetch cycle
    s_iport_ack = 1;
    s_instruction = 0x0ff0000f; // NOP instruction
    run_cycle();                // Should go to STATE_AWAIT_IPORT_ACK
    run_cycle();                // Should go to STATE_DECODE
    run_cycle();                // Should execute NOP and finish instruction

    // Now simulate an interrupt pending
    s_interrupt_pending = 1;
    PN_INFOF(("State before interrupt: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    run_cycle();
    PN_INFOF(("State after interrupt: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_TRAP_ENTRY, "Test 30 failed: should enter TRAP_ENTRY");
    PN_ASSERTM((uint32_t)c_csr_interrupt.read() == 1, "Test 30 failed: c_csr_interrupt should assert");
    // next cycle should move to save mcause
    run_cycle();
    PN_ASSERTM(dut_inst.get_state() == STATE_TRAP_SAVE_MCAUSE, "Test 30 failed: should go to TRAP_SAVE_MCAUSE");
    // cleanup interrupt
    s_interrupt_pending = 0;
    PN_INFOF(("Test #30 passed!\n"));

    /* ------------------- Test 31 ------------------- */
    // Test 31: Validate mstatus MIE clear
    PN_INFOF(("Test 31: Validate mstatus MIE clear"));
    // Already in STATE_TRAP_UPDATE_MSTATUS after Test 30
    run_cycle();
    PN_ASSERTM(dut_inst.get_state() == STATE_TRAP_UPDATE_MSTATUS, "Test 31 failed: Should be STATE_TRAP_UPDATE_MSTATUS");
    PN_ASSERTM(c_csr_bus_adr.read() == 0x300, "Test 31 failed: mstatus address");
    PN_ASSERTM(c_csr_bus_read_en.read() == 1, "Test 31 failed: read_en should assert");
    PN_INFOF(("Test 31 passed!\n"));
    /* ------------------- Test 32 ------------------- */
    // Test 32: Validate trap handler load sequence
    PN_INFOF(("Test 32: Validate trap handler load sequence"));
    run_cycle(); // STATE_TRAP_UPDATE_MSTATUS2
    run_cycle(); // STATE_TRAP_LOAD_HANDLER
    PN_ASSERTM(dut_inst.get_state() == STATE_TRAP_LOAD_HANDLER, "Test 32 failed: Should be STATE_TRAP_LOAD_HANDLER");
    PN_ASSERTM(c_csr_bus_adr.read() == 0x305, "Test 32 failed: mtvec address");
    run_cycle(); // STATE_TRAP_SAVE_PC
    PN_ASSERTM(dut_inst.get_state() == STATE_TRAP_SAVE_PC, "Test 32 failed: Should be STATE_TRAP_SAVE_PC");
    PN_ASSERTM(c_trap_handler_enter.read() == 1, "Test 32 failed: trap_handler_enter should assert");
    PN_INFOF(("Test 32 passed!\n"));

    /* ------------------- Test 33 ------------------- */
    // Test 33: MRET returns to STATE_IPORT_STB
    PN_INFOF(("Test 33: Testing MRET returns to STATE_IPORT_STB"));
    // complete trap return to fetch
    run_cycle();
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test 33 failed: Should be STATE_IPORT_STB after trap return");
    // supply MRET instruction
    s_iport_ack = 1;
    s_instruction = 0x30200073; // SYSTEM_INSTR_MRET
    run_cycle();                // goto AWAIT_IPORT_ACK
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test 33 failed: Should be STATE_AWAIT_IPORT_ACK");
    run_cycle(); // goto DECODE
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test 33 failed: Should be STATE_DECODE");
    run_cycle(); // enter MRET state
    PN_ASSERTM(dut_inst.get_state() == STATE_MRET, "Test 33 failed: Should be STATE_MRET");
    PN_ASSERTM(c_csr_mret.read() == 1, "Test 33 failed: c_csr_mret should assert");
    run_cycle(); // return to IPORT_STB
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test 33 failed: Should return to STATE_IPORT_STB");
    PN_INFOF(("Test 33 passed!\n"));

    /* ------------------- Test 34 ------------------- */
    PN_INFOF(("Test #34: Testing normal AMO Instruction (amoadd.w)."));

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #34 failed: State should be STATE_AWAIT_IPORT_ACK");
    s_iport_ack = 0x1;
    s_instruction = 0x02D892AF; // amoadd.w a0, a1, (a2) (funct5=0x00, rs2=a1, rs1=a2, funct3=W, rd=a0, opcode=OP_A_EXT)

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #34 failed: State should be STATE_DECODE");

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AMO_LOAD1, "Test #34 failed: State should be STATE_AMO_LOAD1");
    s_dport_ack = 0x1;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AMO_LOAD1, "Test #34 failed: Should stall in STATE_AMO_LOAD1");

    s_dport_ack = 0x0;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AMO_LOAD2, "Test #34 failed: State should be STATE_AMO_LOAD2");
    PN_ASSERT(c_dport_stb.read() == 1);
    PN_ASSERT(c_rs1_adr_bsel.read() == 1);
    PN_ASSERT(c_ld_en_adr_bsel.read() == 1);
    PN_ASSERT(c_dport_amo.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AMO_LOAD3, "Test #34 failed: State should be STATE_AMO_LOAD3");
    PN_ASSERT(c_ld_en_rdata.read() == 1);
    s_dport_ack = 0x0;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AMO_LOAD3, "Test #34 failed: Should stall in STATE_AMO_LOAD3");

    s_dport_ack = 0x1;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AMO_OP, "Test #34 failed: State should be STATE_AMO_OP");
    PN_ASSERT(c_alu_rdata_reg.read() == 1);
    PN_ASSERT(c_force_amo.read() == 1);
    PN_ASSERT(c_alu_out_to_wdata.read() == 1);
    PN_ASSERT(c_ld_en_wdata.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AMO_STORE1, "Test #34 failed: State should be STATE_AMO_STORE1");
    s_dport_ack = 0x1;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AMO_STORE1, "Test #34 failed: Should stall in STATE_AMO_STORE1");

    s_dport_ack = 0x0;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AMO_STORE2, "Test #34 failed: State should be STATE_AMO_STORE2");
    PN_ASSERT(c_dport_stb.read() == 1);
    PN_ASSERT(c_dport_we.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AMO_STORE3, "Test #34 failed: State should be STATE_AMO_STORE3");
    PN_ASSERT(c_dport_we.read() == 1);
    s_dport_ack = 0x0;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AMO_STORE3, "Test #34 failed: Should stall in STATE_AMO_STORE3");

    s_dport_ack = 0x1;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_A_STORE_RDATA, "Test #34 failed: State should be STATE_A_STORE_RDATA");
    PN_ASSERT(c_pc_inc4.read() == 1);
    PN_ASSERT(c_reg_ld_en.read() == 1);
    PN_ASSERT(c_reg_ldmem.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #34 failed: State should be STATE_IPORT_STB");

    s_iport_ack = 0x0;
    s_instruction = 0x0;
    s_dport_ack = 0x0;

    PN_INFOF(("Test #34 passed!\n"));

    /* ------------------- Test 35 ------------------- */
    PN_INFOF(("Test #35: Testing LR.W instruction (A-Extension)"));

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #35 failed: State should be STATE_AWAIT_IPORT_ACK");
    s_iport_ack = 0x1;
    s_instruction = 0x100525AF; // lr.w a1, (a0)

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #35 failed: State should be STATE_DECODE");

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_LR1, "Test #35 failed: State should be STATE_LR1");
    s_dport_ack = 0x1;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_LR1, "Test #35 failed: Should stall in STATE_LR1");

    s_dport_ack = 0x0;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_LR2, "Test #35 failed: State should be STATE_LR2");
    PN_ASSERT(c_dport_stb == 1);
    PN_ASSERT(c_dport_lrsc == 1);
    PN_ASSERT(c_rs1_adr_bsel == 1);
    PN_ASSERT(c_ld_en_adr_bsel == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_LR3, "Test #35 failed: State should be STATE_LR3");
    PN_ASSERT(c_ld_en_rdata == 1);
    PN_ASSERT(c_dport_lrsc == 1);
    s_dport_ack = 0x0;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_LR3, "Test #35 failed: Should stall in STATE_LR3");

    s_dport_ack = 0x1;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_A_STORE_RDATA, "Test #35 failed: State should be STATE_A_STORE_RDATA");
    PN_ASSERT(c_pc_inc4.read() == 1);
    PN_ASSERT(c_reg_ld_en.read() == 1);
    PN_ASSERT(c_reg_ldmem.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #35 failed: State should be STATE_IPORT_STB");

    s_iport_ack = 0x0;
    s_instruction = 0x0;
    s_dport_ack = 0x0;

    PN_INFOF(("Test #35 passed!\n"));

    /* ------------------- Test 36 ------------------- */
    PN_INFOF(("Test #36: Testing SC.W instruction (A-Extension)"));

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_AWAIT_IPORT_ACK, "Test #36 failed: State should be STATE_AWAIT_IPORT_ACK");
    s_iport_ack = 0x1;
    s_instruction = 0x1AD892AF; // sc.w a0, a1, (a2) (funct5=0x03, rs2=a1, rs1=a2, funct3=W, rd=a0, opcode=OP_A_EXT)

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_DECODE, "Test #36 failed: State should be STATE_DECODE");

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_SC1, "Test #36 failed: State should be STATE_SC1");
    s_dport_ack = 0x1;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_SC1, "Test #36 failed: Should stall in STATE_SC1");

    s_dport_ack = 0x0;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_SC2, "Test #36 failed: State should be STATE_SC2");
    PN_ASSERT(c_dport_stb.read() == 1);
    PN_ASSERT(c_dport_we.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_SC3, "Test #36 failed: State should be STATE_SC3");
    s_dport_ack = 0x0;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_SC3, "Test #36 failed: Should stall in STATE_SC3");

    s_dport_ack = 0x1;
    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_A_STORE_RDATA, "Test #36 failed: State should be STATE_A_STORE_RDATA");
    PN_ASSERT(c_pc_inc4.read() == 1);
    PN_ASSERT(c_reg_ld_en.read() == 1);
    PN_ASSERT(c_reg_ldmem.read() == 1);

    run_cycle();
    PN_INFOF(("State is: %s", dut_inst.state_to_string(dut_inst.get_state()).c_str()));
    PN_ASSERTM(dut_inst.get_state() == STATE_IPORT_STB, "Test #36 failed: State should be STATE_IPORT_STB");

    s_iport_ack = 0x0;
    s_instruction = 0x0;
    s_dport_ack = 0x0;

    PN_INFOF(("Test #36 passed!\n"));

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}
