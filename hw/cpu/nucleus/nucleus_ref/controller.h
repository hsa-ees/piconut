/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lorenz Sommer <lorenz.sommer@tha.de>
                     2024 Johannes Hofmann <johannes.hofmann1@tha.de>
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

/**
 * @fn SC_MODULE(m_controller)
 * @brief Main state machine of the processor. Reads status signals and controls module behavior via control signals.
 *
 * The controller handles interrupt processing according to RISC-V specifications:
 * - Monitors interrupt pending signals from the CSR module
 * - Implements trap entry sequence (STATE_TRAP_ENTRY, STATE_TRAP_SAVE_PC, etc.)
 * - Handles MRET instruction for returning from machine-mode exceptions/interrupts
 * - Coordinates with CSR module for saving/restoring processor state during traps
 *
 * @par Notes
 * - The syntax "s_<signal_name>" and "c_<signal_name>" is used to denote status and control signals, respectively.
 *   Status signals are outputs of various modules, while control signals are outputs of the controller.
 * - "Current instruction" refers to the value held by the instruction register at the current cycle.
 *
 * @param[in]  clk                             Clock signal
 * @param[in]  reset                           Reset signal
 * @param[in]  s_instruction_in<32>            The current instruction
 * @param[in]  s_alu_less_in<1>                High if ALU operand A is less than operand B
 * @param[in]  s_alu_lessu_in<1>               High if ALU operand A is less than operand B (unsigned)
 * @param[in]  s_alu_equal_in<1>               High if ALU operand A is equal to operand B
 * @param[in]  s_dport_ack_in<1>               Data port acknowledge signal
 * @param[in]  s_iport_ack_in<1>               Instruction port acknowledge signal
 * @param[in]  s_debug_haltrequest_in<1>       Halt request for debugging
 * @param[in]  s_debug_step_in<1>              Debug stepping active
 * @param[in]  s_interrupt_pending_in<1>        Combined interrupt pending signal from CSR module
 * @param[in]  s_mip_msip_in<1>                 Machine Software Interrupt Pending from CSR
 * @param[in]  s_mie_msie_in<1>                 Machine Software Interrupt Enable from CSR
 * @param[in]  s_mip_mtip_in<1>                 Machine Timer Interrupt Pending from CSR
 * @param[in]  s_mie_mtie_in<1>                 Machine Timer Interrupt Enable from CSR
 * @param[in]  s_mip_meip_in<1>                 Machine External Interrupt Pending from CSR
 * @param[in]  s_mie_meie_in<1>                 Machine External Interrupt Enable from CSR
 * @param[in]  s_mstatus_mie_in<1>              Machine Status Global Interrupt Enable from CSR
 *
 * @param[out] c_iport_stb_out<1>               Instruction port strobe signal
 * @param[out] c_dport_stb_out<1>               Data port strobe signal
 * @param[out] c_dport_we_out<1>                Data port write enable signal
 * @param[out] c_dport_lrsc_out<1>              Data port load-reserved/store-conditional signal
 * @param[out] c_dport_amo_out<1>               Data port atomic memory operation signal
 * @param[out] c_ld_en_rdata_out<1>             Enable register for loading rdata
 * @param[out] c_ld_en_wdata_out<1>             Enable register for loading wdata
 * @param[out] c_ld_en_adr_bsel_out<1>          Enable register for loading address bsel
 * @param[out] c_reg_ldpc_out<1>                Enables register file to load value from program counter
 * @param[out] c_reg_ldmem_out<1>               Enables register file to load value from memory
 * @param[out] c_reg_ldimm_out<1>               Enables register file to load immediate value
 * @param[out] c_reg_ldalu_out<1>               Enables register file to load value from ALU
 * @param[out] c_reg_ldcsr_out<1>               Enables register file to load value from CSR-bus read signal
 * @param[out] c_reg_ld_en_out<1>               Enables register file to load a value
 * @param[out] c_alu_pc_out<1>                  Direct program counter to be ALU operand A
 * @param[out] c_alu_imm_out<1>                 Direct immediate value to be ALU operand B
 * @param[out] c_alu_rdata_reg_out<1>           ALU rdata register output enable
 * @param[out] c_rs1_adr_bsel_out<1>            ALU operand B is zero/address select
 * @param[out] c_alu_out_to_wdata_out<1>        ALU output to wdata
 * @param[out] c_force_add_out<1>               Force ALU to perform addition
 * @param[out] c_force_amo_out<1>               Force atomic memory operation
 * @param[out] c_alu_mode_out<3>                ALU mode output
 * @param[out] c_pc_inc4_out<1>                 Increments program counter by 4 on next rising edge
 * @param[out] c_pc_ld_en_out<1>                Enables program counter to load a new value
 * @param[out] c_ir_ld_en_out<1>                Enables instruction register to load new value from memory
 * @param[out] c_debug_haltrequest_ack_out<1>   Acknowledge halt request for debugging
 * @param[out] c_debug_level_enter_ebreak_out<1>      Debug level enter request caused by ebreak
 * @param[out] c_debug_level_enter_haltrequest_out<1> Debug level enter request caused by halt request
 * @param[out] c_debug_level_enter_step_out<1>        Debug level enter request caused by step
 * @param[out] c_debug_level_leave_out<1>             Debug level leave request (triggered by dret)
 * @param[out] c_csr_bus_adr_out<PN_CFG_CSR_BUS_ADR_WIDTH> CSR-bus address
 * @param[out] c_csr_bus_write_en_out<1>              CSR-bus write enable
 * @param[out] c_csr_bus_read_en_out<1>               CSR-bus read enable
 * @param[out] c_csr_imm_en_out<1>                    CSR immediate value enable
 * @param[out] c_csr_imm_out<5>                       CSR immediate value
 * @param[out] c_csr_write_mode_out<2>                CSR write mode
 * @param[out] c_csr_bus_wdata_out<PN_CFG_CSR_BUS_DATA_WIDTH> CSR-bus write data
 * @param[out] c_csr_interrupt_out<1>                 Signal for interrupt handling to CSR module
 * @param[out] c_csr_mret_out<1>                      Signal for MRET instruction execution to CSR module
 * @param[out] c_trap_handler_enter_out<1>            Signal to initiate trap handler entry sequence
 */

#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <systemc.h>
#include <piconut.h>

#include <stdint.h>

#include "typedef.h"
#include "csr.h"

/* States */
typedef enum
{
    STATE_RESET,
    STATE_IPORT_STB,
    STATE_AWAIT_IPORT_ACK,
    STATE_DECODE,
    STATE_ALU,
    STATE_ALU_IMM,
    STATE_LOAD1,
    STATE_LOAD2,
    STATE_LOAD3,
    STATE_LOAD4,
    STATE_STORE1,
    STATE_STORE2,
    STATE_STORE3,
    STATE_BRANCH,
    STATE_DONT_BRANCH,
    STATE_JAL,
    STATE_JALR,
    STATE_NOP,
    STATE_HALT,
    STATE_LUI,
    STATE_AUIPC,
    // Debug
    STATE_DEBUG_HALTREQUEST1,
    STATE_DEBUG_HALTREQUEST2,
    STATE_DEBUG_STEP,
    // CSR
    STATE_CSRRW1,
    STATE_CSRRW2,
    STATE_CSRRC1,
    STATE_CSRRC2,
    STATE_CSRRS1,
    STATE_CSRRS2,
    STATE_CSRRWI1,
    STATE_CSRRWI2,
    STATE_CSRRCI1,
    STATE_CSRRCI2,
    STATE_CSRRSI1,
    STATE_CSRRSI2,
    // A-EXT
    STATE_LR1,
    STATE_LR2,
    STATE_LR3,
    STATE_AMO_LOAD1,
    STATE_AMO_LOAD2,
    STATE_AMO_LOAD3,
    STATE_AMO_OP,
    STATE_AMO_STORE1,
    STATE_AMO_STORE2,
    STATE_AMO_STORE3,
    STATE_A_STORE_RDATA,
    STATE_SC1,
    STATE_SC2,
    STATE_SC3,
    // System
    STATE_EBREAK,
    STATE_ECALL,
    STATE_DRET,
    // Trap/Exception handling
    STATE_TRAP_ENTRY,
    STATE_TRAP_SAVE_PC,
    STATE_MRET,

    STATE_TRAP_SAVE_MCAUSE,
    STATE_TRAP_UPDATE_MSTATUS,
    STATE_TRAP_UPDATE_MSTATUS2,
    STATE_TRAP_LOAD_HANDLER,
    // At max 2^STATE_WIDTH states
} e_controller_states;
#define STATE_WIDTH 6

typedef enum
{
    BRANCH_EQ = 0x0,
    BRANCH_NE = 0x1,
    BRANCH_LT = 0x4,
    BRANCH_GE = 0x5,
    BRANCH_LTU = 0x6,
    BRANCH_GEU = 0x7,
} e_controller_branch_type;

typedef enum
{
    SYSTEM_INSTR_ECALL = 0x00000073,
    SYSTEM_INSTR_EBREAK = 0x00100073,
    SYSTEM_INSTR_DRET = 0x7b200073,
    SYSTEM_INSTR_MRET = 0x30200073,
} e_controller_system_instructions;

typedef enum
{
    INTERRUPT_TYPE_MACHINE_SOFTWARE = 3,
    INTERRUPT_TYPE_MACHINE_TIMER = 7,
    INTERRUPT_TYPE_MACHINE_EXTERNAL = 11,
} e_interrupt_type;

SC_MODULE(m_controller)
{
public:
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    /* ---------------  Status signals ---------------  */
    sc_in<sc_uint<32>> PN_NAME(s_instruction_in);

    sc_in<bool> PN_NAME(s_alu_less_in);
    sc_in<bool> PN_NAME(s_alu_lessu_in);
    sc_in<bool> PN_NAME(s_alu_equal_in);
    sc_in<bool> PN_NAME(s_dport_ack_in);
    sc_in<bool> PN_NAME(s_iport_ack_in);

    /* Debug */
    sc_in<bool> PN_NAME(s_debug_haltrequest_in);
    sc_in<bool> PN_NAME(s_debug_step_in);

    /* --------------- Control signals ---------------  */

    /* IPort/DPort */
    sc_out<bool> PN_NAME(c_iport_stb_out);
    sc_out<bool> PN_NAME(c_dport_stb_out);
    sc_out<bool> PN_NAME(c_dport_we_out);
    sc_out<bool> PN_NAME(c_dport_lrsc_out);
    sc_out<bool> PN_NAME(c_dport_amo_out);

    /* Port Registers */
    sc_out<bool> PN_NAME(c_ld_en_rdata_out);
    sc_out<bool> PN_NAME(c_ld_en_wdata_out);
    sc_out<bool> PN_NAME(c_ld_en_adr_bsel_out);

    /* Regfile input */
    sc_out<bool> PN_NAME(c_reg_ldpc_out);
    sc_out<bool> PN_NAME(c_reg_ldmem_out);
    sc_out<bool> PN_NAME(c_reg_ldimm_out);
    sc_out<bool> PN_NAME(c_reg_ldalu_out);
    sc_out<bool> PN_NAME(c_reg_ldcsr_out);

    /* Regfile load enable*/
    sc_out<bool> PN_NAME(c_reg_ld_en_out);

    /* ALU */
    sc_out<bool> PN_NAME(c_alu_pc_out);
    sc_out<bool> PN_NAME(c_alu_imm_out);
    sc_out<bool> PN_NAME(c_alu_rdata_reg_out);
    sc_out<bool> PN_NAME(c_rs1_adr_bsel_out);     // ALU operand B is zero
    sc_out<bool> PN_NAME(c_alu_out_to_wdata_out); // ALU output to wdata
    sc_out<bool> PN_NAME(c_force_add_out);
    sc_out<bool> PN_NAME(c_force_amo_out);
    sc_out<sc_uint<3>> PN_NAME(c_alu_mode_out);

    /* Program counter */
    sc_out<bool> PN_NAME(c_pc_inc4_out);
    sc_out<bool> PN_NAME(c_pc_ld_en_out);

    /* Instruction register */
    sc_out<bool> PN_NAME(c_ir_ld_en_out);

    /* Debug */
    sc_out<bool> PN_NAME(c_debug_haltrequest_ack_out);
    sc_out<bool> PN_NAME(c_debug_level_enter_ebreak_out);
    sc_out<bool> PN_NAME(c_debug_level_enter_haltrequest_out);
    sc_out<bool> PN_NAME(c_debug_level_enter_step_out);
    sc_out<bool> PN_NAME(c_debug_level_leave_out); // Note: Triggerd when dret instruction is hit.

    /* CSR */
    sc_out<sc_uint<PN_CFG_CSR_BUS_ADR_WIDTH>> PN_NAME(c_csr_bus_adr_out);
    sc_out<bool> PN_NAME(c_csr_bus_write_en_out);
    sc_out<bool> PN_NAME(c_csr_bus_read_en_out);
    sc_out<bool> PN_NAME(c_csr_imm_en_out);
    sc_out<sc_uint<5>> PN_NAME(c_csr_imm_out);
    sc_out<sc_uint<2>> PN_NAME(c_csr_write_mode_out);

    // CSR Bus write data signal for direct CSR access
    sc_out<sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(c_csr_bus_wdata_out);

    // Interrupt handling signals
    sc_in<bool> PN_NAME(s_interrupt_pending_in); // Tracks if an interrupt is pending

    sc_out<bool> PN_NAME(c_csr_interrupt_out); // Signal for interrupt handling
    sc_out<bool> PN_NAME(c_csr_mret_out);      // Signal for MRET instruction

    // Trap handling
    sc_out<bool> PN_NAME(c_trap_handler_enter_out); // Signal to load trap handler address
    sc_in<bool> PN_NAME(trap_handler_enter_in);

    // Interrupt status inputs from CSR
    sc_in<bool> PN_NAME(s_mip_msip_in);    // Software interrupt pending
    sc_in<bool> PN_NAME(s_mie_msie_in);    // Software interrupt enable
    sc_in<bool> PN_NAME(s_mip_mtip_in);    // Timer interrupt pending
    sc_in<bool> PN_NAME(s_mie_mtie_in);    // Timer interrupt enable
    sc_in<bool> PN_NAME(s_mip_meip_in);    // External interrupt pending
    sc_in<bool> PN_NAME(s_mie_meie_in);    // External interrupt enable
    sc_in<bool> PN_NAME(s_mstatus_mie_in); // Global machine interrupt enable (new)

    /* Constructor... */
    SC_CTOR(m_controller)
    {
        c_trap_handler_enter_out.initialize(false);

        SC_CTHREAD(proc_clk_controller, clk.pos());
        reset_signal_is(reset, true);
        SC_METHOD(proc_cmb_controller);
        sensitive << s_instruction_in << s_alu_less_in << s_alu_lessu_in
                  << s_alu_equal_in << s_dport_ack_in << s_iport_ack_in
                  << state << s_debug_haltrequest_in
                  << s_debug_step_in << s_interrupt_pending_in
                  << s_mip_msip_in << s_mie_msie_in << s_mip_meip_in
                  << s_mie_meie_in << s_mip_mtip_in << s_mie_mtie_in
                  << s_mstatus_mie_in;
    }

    void pn_trace(sc_trace_file * tf, int level = 1);

    /* Process functions */
    void proc_cmb_controller();
    void proc_clk_controller();

    /* Helper functions */
    uint8_t get_state() const;
    sc_uint<6> get_state_signal() const;
    std::string state_to_string(sc_uint<32> state) const;

protected:
    /* Helper functions */
    void handle_state_decode();
    void handle_branch_decode();
    void handle_system_decode();
    void handle_a_ext_decode();
    void instruction_finished();

protected:
    sc_signal<sc_uint<STATE_WIDTH>> PN_NAME(state);
    sc_signal<sc_uint<STATE_WIDTH>> PN_NAME(next_state);
#undef STATE_WIDTH
};

#endif //__CONTROLLER_H__
