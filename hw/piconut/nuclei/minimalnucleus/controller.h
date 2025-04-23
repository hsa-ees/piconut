/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lorenz Sommer <lorenz.sommer@tha.de>
                     2024 Johannes Hofmann <johannes.hofmann1@tha.de>
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

 /** Controller
 *
 * This module is the main state machine of the processor.
 * It reads various status signals from other modules and controls how the these
 * modules behave with control signals.
 *
 * Note: The syntax "s_<signal_name>" and "c_<signal_name>" is used to denote
 * status and controls signals respectively. Status signals are the outputs of
 * various modules while control signals are the controllers outputs.
 *
 * Note: "current instruction" refers to the value held by the instruction
 * register at the current cycle.
 *
 * Input ports:
 *  - s_instruction_in<32>: The current instruction.
 *
 *  - s_alu_less_in<1>:    High if ALU operand A is less than operand B.
 *  - s_alu_lessu_in<1>:   High ALU operand A is less than operand B (unsigned).
 *  - s_alu_equal_in<1>:   High ALU operand A is equal to operand B.
 *
 *  - s_dport_ack_in<1>:   Data port acknowledge signal.
 *  - s_iport_ack_in<1>:   Instruction port acknowledge signal.
 *
 *  - s_debug_haltrequest_in<1>: Halt request for debugging.
 *  - s_debug_step_in<1>:        Debug stepping active.
 *
 * Output ports:
 *  - c_iport_stb_out<1>:   Instruction port strobe signal.
 *  - c_dport_stb_out<1>:   Data port strobe signal.
 *  - c_dport_we_out<1>:    Data port write enable signal.
 *
 *  - c_pc_ld_en_out<1>:    Enables the program counter to load a new value.
 *  - c_pc_inc4_out<1>:     Increments the program counter by 4 come the next
 * rising edge.
 *
 *  - c_reg_ld_en_out<1>:   Enables the register file to load a value.
 *  - c_reg_ldmem_out<1>:   Enables the register file to load a value from
 * memory
 *  - c_reg_ldimm_out<1>:   Enables the register file to load an immediate
 * value.
 *  - c_reg_ldalu_out<1>:   Enables the register file to load a value from the
 * ALU.
 *  - c_reg_ldpc_out<1>:    Enables the register file to load a value from the
 * program counter.
 *  - c_reg_ldcsr_out<1>:   Enables the register file to load a value from the
 * CSR-bus read signal.
 *
 *  - c_alu_pc_out<1>:      Direct the program counter to be ALU operand A.
 *  - c_alu_imm_out<1>:     Direct the immediate value to be ALU operand B.
 *  - c_force_add_out<1>:   Force the ALU to perform an addition.
 *
 *  - c_ir_ld_en_out<1>:       Enables the instruction register to load a new
 * value from memory.
 *
 *  - c_debug_haltrequest_ack_out<1>:         Acknowledge halt request for debugging.
 *  - c_debug_level_enter_ebreak_out<1>:      Debug level enter request caused by ebreak.
 *  - c_debug_level_enter_haltrequest_out<1>: Debug level enter request caused by halt request.
 *  - c_debug_level_enter_step_out<1>:        Debug level enter request caused by step.
 *  - c_debug_level_enter_leave_out<1>:       Debug level leave request.
 *
 *  - c_csr_bus_adr_out<CFG_CSR_BUS_ADR_WIDTH>: CSR-bus address.
 *  - c_csr_bus_write_en_out<1>:                CSR-bus write enable.
 *  - c_csr_bus_read_en_out<1>:                 CSR-bus read enable.
 *  - c_csr_imm_en_out<1>:                      CSR immediate value enable.
 *  - c_csr_imm_out<5>:                         CSR immediate value.
 *  - c_csr_write_mode_out<2>:                  CSR write mode.
 *
 */

#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <systemc.h>
#include <stdint.h>
#include <base.h>
#include <piconut-config.h>
#include <typedef.h>

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
    // System
    STATE_EBREAK,
    STATE_ECALL,
    STATE_DRET,
} e_controller_states;

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
} e_controller_system_instructions;

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
    sc_out<bool> PN_NAME(c_force_add_out);
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

    /* Csr */
    sc_out<sc_uint<CFG_CSR_BUS_ADR_WIDTH>> PN_NAME(c_csr_bus_adr_out);
    sc_out<bool> PN_NAME(c_csr_bus_write_en_out);
    sc_out<bool> PN_NAME(c_csr_bus_read_en_out);
    sc_out<bool> PN_NAME(c_csr_imm_en_out);
    sc_out<sc_uint<5>> PN_NAME(c_csr_imm_out);
    sc_out<sc_uint<2>> PN_NAME(c_csr_write_mode_out);

    /* Constructor... */
    SC_CTOR(m_controller)
    {
        SC_CTHREAD(proc_clk_controller, clk.pos());
        reset_signal_is(reset, true);
        SC_METHOD(proc_cmb_controller);
        sensitive << state << s_instruction_in << s_alu_less_in
                  << s_alu_lessu_in << s_alu_equal_in << s_dport_ack_in
                  << s_iport_ack_in << s_debug_haltrequest_in
                  << s_debug_step_in;
    }

    void Trace(sc_trace_file * tf, int level = 1);

    /* Process functions */
    void proc_cmb_controller();
    void proc_clk_controller();

    /* Helper functions */
    uint8_t get_state() const;
    sc_uint<6> get_state_signal() const;
    std::string state_to_string(uint32_t state) const;

protected:
    /* Helper functions */
    void handle_state_decode();
    void handle_branch_decode();
    void handle_system_decode();
    void instruction_finished();

protected:
    sc_signal<sc_uint<6>> PN_NAME(state);
    sc_signal<sc_uint<6>> PN_NAME(next_state);
};

#endif //__CONTROLLER_H__