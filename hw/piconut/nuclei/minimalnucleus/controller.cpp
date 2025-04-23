/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lorenz Sommer <lorenz.sommer@tha.de>
                     2024 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg


  Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *************************************************************************/

#include "controller.h"


 namespace {
    sc_uint<5> opcode(sc_uint<32> instruction)
    {
        return instruction.range(6, 2);
    }

    sc_uint<3> funct3(sc_uint<32> instruction)
    {
        return instruction.range(14, 12);
    }

    sc_uint<7> funct7(sc_uint<32> instruction)
    {
        return instruction.range(31, 25);
    }

    sc_uint<12> funct12(sc_uint<32> instruction)
    {
        return instruction.range(31, 20);
    }

    sc_uint<5> rd(sc_uint<32> instruction)
    {
        return instruction.range(11, 7);
    }

    sc_uint<5> rs1(sc_uint<32> instruction)
    {
        return instruction.range(19, 15);
    }
 }

void m_controller::Trace(sc_trace_file* tf, int level)
{

    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);

    /* ---------------  Status signals ---------------  */
    PN_TRACE(tf, s_instruction_in);
    PN_TRACE(tf, s_alu_less_in);
    PN_TRACE(tf, s_alu_lessu_in);
    PN_TRACE(tf, s_alu_equal_in);
    PN_TRACE(tf, s_iport_ack_in);
    PN_TRACE(tf, s_dport_ack_in);

    /* Debug */
    PN_TRACE(tf, s_debug_haltrequest_in);
    PN_TRACE(tf, s_debug_step_in);

    /* --------------- Control signals ---------------  */
    /* IPort/DPort */
    PN_TRACE(tf, c_iport_stb_out);
    PN_TRACE(tf, c_dport_stb_out);
    PN_TRACE(tf, c_dport_we_out);

    /* Regfile input */
    PN_TRACE(tf, c_reg_ldpc_out);
    PN_TRACE(tf, c_reg_ldmem_out);
    PN_TRACE(tf, c_reg_ldimm_out);
    PN_TRACE(tf, c_reg_ldalu_out);
    PN_TRACE(tf, c_reg_ldcsr_out);

    /* Regfile load enable */
    PN_TRACE(tf, c_reg_ld_en_out);

    /* ALU */
    PN_TRACE(tf, c_alu_pc_out);
    PN_TRACE(tf, c_alu_imm_out);
    PN_TRACE(tf, c_force_add_out);

    /* Program counter*/
    PN_TRACE(tf, c_pc_inc4_out);
    PN_TRACE(tf, c_pc_ld_en_out);

    /* Instruction register */
    PN_TRACE(tf, c_ir_ld_en_out);

    /* Debug */
    PN_TRACE(tf, c_debug_haltrequest_ack_out);
    PN_TRACE(tf, c_debug_level_enter_ebreak_out);
    PN_TRACE(tf, c_debug_level_enter_haltrequest_out);
    PN_TRACE(tf, c_debug_level_enter_step_out);
    PN_TRACE(tf, c_debug_level_leave_out);

    /* Csr */
    PN_TRACE(tf, c_csr_bus_adr_out);
    PN_TRACE(tf, c_csr_bus_write_en_out);
    PN_TRACE(tf, c_csr_bus_read_en_out);
    PN_TRACE(tf, c_csr_imm_en_out);
    PN_TRACE(tf, c_csr_imm_out);
    PN_TRACE(tf, c_csr_write_mode_out);

    /* Internal states */
    PN_TRACE(tf, state);
    PN_TRACE(tf, next_state);
}

void m_controller::proc_cmb_controller()
{

    // Default values
    c_iport_stb_out = 0x0;
    c_dport_stb_out = 0x0;
    c_dport_we_out = 0x0;
    c_alu_imm_out = 0x0;
    c_alu_pc_out = 0x0;
    c_force_add_out = 0x0;
    c_reg_ld_en_out = 0x0;
    c_reg_ldalu_out = 0x0;
    c_reg_ldmem_out = 0x0;
    c_reg_ldimm_out = 0x0;
    c_reg_ldpc_out = 0x0;
    c_reg_ldcsr_out = 0x0;
    c_pc_inc4_out = 0x0;
    c_pc_ld_en_out = 0x0;
    c_ir_ld_en_out = 0x0;
    c_alu_mode_out = 0x0;

    // Debug
    c_debug_haltrequest_ack_out = 0x0;
    c_debug_level_enter_ebreak_out = 0x0;
    c_debug_level_enter_haltrequest_out = 0x0;
    c_debug_level_enter_step_out = 0x0;
    c_debug_level_leave_out = 0x0;

    // Csr
    c_csr_bus_adr_out = 0x0;
    c_csr_bus_write_en_out = 0x0;
    c_csr_bus_read_en_out = 0x0;
    c_csr_imm_en_out = 0x0;
    c_csr_imm_out = 0x0;
    c_csr_write_mode_out = 0x0;

    next_state = state;

    switch(state.read())
    {
        case STATE_RESET:
            next_state = STATE_IPORT_STB;
            break;
        case STATE_IPORT_STB:

            //  Set IPort stb signal to request next instruction
            c_iport_stb_out = 0x1;

            // Go to FETCH2
            next_state = STATE_AWAIT_IPORT_ACK;
            break;

        case STATE_AWAIT_IPORT_ACK:
            // Set strobe low
            c_iport_stb_out = 0;
            // Set c_ir_ld_en_out to 1 -> instruction can be loaded
            c_ir_ld_en_out = 0x1;
            //  Wait for memu to acknowledge
            if(s_iport_ack_in.read() == 1)
            {
                // IPort acknowledged, instruction is ready, go to decode
                next_state = STATE_DECODE;
            }
            else
            {
                // Idle in FETCH2 and wait for IPort ACK
                next_state = STATE_AWAIT_IPORT_ACK;
            }
            break;

        case STATE_DECODE:
            // Instruction is in IR
            // Determine instruction type and set next state accordingly
            // A separate function is used to not clutter this code any more than necessary
            handle_state_decode();
            break;

        case STATE_ALU:
            //  Set control registers for R-Type ALU instructions
            c_pc_inc4_out = 1;   // increment PC by 4
            c_reg_ld_en_out = 1; // regfile load enable
            c_reg_ldalu_out = 1; // set Regfile input to ALU
            c_alu_mode_out = ALU_MODE_REG_REG;

            instruction_finished();
            break;

        case STATE_ALU_IMM:
            //  Set control registers for I-Type ALU instructions

            c_pc_inc4_out = 1;   // increment PC by 4
            c_reg_ld_en_out = 1; // regfile load enable
            c_reg_ldalu_out = 1; // set ALU operand A from regfile
            c_alu_imm_out = 1;   // set ALU operand B to be the decoded immediate value
            c_alu_mode_out = ALU_MODE_REG_IMM;

            instruction_finished();
            break;

        case STATE_LUI:
            //  rd = imm
            c_reg_ld_en_out = 1; // regfile load enable
            c_reg_ldimm_out = 1; // load immediate value into register
            c_pc_inc4_out = 1;   // increment PC by 4

            instruction_finished();
            break;

        case STATE_AUIPC:
            //  rd = pc + imm
            c_reg_ldalu_out = 1; // set regfile input to be ALU output
            c_reg_ld_en_out = 1; // regfile load enable
            c_pc_inc4_out = 1;   // advance PC
            c_alu_pc_out = 1;    // set ALU operand A to be the PC
            c_alu_imm_out = 1;   // set ALU operand B to the decoded immediate value
            c_force_add_out = 1; // force ALU to add

            instruction_finished();
            break;

        case STATE_JAL:
            // rd = PC + 4, PC = PC + imm

            // PC = PC + imm
            c_alu_imm_out = 1;   // set ALU operand B to the decoded immediate value
            c_alu_pc_out = 1;    // set ALU operand A to be the PC
            c_force_add_out = 1; // force ALU to add
            c_pc_ld_en_out = 1;  // enable PC to load new value

            // rd = PC + 4
            c_reg_ldpc_out = 1;  // set regfile input to PC+4
            c_reg_ld_en_out = 1; // regfile load enable

            instruction_finished();
            break;

        case STATE_JALR:
            //  rd = PC + 4, pc = rs1 + imm
            c_force_add_out = 1; // force ALU to add
            // PC = RS1 + IMM
            c_alu_imm_out = 1;  // set ALU operand B to the decoded immediate value
            c_pc_ld_en_out = 1; // enable PC to load the ALU output
            // rd = PC + 4
            c_reg_ldpc_out = 1;  // load PC + 4 into register file
            c_reg_ld_en_out = 1; // regfile load enable

            instruction_finished();
            break;

        case STATE_BRANCH:
            c_force_add_out = 1; // force ALU to add
            c_pc_ld_en_out = 1;  // enable PC to load the ALU output
            c_alu_pc_out = 1;    // set ALU operand A to be the PC
            c_alu_imm_out = 1;   // set ALU operand B to the decoded immediate value

            instruction_finished();
            break;

        case STATE_DONT_BRANCH:
            c_pc_inc4_out = 1; // increment PC by 4
            instruction_finished();
            break;

        case STATE_LOAD1:
            // Await dport ack = 0 -> bus is ready for a new transaction
            if(s_dport_ack_in.read() == 0)
            {
                next_state = STATE_LOAD2;
            }
            else
            {
                // If ack != 0, remain
                next_state = STATE_LOAD1;
            }
            break;

        case STATE_LOAD2:
            // Now the bus is ready for a new transaction

            // Set dport strobe to begin transaction
            c_dport_stb_out = 1;
            // Let ALU calculate the target address (rs1 + imm)
            c_force_add_out = 1;
            c_alu_imm_out = 1;
            // advance PC
            c_pc_inc4_out = 1;

            next_state = STATE_LOAD3;
            break;

        case STATE_LOAD3:
            // Wait for dport ack

            // Let ALU calculate the target address (rs1 + imm)
            c_force_add_out = 1;
            c_alu_imm_out = 1;

            // Await dport ack = 1
            if(s_dport_ack_in.read() == 1)
            {
                // Move to next instruction
                next_state = STATE_LOAD4;
            }
            else
            {
                // If ack != 1, remain
                next_state = STATE_LOAD3;
            }
            break;

        case STATE_LOAD4:
            // ack was received ->  valid data is on the bus lines and it can be stored in the register file

            // Set control signals
            c_reg_ldmem_out = 1; // set regfile input to memory
            c_reg_ld_en_out = 1; // regfile load enable
            c_alu_imm_out = 1;   // set ALU operand B to the decoded immediate value
            c_force_add_out = 1; // force ALU to add

            instruction_finished();
            break;

        case STATE_STORE1:
            //  Wait for dport ack = 0 -> bus is ready for a new transaction
            if(s_dport_ack_in.read() == 0)
            {
                next_state = STATE_STORE2;
            }
            else
            {
                // If ack != 0, remain
                next_state = STATE_STORE1;
            }
            break;

        case STATE_STORE2:
            // Bus is ready for a new transaction

            // Set dport strobe to begin transaction
            c_dport_stb_out = 1;
            // Set dport write enable
            c_dport_we_out = 1;
            // Let ALU calculate the target address (rs1 + imm)
            c_force_add_out = 1;
            c_alu_imm_out = 1;
            // advance PC
            c_pc_inc4_out = 1;

            next_state = STATE_STORE3;

            break;

        case STATE_STORE3:
            // Set dport write enable
            c_dport_we_out = 1;
            // Let ALU calculate the target address (rs1 + imm)
            c_force_add_out = 1;
            c_alu_imm_out = 1;

            // Await dport ack = 1 -> the transaction is complete and data has been stored
            if(s_dport_ack_in.read() == 1)
            {
                // Move to next instruction
                instruction_finished();
            }
            else
            {
                // If ack != 1, remain
                next_state = STATE_STORE3;
            }

            break;

        case STATE_NOP:
            // Do nothing
            c_pc_inc4_out = 1;

            instruction_finished();
            break;

            /* Debug -------------------------------------------------------- */
        case STATE_DEBUG_HALTREQUEST1:
            c_debug_haltrequest_ack_out = 1;
            c_debug_level_enter_haltrequest_out = 1;

            next_state = STATE_DEBUG_HALTREQUEST2;
            break;

        case STATE_DEBUG_HALTREQUEST2:

            if(s_debug_haltrequest_in.read() == 0)
            {
                next_state = STATE_IPORT_STB;
            }
            else
            {
                next_state = STATE_DEBUG_HALTREQUEST2;
            }
            break;

        case STATE_DEBUG_STEP:
            c_debug_level_enter_step_out = 1;

            next_state = STATE_IPORT_STB;
            break;

            /* CSR -----------------------------------------------------------*/
        case STATE_CSRRW1:
            if(rd(s_instruction_in.read()) != 0x0)
            {
                // first, read CSR and store it in rd
                // begin CSR bus transaction with setting of csr_rd_out
                c_csr_bus_read_en_out = 1; // set rd high
                c_csr_bus_adr_out = funct12(s_instruction_in.read());
            }

            next_state = STATE_CSRRW2;
            break;

        case STATE_CSRRW2:
            if(rd(s_instruction_in.read()) != 0x0)
            {
                // CSR responds within one clock cycle
                // set control signals to store CSR value in rd
                c_reg_ld_en_out = 1; // regfile load enable
                c_reg_ldcsr_out = 1; // set regfile input to csr_rdata
            }

            // Write rs1 to Csr
            c_csr_bus_write_en_out = 1;
            c_csr_bus_adr_out = funct12(s_instruction_in.read());
            c_csr_write_mode_out = e_csr_write_mode::CSR_WRITE_ALL;

            c_pc_inc4_out = 1; // increment PC by 4

            instruction_finished();
            break;

        case STATE_CSRRC1:
            // begin CSR bus transaction with setting of csr_rd_out
            c_csr_bus_read_en_out = 1; // set rd high
            c_csr_bus_adr_out = funct12(s_instruction_in.read());
            next_state = STATE_CSRRC2;
            break;

        case STATE_CSRRC2:
            // CSR responds within one clock cycle
            // CSR value is now present on csr_rdata and can be stored
            c_reg_ld_en_out = 1; // regfile load enable
            c_reg_ldcsr_out = 1; // set regfile input to ALU

            if(rs1(s_instruction_in.read()) != 0x0)
            {
                // begin writing back to CSR
                c_csr_bus_write_en_out = 1; // set csr_we high
                c_csr_bus_adr_out = funct12(s_instruction_in.read());
                c_csr_write_mode_out = e_csr_write_mode::CSR_WRITE_CLEAR; // signal CSR to clear bits (AND with 0x0)
            }

            c_pc_inc4_out = 1; // increment PC by 4

            // CSR will take on new value within one clock cycle, no wait needed
            instruction_finished();
            break;

        case STATE_CSRRS1:
            // first, read CSR and store it in rd
            // begin CSR bus transaction with setting of csr_rd_out
            c_csr_bus_read_en_out = 1; // set rd high
            c_csr_bus_adr_out = funct12(s_instruction_in.read());
            next_state = STATE_CSRRS2;
            break;

        case STATE_CSRRS2:
            // CSR responds within one clock cycle
            // CSR value is now present on csr_rdata and can be stored
            c_reg_ld_en_out = 1; // regfile load enable
            c_reg_ldcsr_out = 1; // set regfile input to ALU

            if(rs1(s_instruction_in.read()) != 0x0)
            {
                // begin writing back to CSR
                c_csr_bus_write_en_out = 1; // set csr_we high
                c_csr_bus_adr_out = funct12(s_instruction_in.read());
                c_csr_write_mode_out = e_csr_write_mode::CSR_WRITE_SET; // signal CSR to clear bits (AND with 0x0)
            }

            c_pc_inc4_out = 1; // increment PC by 4

            // CSR will take on new value within one clock cycle, no wait needed
            instruction_finished();
            break;

        case STATE_CSRRWI1:
            if(rd(s_instruction_in.read()) != 0x0)
            {
                // first, read CSR and store it in rd
                // begin CSR bus transaction with setting of csr_rd_out
                c_csr_bus_read_en_out = 1; // set rd high
                c_csr_bus_adr_out = funct12(s_instruction_in.read());
            }

            next_state = STATE_CSRRWI2;
            break;

        case STATE_CSRRWI2:
            if(rd(s_instruction_in.read()) != 0x0)
            {
                // CSR responds within one clock cycle
                // set control signals to store CSR value in rd
                c_reg_ld_en_out = 1; // regfile load enable
                c_reg_ldcsr_out = 1; // set regfile input to csr_rdata
            }

            // Write imm to Csr
            c_csr_bus_write_en_out = 1;
            c_csr_bus_adr_out = funct12(s_instruction_in.read());
            c_csr_imm_en_out = 1;
            c_csr_imm_out = rs1(s_instruction_in.read()); // Immediate value is at the same location as reg source
            c_csr_write_mode_out = e_csr_write_mode::CSR_WRITE_ALL;

            c_pc_inc4_out = 1; // increment PC by 4

            instruction_finished();
            break;

        case STATE_CSRRCI1:
            // begin CSR bus transaction with setting of csr_rd_out
            c_csr_bus_read_en_out = 1; // set rd high
            c_csr_bus_adr_out = funct12(s_instruction_in.read());
            next_state = STATE_CSRRCI2;
            break;

        case STATE_CSRRCI2:
            // CSR responds within one clock cycle
            // CSR value is now present on csr_rdata and can be stored
            c_reg_ld_en_out = 1; // regfile load enable
            c_reg_ldcsr_out = 1; // set regfile input to ALU

            if(rs1(s_instruction_in.read()) != 0x0)
            {
                // begin writing back to CSR
                c_csr_bus_write_en_out = 1; // set csr_we high
                c_csr_bus_adr_out = funct12(s_instruction_in.read());
                c_csr_imm_en_out = 1;
                c_csr_imm_out = rs1(s_instruction_in.read());                               // Immediate value is at the same location as reg source
                c_csr_write_mode_out = e_csr_write_mode::CSR_WRITE_CLEAR; // signal CSR to clear bits (AND with 0x0)
            }

            c_pc_inc4_out = 1; // increment PC by 4

            // CSR will take on new value within one clock cycle, no wait needed
            instruction_finished();
            break;

        case STATE_CSRRSI1:
            // first, read CSR and store it in rd
            // begin CSR bus transaction with setting of csr_rd_out
            c_csr_bus_read_en_out = 1; // set rd high
            c_csr_bus_adr_out = funct12(s_instruction_in.read());
            next_state = STATE_CSRRSI2;
            break;

        case STATE_CSRRSI2:
            // CSR responds within one clock cycle
            // CSR value is now present on csr_rdata and can be stored
            c_reg_ld_en_out = 1; // regfile load enable
            c_reg_ldcsr_out = 1; // set regfile input to ALU

            if(rs1(s_instruction_in.read()) != 0x0)
            {
                // begin writing back to CSR
                c_csr_bus_write_en_out = 1; // set csr_we high
                c_csr_bus_adr_out = funct12(s_instruction_in.read());
                c_csr_imm_en_out = 1;
                c_csr_imm_out = rs1(s_instruction_in.read());                             // Immediate value is at the same location as reg source
                c_csr_write_mode_out = e_csr_write_mode::CSR_WRITE_SET; // signal CSR to clear bits (AND with 0x0)
            }

            c_pc_inc4_out = 1; // increment PC by 4

            // CSR will take on new value within one clock cycle, no wait needed
            instruction_finished();
            break;

            /* System ------------------------------------------------------- */
        case STATE_EBREAK:
            c_debug_level_enter_ebreak_out = 1;

            next_state = STATE_IPORT_STB;
            break;

        case STATE_ECALL:
            // Not implemented yet

            next_state = STATE_HALT;
            break;

        case STATE_DRET:
            c_debug_level_leave_out = 1;

            next_state = STATE_IPORT_STB;
            break;

        case STATE_HALT:
            PN_INFOF(("State: HALT"));
            // Do nothing and remain in HALT state
            next_state = STATE_HALT;
            break;

        default:
            next_state = 0x0;
            PN_ERROR("Controller: Invalid state.");
            break;
    }
}
void m_controller::proc_clk_controller()
{
    // Reset internal register
    state = STATE_RESET;

    while(true)
    {
        wait();
        // State transition
        state = next_state.read();
    }
}

void m_controller::handle_state_decode()
{
    // Read opcode and set next state accordingly
    switch(opcode(s_instruction_in.read()))
    {
        case OP_ALU:

            switch(funct7(s_instruction_in.read()))
            {
                case FUNCT7_RV32I_BASE:
                    next_state = STATE_ALU;
                    break;

                case FUNCT7_RV32I_SRA_SUB:
                    next_state = STATE_ALU;
                    break;

                default:
                    next_state = STATE_HALT;
                    PN_ERROR("Controller: Invalid instruction.");
                    break;
            }
            break;
        case OP_ALUI:
            next_state = STATE_ALU_IMM;
            break;
        case OP_LUI:
            next_state = STATE_LUI;
            break;
        case OP_AUIPC:
            next_state = STATE_AUIPC;
            break;
        case OP_JAL:
            next_state = STATE_JAL;
            break;
        case OP_JALR:
            next_state = STATE_JALR;
            break;
        case OP_LOAD:
            next_state = STATE_LOAD1;
            break;
        case OP_STORE:
            next_state = STATE_STORE1;
            break;
        case OP_BRANCH:
            handle_branch_decode();
            break;
        case OP_SYSTEM:
            handle_system_decode();
            break;
        case OP_FENCE:
            next_state = STATE_NOP;
            break;
        default:
            PN_ERROR("Controller: Unknown opcode.");
            next_state = STATE_HALT;
            break;
    }
}

void m_controller::handle_branch_decode()
{
    // Read funct3 from opcode and determine whether to branch via the ALU flags
    switch(funct3(s_instruction_in.read()))
    {
        case BRANCH_EQ:
            next_state =
                (s_alu_equal_in.read() == 1) ? STATE_BRANCH : STATE_DONT_BRANCH;
            break;
        case BRANCH_NE:
            next_state =
                (s_alu_equal_in.read() == 0) ? STATE_BRANCH : STATE_DONT_BRANCH;
            break;
        case BRANCH_LT:
            next_state = (s_alu_less_in.read() == 1) ? STATE_BRANCH : STATE_DONT_BRANCH;
            break;
        case BRANCH_GE:
            next_state = (s_alu_less_in.read() == 0) ? STATE_BRANCH : STATE_DONT_BRANCH;
            break;
        case BRANCH_LTU:
            next_state =
                (s_alu_lessu_in.read() == 1) ? STATE_BRANCH : STATE_DONT_BRANCH;
            break;
        case BRANCH_GEU:
            next_state =
                (s_alu_lessu_in.read() == 0) ? STATE_BRANCH : STATE_DONT_BRANCH;
            break;
        default:
            next_state = STATE_HALT;
            PN_ERROR("Controller: Unknown branch type.");
            break;
    }
}

void m_controller::handle_system_decode()
{
    if(s_instruction_in.read() == SYSTEM_INSTR_EBREAK)
    {
        next_state = STATE_EBREAK;
    }
    else if(s_instruction_in.read() == SYSTEM_INSTR_ECALL)
    {
        next_state = STATE_ECALL;
    }
    else if(s_instruction_in.read() == SYSTEM_INSTR_DRET)
    {
        next_state = STATE_DRET;
    }
    else // Csr instructions
    {
        switch(funct3(s_instruction_in.read()))
        {
            case FUNCT3_CSR_CSRRW:
                next_state = STATE_CSRRW1;
                break;
            case FUNCT3_CSR_CSRRS:
                next_state = STATE_CSRRS1;
                break;
            case FUNCT3_CSR_CSRRC:
                next_state = STATE_CSRRC1;
                break;
            case FUNCT3_CSR_CSRRWI:
                next_state = STATE_CSRRWI1;
                break;
            case FUNCT3_CSR_CSRRSI:
                next_state = STATE_CSRRSI1;
                break;
            case FUNCT3_CSR_CSRRCI:
                next_state = STATE_CSRRCI1;
                break;
            default:
                PN_ERROR("Controller: Unknown CSR instruction.");
                next_state = STATE_HALT;
                break;
        }
    }
}

/* Helper functions */
uint8_t m_controller::get_state() const
{
    return (uint8_t)state.read();
}

sc_uint<6> m_controller::get_state_signal() const
{
    return state.read();
}

// Convert state to string for debugging
std::string m_controller::state_to_string(uint32_t state) const
{
    // Note: This array needs to match the e_controller_states enum order.
    static const std::string state_names[] = {
        "STATE_RESET",
        "STATE_IPORT_STB",
        "STATE_AWAIT_IPORT_ACK",
        "STATE_DECODE",
        "STATE_ALU",
        "STATE_ALU_IMM",
        "STATE_LOAD1",
        "STATE_LOAD2",
        "STATE_LOAD3",
        "STATE_LOAD4",
        "STATE_STORE1",
        "STATE_STORE2",
        "STATE_STORE3",
        "STATE_BRANCH",
        "STATE_DONT_BRANCH",
        "STATE_JAL",
        "STATE_JALR",
        "STATE_NOP",
        "STATE_HALT",
        "STATE_LUI",
        "STATE_AUIPC",
        // Debug
        "STATE_DEBUG_HALTREQUEST1",
        "STATE_DEBUG_HALTREQUEST2",
        "STATE_DEBUG_STEP",
        // CSR
        "STATE_CSRRW1",
        "STATE_CSRRW2",
        "STATE_CSRRC1",
        "STATE_CSRRC2",
        "STATE_CSRRS1",
        "STATE_CSRRS2",
        "STATE_CSRRWI1",
        "STATE_CSRRWI2",
        "STATE_CSRRCI1",
        "STATE_CSRRCI2",
        "STATE_CSRRSI1",
        "STATE_CSRRSI2",
        // System
        "STATE_EBREAK",
        "STATE_ECALL",
        "STATE_DRET",
    };
    return state_names[state];
}

void m_controller::instruction_finished()
{
    // Execute debug routine when external haltrequst is active
    if(s_debug_haltrequest_in.read() == 0x1)
    {
        next_state = STATE_DEBUG_HALTREQUEST1;
    }
    // Execute debug routine when stepping is active
    else if(s_debug_step_in.read() == 0x1)
    {
        next_state = STATE_DEBUG_STEP;
    }
    // Execute standard fetch
    else
    {
        next_state = STATE_IPORT_STB;
    }
}
