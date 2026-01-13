/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lorenz Sommer <lorenz.sommer@tha.de>
                     2024 Johannes Hofmann <johannes.hofmann1@tha.de>
                     2025 Alexander Beck <alexander.beck1@tha.de>
                     2025 Christian Zellinger <christian.zellinger1@tha.de>
                     2025 Niklas Sirch  <niklas.sirch1@tha.de
                     2025 Daniel Sommerfeldt <daniel.sommerfeldt1@tha.de>
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
/* opcode without bits for compressed instructions */
sc_uint<5> opcode(sc_uint<32> instruction)
{
    return instruction.range(6, 2);
}

sc_uint<3> funct3(sc_uint<32> instruction)
{
    return instruction.range(14, 12);
}

sc_uint<5> funct5(sc_uint<32> instruction)
{
    return instruction.range(31, 27);
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
} // namespace

void m_controller::pn_trace(sc_trace_file* tf, int level)
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
    PN_TRACE(tf, c_dport_lrsc_out);
    PN_TRACE(tf, c_dport_amo_out);

    /* Port Registers */
    PN_TRACE(tf, c_ld_en_rdata_out);
    PN_TRACE(tf, c_ld_en_wdata_out);
    PN_TRACE(tf, c_ld_en_adr_bsel_out);

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
    PN_TRACE(tf, c_rs1_adr_bsel_out);
    PN_TRACE(tf, c_alu_rdata_reg_out);
    PN_TRACE(tf, c_alu_out_to_wdata_out);
    PN_TRACE(tf, c_force_amo_out);

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
    c_csr_interrupt_out = 0;

    // Default values
    c_iport_stb_out = 0x0;
    c_dport_stb_out = 0x0;
    c_dport_we_out = 0x0;
    c_dport_lrsc_out = 0x0;
    c_dport_amo_out = 0x0;
    c_alu_imm_out = 0x0;
    c_alu_pc_out = 0x0;
    c_force_add_out = 0x0;
    c_force_amo_out = 0x0;
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
    c_rs1_adr_bsel_out = 0x0;
    c_alu_rdata_reg_out = 0x0;
    c_alu_out_to_wdata_out = 0x0;

    c_ld_en_rdata_out = 0x0;
    c_ld_en_wdata_out = 0x0;
    c_ld_en_adr_bsel_out = 0x0;

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
    c_csr_mret_out = 0x0;
    c_csr_bus_wdata_out = 0x0;
    c_trap_handler_enter_out = 0x0;

    next_state = state.read();

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
            //  Wait for membrana to acknowledge
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

        case STATE_LR1:
            // Await dport ack = 0 -> bus is ready for a new transaction

            if(s_dport_ack_in.read() == 0)
            {
                next_state = STATE_LR2;
            }
            else
            {
                // If ack != 0, remain
                next_state = STATE_LR1;
            }
            break;

        case STATE_LR2:
            // Now the bus is ready for a new transaction

            // Set dport strobe to begin transaction
            c_dport_stb_out = 1;
            c_dport_lrsc_out = 1;

            c_rs1_adr_bsel_out = 1;
            c_ld_en_adr_bsel_out = 1;

            next_state = STATE_LR3;
            break;

        case STATE_LR3:
            // Wait for dport ack

            // Reservation set is set in a few cycles after strobe.
            c_dport_lrsc_out = 1;
            // save to rdata until data is vaild then go to next state
            c_ld_en_rdata_out = 1;

            // Await dport ack = 1
            if(s_dport_ack_in.read() == 1)
            {
                // from rdata register put in rd
                next_state = STATE_A_STORE_RDATA;
            }
            else
            {
                // If ack != 1, remain
                next_state = STATE_LR3;
            }
            break;

        case STATE_AMO_LOAD1:
            // Await dport ack = 0 -> bus is ready for a new transaction
            if(s_dport_ack_in.read() == 0)
            {
                next_state = STATE_AMO_LOAD2;
            }
            else
            {
                // If ack != 0, remain
                next_state = STATE_AMO_LOAD1;
            }
            break;

        case STATE_AMO_LOAD2:
            // Now the bus is ready for a new transaction

            // Set dport strobe to begin transaction
            c_dport_stb_out = 1;
            // mark amo_pending
            c_dport_amo_out = 1;

            c_rs1_adr_bsel_out = 1;
            c_ld_en_adr_bsel_out = 1;

            next_state = STATE_AMO_LOAD3;
            break;

        case STATE_AMO_LOAD3:
            // Wait for dport ack
            // save to rdata until data is vaild then go to next state
            c_ld_en_rdata_out = 1;

            // Await dport ack = 1
            if(s_dport_ack_in.read() == 1)
            {
                // Saved in reg_rdata so go to amo decode
                next_state = STATE_AMO_OP;
            }
            else
            {
                // If ack != 1, remain
                next_state = STATE_AMO_LOAD3;
            }
            break;

        case STATE_AMO_OP:
            // use ALU-operation decoded in AMO funct5 on *(rs1) and rs2
            c_alu_rdata_reg_out = 1;
            c_force_amo_out = 1;

            c_alu_out_to_wdata_out = 1;
            c_ld_en_wdata_out = 1;

            next_state = STATE_AMO_STORE1;

            break;

        case STATE_AMO_STORE1:
            //  Wait for dport ack = 0 -> bus is ready for a new transaction
            if(s_dport_ack_in.read() == 0)
            {
                next_state = STATE_AMO_STORE2;
            }
            else
            {
                // If ack != 0, remain
                next_state = STATE_AMO_STORE1;
            }
            break;

        case STATE_AMO_STORE2:
            // Bus is ready for a new transaction

            // Set dport strobe to begin transaction
            c_dport_stb_out = 1;
            // Set dport write enable
            c_dport_we_out = 1;
            // unmark amo_pending
            c_dport_amo_out = 1;

            next_state = STATE_AMO_STORE3;

            break;

        case STATE_AMO_STORE3:

            // Read by membrana after a few cycles
            c_dport_we_out = 1;
            c_dport_amo_out = 1;

            // Await dport ack = 1 -> the transaction is complete and data has been stored
            if(s_dport_ack_in.read() == 1)
            {
                next_state = STATE_A_STORE_RDATA;
            }
            else
            {
                // If ack != 1, remain
                next_state = STATE_AMO_STORE3;
            }

            break;

        case STATE_A_STORE_RDATA: // (basically a copy of STATE_LOAD4 but for clarity seperate)
            // Write old value or status to rd.

            // increase PC for every A-Ext instruction
            c_pc_inc4_out = 1;

            c_reg_ldmem_out = 1; // set regfile input to memory
            c_reg_ld_en_out = 1; // regfile load enable

            instruction_finished();
            break;

        case STATE_SC1:
            //  Wait for dport ack = 0 -> bus is ready for a new transaction
            if(s_dport_ack_in.read() == 0)
            {
                next_state = STATE_SC2;
            }
            else
            {
                // If ack != 0, remain
                next_state = STATE_SC1;
            }
            break;

        case STATE_SC2:
            // Bus is ready for a new transaction

            // Set dport strobe to begin transaction
            c_dport_stb_out = 1;
            c_dport_we_out = 1;
            c_dport_lrsc_out = 1;

            // adr = rs1 + 0
            c_rs1_adr_bsel_out = 1;
            c_ld_en_adr_bsel_out = 1;

            // rs2 in wdata
            c_ld_en_wdata_out = 1;

            next_state = STATE_SC3;

            break;

        case STATE_SC3:

            // Reservation set is cleared in a few cycles after strobe.
            c_dport_lrsc_out = 1;

            // Enable rdata register to load the status value from rdata
            c_ld_en_rdata_out = 1;

            // Await dport ack = 1 -> the transaction is complete and data has been stored
            if(s_dport_ack_in.read() == 1)
            {
                // Membrana writes the status of the store operation to rdata
                // Controller loads it into `rd`
                next_state = STATE_A_STORE_RDATA;
            }
            else
            {
                // If ack != 1, remain
                next_state = STATE_SC3;
            }

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
            c_ld_en_adr_bsel_out = 1;
            // advance PC
            c_pc_inc4_out = 1;

            next_state = STATE_LOAD3;
            break;

        case STATE_LOAD3:
            // Wait for dport ack

            // save to rdata until data is vaild then go to next state
            c_ld_en_rdata_out = 1;

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
            // ack was received ->  valid data was saved to reg_rdata and it can be stored in the register file

            // Set control signals
            c_reg_ldmem_out = 1; // set regfile input to memory
            c_reg_ld_en_out = 1; // regfile load enable

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
            c_ld_en_adr_bsel_out = 1;
            c_ld_en_wdata_out = 1;
            // advance PC
            c_pc_inc4_out = 1;

            next_state = STATE_STORE3;

            break;

        case STATE_STORE3:
            // Keep address stable for datahandler.
            // Datahandler does not get the bsel from register but immediate to
            // ensure valid data from strobe until ready.
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
                c_csr_imm_out = rs1(s_instruction_in.read());             // Immediate value is at the same location as reg source
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
                c_csr_imm_out = rs1(s_instruction_in.read());           // Immediate value is at the same location as reg source
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

            /* Interrupt */

        case STATE_TRAP_ENTRY:
            c_csr_interrupt_out = 1;
            // PC in MEPC (0x341) speichern
            c_csr_bus_adr_out = 0x341;
            c_csr_bus_write_en_out = 1;
            c_csr_write_mode_out = e_csr_write_mode::CSR_WRITE_ALL;

            next_state = STATE_TRAP_SAVE_MCAUSE;
            break;

        case STATE_TRAP_SAVE_MCAUSE:
            c_csr_interrupt_out = 1;
            {
                // mcause Register schreiben - mit Interrupt-Flag (Bit 31)
                c_csr_bus_adr_out = 0x342;
                c_csr_bus_write_en_out = 1;
                c_csr_write_mode_out = e_csr_write_mode::CSR_WRITE_ALL;

                // Determine which interrupt type is pending based on CSR register values
                // Check in priority order: External > Timer > Software
                sc_uint<32> interrupt_type;

                // Check if Machine External Interrupt is pending
                if(s_mip_meip_in.read() && s_mie_meie_in.read())
                {
                    interrupt_type = INTERRUPT_TYPE_MACHINE_EXTERNAL; // MEIP (Machine External Interrupt)
                }
                else if(s_mip_mtip_in.read() && s_mie_mtie_in.read())
                {
                    interrupt_type = INTERRUPT_TYPE_MACHINE_TIMER; // MTIP (Machine Timer Interrupt)
                }
                else if(s_mip_msip_in.read() && s_mie_msie_in.read())
                {
                    interrupt_type = INTERRUPT_TYPE_MACHINE_SOFTWARE; // MSIP (Machine Software Interrupt)
                }
                else
                {
                    // Default to machine software interrupt if somehow none are pending
                    // This shouldn't happen as we check interrupt_pending before entering the trap
                    interrupt_type = INTERRUPT_TYPE_MACHINE_SOFTWARE;
                }

                // Setze das Interrupt-Flag (Bit 31) und die Ursache
                sc_uint<32> mcause_value = 0x80000000 | interrupt_type;
                c_csr_bus_wdata_out = mcause_value;

                next_state = STATE_TRAP_UPDATE_MSTATUS;
                break;
            }

        case STATE_TRAP_UPDATE_MSTATUS:
            c_csr_interrupt_out = 1;
            // mstatus Register lesen und MIE Bit deaktivieren
            c_csr_bus_adr_out = 0x300;
            c_csr_bus_read_en_out = 1;

            next_state = STATE_TRAP_UPDATE_MSTATUS2;
            break;

        case STATE_TRAP_UPDATE_MSTATUS2:
            c_csr_interrupt_out = 1;
            // mstatus Register aktualisieren - MIE Bit deaktivieren
            c_csr_bus_adr_out = 0x300;
            c_csr_bus_write_en_out = 1;
            c_csr_write_mode_out = e_csr_write_mode::CSR_WRITE_CLEAR;
            c_csr_imm_en_out = 1;
            c_csr_imm_out = 0x8; // Bit 3 = MIE

            next_state = STATE_TRAP_LOAD_HANDLER;
            break;

        case STATE_TRAP_LOAD_HANDLER:
            c_csr_interrupt_out = 1;
            // PC aus MTVEC (0x305) lesen - NUR LESEN, NICHT PC LADEN
            c_csr_bus_adr_out = 0x305;
            c_csr_bus_read_en_out = 1;

            // Wait for result in next cycle
            next_state = STATE_TRAP_SAVE_PC;
            break;

        case STATE_TRAP_SAVE_PC:
            c_csr_interrupt_out = 1;
            // Signal trap handler entry to PC
            c_trap_handler_enter_out = 1;

            // Use CSR as data source for PC
            c_csr_bus_adr_out = 0x305;
            c_csr_bus_read_en_out = 1;

            // ALU-Signale auf 0 setzen
            c_force_add_out = 0;
            c_alu_imm_out = 0;
            c_alu_pc_out = 0;

            next_state = STATE_IPORT_STB;
            break;

        case STATE_MRET:
            // Return from Machine Exception - restore PC from MEPC and MPIE to MIE

            // Signal CSR to restore MPIE to MIE and set MPIE to 1
            c_csr_mret_out = 1;
            c_csr_interrupt_out = 0; // Explicitly clear interrupt signal on MRET

            // CRITICAL: Explicitly reset all memory access signals to prevent glitches
            c_dport_stb_out = 0;
            c_dport_we_out = 0;
            c_iport_stb_out = 0;
            c_alu_imm_out = 0;
            c_alu_pc_out = 0;
            c_force_add_out = 0;

            // Force PC to load from MEPC with no ALU involvement
            c_pc_ld_en_out = 0; // Don't load from ALU
            c_pc_inc4_out = 0;  // Don't increment

            instruction_finished();
            next_state = STATE_IPORT_STB;
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
        case OP_FENCE: // or fence.i NOP as no multicore + no out of order memory
            next_state = STATE_NOP;
            break;
        case OP_A_EXT:
            handle_a_ext_decode();
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
    else if(s_instruction_in.read() == SYSTEM_INSTR_MRET)
    {
        next_state = STATE_MRET;
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

void m_controller::handle_a_ext_decode()
{
    switch(funct5(s_instruction_in.read()))
    {
        case FUNCT5A_SC:
            next_state = STATE_SC1;
            break;

        case FUNCT5A_LR:
            next_state = STATE_LR1;
            break;

        default:
            next_state = STATE_AMO_LOAD1;
            break;
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
std::string m_controller::state_to_string(sc_uint<32> state) const
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
        // A-Ext
        "STATE_LR1",
        "STATE_LR2",
        "STATE_LR3",
        "STATE_AMO_LOAD1",
        "STATE_AMO_LOAD2",
        "STATE_AMO_LOAD3",
        "STATE_AMO_OP"
        "STATE_AMO_STORE1",
        "STATE_AMO_STORE2",
        "STATE_AMO_STORE3",
        "STATE_A_STORE_RDATA",
        "STATE_SC1",
        "STATE_SC2",
        "STATE_SC3",
        // System
        "STATE_EBREAK",
        "STATE_ECALL",
        "STATE_DRET",
        // Trap/Exception handling
        "STATE_TRAP_ENTRY",
        "STATE_TRAP_SAVE_PC",
        "STATE_MRET",
        "STATE_TRAP_SAVE_MCAUSE",
        "STATE_TRAP_UPDATE_MSTATUS",
        "STATE_TRAP_UPDATE_MSTATUS2",
        "STATE_TRAP_LOAD_HANDLER",
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
    // Check for pending interrupts
    else if(s_interrupt_pending_in.read() == 0x1 &&
            state.read() != STATE_MRET &&
            state.read() != STATE_TRAP_SAVE_PC &&
            state.read() != STATE_TRAP_LOAD_HANDLER)
    {
        next_state = STATE_TRAP_ENTRY;
    }
    // Execute standard fetch
    else
    {
        next_state = STATE_IPORT_STB;
    }
}
