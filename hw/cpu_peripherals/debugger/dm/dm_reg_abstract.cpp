/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the definition of the m_dm_reg_abstract module.

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

#include "dm_reg_abstract.h"

namespace abstract_commands {
// nop (addi x0, x0, 0)
static const sc_uint<32> nop = 0x13;
// ebreak
static const sc_uint<32> ebreak = 0x00100073;
} // namespace abstract_commands

namespace {
enum e_dm_opcode
{
    DM_OPCODE_LW = 0b0000011,   // load word
    DM_OPCODE_SW = 0b0100011,   // store word
    DM_OPCODE_ADDI = 0b0010011, // add integer
    DM_OPCODE_LUI = 0b0110111,  // load upper immediate
    DM_OPCODE_JAL = 0b1101111,  // jump and link
    DM_OPCODE_CSR = 0b1110011,  // csr
};

enum e_dm_funct3
{
    DM_FUNCT3_LW = 0b010,    // load word
    DM_FUNCT3_SW = 0b010,    // store word
    DM_FUNCT3_ADDI = 0b000,  // add integer
    DM_FUNCT3_CSRRS = 0b010, // csr read-set
    DM_FUNCT3_CSRRW = 0b001, // csr read-write
};

sc_uint<32> make_instruction_immediate(
    sc_uint<12> imm,
    sc_uint<5> rs1,
    sc_uint<3> funct3,
    sc_uint<5> rd,
    sc_uint<7> opcode)
{
    return (imm, rs1, funct3, rd, opcode);
}

sc_uint<32> make_instruction_store(
    sc_uint<5> rs2,
    sc_uint<5> rs1,
    sc_uint<3> funct3,
    sc_uint<12> imm,
    sc_uint<7> opcode)
{
    return (imm.range(11, 5), rs2, rs1, funct3, imm.range(4, 0), opcode);
}

} // namespace

void m_dm_reg_abstract::pn_trace(sc_trace_file* tf, int level)
{
    // Ports...
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);

    PN_TRACE_BUS(tf, abstract_regs_o, NUM_ABSTRACT_REGS);
    PN_TRACE(tf, command_reg_i);
    PN_TRACE(tf, c_acmds_generate_i);
    PN_TRACE(tf, s_error_not_supported);

    // Internal
    PN_TRACE_BUS(tf, abstract_regs, NUM_ABSTRACT_REGS);
}

void m_dm_reg_abstract::proc_clk()
{
    while(true)
    {
        // TODO: This is somehow not working but should:
        // According to iscsc systemc supported bullet point 38
        // sc_uint<32> abstract_regs_var[NUM_ABSTRACT_REGS];
        // for(size_t i = 0; i < NUM_ABSTRACT_REGS; ++i)
        // {
        //     abstract_regs_var[i] = abstract_regs[i].read();
        // }

        s_error_not_supported = 0;

        if(reset.read() == 1)
        {
            for(size_t i = 0; i < NUM_ABSTRACT_REGS; ++i)
            {
                abstract_regs[i] = 0;
            }
        }
        else if(c_acmds_generate_i.read() == 1)
        {
            _generate_acmds();
        }

        wait();
    }
}

void m_dm_reg_abstract::proc_cmb_out()
{
    for(size_t i = 0; i < NUM_ABSTRACT_REGS; i++)
    {
        abstract_regs_o[i] = abstract_regs[i].read();
    }
}

void m_dm_reg_abstract::_generate_acmds()
{
    switch(command_reg_i.read().range(COMMAND_CMDTYPE))
    {
        case e_command_cmdtype::COMMAND_CMDTYPE_REG:
            _generate_acmds_cmdtype_reg();
            break;
        case e_command_cmdtype::COMMAND_CMDTYPE_MEMORY:
            _generate_acmds_cmdtype_mem();
            break;
        case e_command_cmdtype::COMMAND_CMDTYPE_QUICK:
        default:
            s_error_not_supported = 1;
            PN_ERROR("Unsupported cmdtype");
            break;
    }
}

void m_dm_reg_abstract::_generate_acmds_cmdtype_reg()
{
    PN_ASSERT(command_reg_i.read().range(COMMAND_CMDTYPE) == e_command_cmdtype::COMMAND_CMDTYPE_REG);

    if(command_reg_i.read().range(COMMAND_REG_AARSIZE) !=
        e_command_reg_aarsize::COMMAND_REG_AARSIZE_32BIT)
    {
        s_error_not_supported = 1;
    }
    else
    {
        if(command_reg_i.read().range(COMMAND_REG_REGNO) <=
            e_command_cmdtype_reg::COMMAND_CMDTYPE_REG_CSR)
        {
            _generate_acmds_cmdtype_reg_csr();
        }
        else if(command_reg_i.read().range(COMMAND_REG_REGNO) <=
                e_command_cmdtype_reg::COMMAND_CMDTYPE_REG_GPR)
        {
            _generate_acmds_cmdtype_reg_gpr();
        }
        else if(command_reg_i.read().range(COMMAND_REG_REGNO) <=
                e_command_cmdtype_reg::COMMAND_CMDTYPE_REG_FLOAT)
        {
            // Unsupported
            s_error_not_supported = 1;
            PN_ERROR("Unsupported cmdtype reg");
        }
    }
}

void m_dm_reg_abstract::_generate_acmds_cmdtype_reg_csr()
{
    sc_uint<16> regno_csr = command_reg_i.read().range(COMMAND_REG_REGNO);
    const sc_uint<5> regno_gpr_t6 = 31;

    // TODO: Add filter to block access to unimplemented csrs.

    if(command_reg_i.read()[COMMAND_REG_WRITE] == 1)
    {
        // lw t6, data0(zero)
        abstract_regs[0] = make_instruction_immediate(
            e_wb_reg_adrs::WB_DATA0,
            0,
            e_dm_funct3::DM_FUNCT3_LW,
            regno_gpr_t6,
            e_dm_opcode::DM_OPCODE_LW);
        // csrrw zero, regno, t6
        abstract_regs[1] = make_instruction_immediate(
            regno_csr,
            regno_gpr_t6,
            e_dm_funct3::DM_FUNCT3_CSRRW,
            0,
            e_dm_opcode::DM_OPCODE_CSR);
    }
    else
    {
        // csrrs t6, regno, zero
        abstract_regs[0] = make_instruction_immediate(
            regno_csr,
            0,
            e_dm_funct3::DM_FUNCT3_CSRRS,
            regno_gpr_t6,
            e_dm_opcode::DM_OPCODE_CSR);
        // sw t6, data0(0)
        abstract_regs[1] = make_instruction_store(
            regno_gpr_t6,
            0,
            e_dm_funct3::DM_FUNCT3_SW,
            e_wb_reg_adrs::WB_DATA0,
            e_dm_opcode::DM_OPCODE_SW);
    }
    abstract_regs[2] = abstract_commands::ebreak;
}

void m_dm_reg_abstract::_generate_acmds_cmdtype_reg_gpr()
{
    sc_uint<16> regno_gpr =
        command_reg_i.read().range(COMMAND_REG_REGNO) - 0x1000;
    if(command_reg_i.read()[COMMAND_REG_WRITE] == 1)
    {
        // lw regno, data0(zero)
        abstract_regs[0] = make_instruction_immediate(
            e_wb_reg_adrs::WB_DATA0,
            0,
            e_dm_funct3::DM_FUNCT3_LW,
            regno_gpr,
            e_dm_funct3::DM_FUNCT3_LW);
    }
    else
    {
        // sw regno, data0(zero)
        abstract_regs[0] = make_instruction_store(
            regno_gpr,
            0,
            e_dm_funct3::DM_FUNCT3_SW,
            e_wb_reg_adrs::WB_DATA0,
            e_dm_opcode::DM_OPCODE_SW);
    }
    abstract_regs[1] = abstract_commands::ebreak;
}

void m_dm_reg_abstract::_generate_acmds_cmdtype_mem()
{
    PN_ASSERT(command_reg_i.read().range(COMMAND_CMDTYPE) == e_command_cmdtype::COMMAND_CMDTYPE_MEMORY);

    if(command_reg_i.read().range(COMMAND_MEM_AAMSIZE) !=
        e_command_mem_aamsize::COMMAND_MEM_AAMSIZE_32BIT)
    {
        s_error_not_supported = 1;
    }
    else
    {
        const sc_uint<5> regno_gpr_t5 = 30;
        const sc_uint<5> regno_gpr_t6 = 31;

        // lw t6, data1(zero)
        abstract_regs[0] = make_instruction_immediate(
            e_wb_reg_adrs::WB_DATA1,
            0,
            e_dm_funct3::DM_FUNCT3_LW,
            regno_gpr_t6,
            e_dm_opcode::DM_OPCODE_LW);

        if(command_reg_i.read()[COMMAND_MEM_WRITE] == 1)
        {
            // lw t5, data0(zero)
            abstract_regs[1] = make_instruction_immediate(
                e_wb_reg_adrs::WB_DATA0,
                0,
                e_dm_funct3::DM_FUNCT3_LW,
                regno_gpr_t5,
                e_dm_opcode::DM_OPCODE_LW);
            // sw t5, 0(t6)
            abstract_regs[2] = make_instruction_store(
                regno_gpr_t5,
                regno_gpr_t6,
                e_dm_funct3::DM_FUNCT3_SW,
                0,
                e_dm_opcode::DM_OPCODE_SW);
        }
        else
        {
            // lw t6 0(t6)
            abstract_regs[1] = make_instruction_immediate(
                0,
                regno_gpr_t6,
                e_dm_funct3::DM_FUNCT3_LW,
                regno_gpr_t6,
                e_dm_opcode::DM_OPCODE_LW);
            // sw t6, data0(0)
            abstract_regs[2] = make_instruction_store(
                regno_gpr_t6,
                0,
                e_dm_funct3::DM_FUNCT3_SW,
                e_wb_reg_adrs::WB_DATA0,
                e_dm_opcode::DM_OPCODE_SW);
        }
        abstract_regs[3] = abstract_commands::ebreak;
    }
}
