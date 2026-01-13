/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: This file contains the implementation of the c_soft_remote_bitbang simulation ONLY

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

#include "c_debug_reg_command.h"

#include "c_debug_reg_abstractcs.h"
#include "c_debug_regs.h"
#include "dm_soft.h"
#include "reg_cast.h"

#include <piconut.h>

namespace abstract_commands {
// nop (addi x0, x0, 0)
static constexpr uint32_t nop = 0x13;
// ebreak
static constexpr uint32_t ebreak = 0x00100073;
} // namespace abstract_commands

namespace opcode {
// load word
static constexpr uint32_t lw = 0b0000011;
// store word
static constexpr uint32_t sw = 0b0100011;
// add integer
static constexpr uint32_t addi = 0b0010011;
// load upper immediate
static constexpr uint32_t lui = 0b0110111;
// jump and link
static constexpr uint32_t jal = 0b1101111;
// csr
static constexpr uint32_t csr = 0b1110011;
} // namespace opcode

namespace funct3 {
// load word
static constexpr uint32_t lw = 0b010;
// store word
static constexpr uint32_t sw = 0b010;
// add integer
static constexpr uint32_t addi = 0b000;
// csr read-set
static constexpr uint32_t csrrs = 0b010;
// csr read-write
static constexpr uint32_t csrrw = 0b001;
} // namespace funct3

namespace {

uint32_t make_instruction_immediate(
    uint32_t imm,
    uint32_t rs1,
    uint32_t funct3,
    uint32_t rd,
    uint32_t opcode)
{
    static constexpr int offset_imm = 20;
    static constexpr int offset_rs1 = 15;
    static constexpr int offset_funct3 = 12;
    static constexpr int offset_rd = 7;
    static constexpr int offset_opcode = 0;

    uint32_t command = 0;
    command |= (imm & 0b111111111111) << offset_imm;
    command |= (rs1 & 0b11111) << offset_rs1;
    command |= (funct3 & 0b111) << offset_funct3;
    command |= (rd & 0b11111) << offset_rd;
    command |= opcode & 0b1111111 << offset_opcode;
    return command;
}

uint32_t make_instruction_upper_immediate(
    uint32_t imm,
    uint32_t rd,
    uint32_t opcode)
{
    static constexpr int offset_imm = 12;
    static constexpr int offset_rd = 7;
    static constexpr int offset_opcode = 0;

    uint32_t command = 0;
    command |= (imm & 0b11111111111111111111) << offset_imm;
    command |= (rd & 0b11111) << offset_rd;
    command |= opcode & 0b1111111 << offset_opcode;
    return command;
}

uint32_t make_instruction_store(
    uint32_t rs2,
    uint32_t rs1,
    uint32_t funct3,
    uint32_t imm,
    uint32_t opcode)
{
    static constexpr int offset_imm_high = 25;
    static constexpr int offset_rs2 = 20;
    static constexpr int offset_rs1 = 15;
    static constexpr int offset_funct3 = 12;
    static constexpr int offset_imm_low = 7;
    static constexpr int offset_opcode = 0;

    const uint32_t imm_low = imm & 0b11111;
    const uint32_t imm_high = (imm >> 5) & 0b1111111;

    uint32_t command = 0;
    command |= (imm_high & 0b1111111) << offset_imm_high;
    command |= (rs2 & 0b11111) << offset_rs2;
    command |= (rs1 & 0b11111) << offset_rs1;
    command |= (funct3 & 0b111) << offset_funct3;
    command |= (imm_low & 0b11111) << offset_imm_low;
    command |= opcode & 0b1111111 << offset_opcode;
    return command;
}

uint32_t make_instruction_jump(
    uint32_t imm,
    uint32_t rd,
    uint32_t opcode)
{
    static constexpr int offset_imm_20 = 31;
    static constexpr int offset_imm_10_1 = 21;
    static constexpr int offset_imm_11 = 20;
    static constexpr int offset_imm_19_12 = 12;
    static constexpr int offset_rd = 7;
    static constexpr int offset_opcode = 0;

    const uint32_t imm_20 = (imm >> 20) & 0b1;
    const uint32_t imm_10_1 = (imm >> 1) & 0b1111111111;
    const uint32_t imm_11 = (imm >> 11) & 0b1;
    const uint32_t imm_19_12 = (imm >> 12) & 0b11111111;

    uint32_t command = 0;
    command |= (imm_20 & 0b1) << offset_imm_20;
    command |= (imm_10_1 & 0b1111111111) << offset_imm_10_1;
    command |= (imm_11 & 0b1) << offset_imm_11;
    command |= (imm_19_12 & 0b11111111) << offset_imm_19_12;
    command |= (rd & 0b1111111) << offset_rd;
    command |= opcode & 0b1111111 << offset_opcode;
    return command;
}

} // namespace

c_debug_reg_command::c_debug_reg_command(
    c_soft_dm* dm,
    c_debug_reg_abstractcs* abstractcs)
    : dm{dm}
    , abstractcs{abstractcs}
    , reg{0}
{
    PN_ASSERT(dm != nullptr);
    PN_ASSERT(abstractcs != nullptr);
}

c_debug_reg_command::~c_debug_reg_command()
{
}

void c_debug_reg_command::write(
    uint32_t data)
{
    reg = int_to_reg<reg_t>(data);
    _handle_new_command();
}

uint32_t c_debug_reg_command::read()
{
    return reg_to_int(reg);
}

void c_debug_reg_command::set_control(uint32_t control)
{
    reg.control = control;
}

void c_debug_reg_command::set_cmdtype(e_cmdtype cmdtype)
{
    reg.cmdtype = cmdtype;
}

uint32_t c_debug_reg_command::control()
{
    return reg.control;
}

c_debug_reg_command::e_cmdtype c_debug_reg_command::cmdtype()
{
    return reg.cmdtype;
}

void c_debug_reg_command::_handle_new_command()
{
    const e_cmdtype command_type = reg.cmdtype;

    switch(command_type)
    {
        case e_cmdtype::REG_ACCESS:
            _handle_reg_access_command();
            return;
        case e_cmdtype::QUICK_ACCESS:
            _handle_quick_access_command();
            return;
        case e_cmdtype::MEMORY_ACCESS:
            _handle_memory_access_command();
            return;
    }

    PN_WARNING("c_debug_reg_command: Unsupported command.");

    abstractcs->set_cmderr(c_debug_reg_abstractcs::e_cmderr::NOT_SUPPORTED);
}

void c_debug_reg_command::_handle_reg_access_command()
{
    const std::vector<uint32_t> abstract_commands =
        std::move(_make_abstract_command_reg_access());

    if(abstract_commands.empty())
    {
        return;
    }

    dm->_set_abstract_commands(abstract_commands);
}

void c_debug_reg_command::_handle_quick_access_command()
{
    // Not implemented.
    abstractcs->set_cmderr(c_debug_reg_abstractcs::e_cmderr::NOT_SUPPORTED);
}

void c_debug_reg_command::_handle_memory_access_command()
{
    const std::vector<uint32_t> abstract_commands =
        std::move(_make_abstract_command_memory_access());

    if(abstract_commands.empty())
    {
        return;
    }

    dm->_set_abstract_commands(abstract_commands);
}

std::vector<uint32_t> c_debug_reg_command::_make_abstract_command_reg_access() const
{
    reg_command_reg_access_t reg_command_reg_access =
        reg_to_reg<reg_t, reg_command_reg_access_t>(reg);

    PN_ASSERT(e_cmdtype::REG_ACCESS == reg_command_reg_access.cmdtype);

    if(e_cmd_reg_access_aarsize::LOWEST_32 != reg_command_reg_access.aarsize)
    {
        abstractcs->set_cmderr(c_debug_reg_abstractcs::e_cmderr::NOT_SUPPORTED);
        return {};
    }

    auto add_abstract_commands_ending =
        [&reg_command_reg_access](
            std::vector<uint32_t> abstract_commands) -> std::vector<uint32_t> {
        if(reg_command_reg_access.postexec == 1)
        {
            const uint32_t offset_to_progbuf0_default =
                static_cast<uint32_t>(c_soft_dm::e_regs::PROGRAM_BUFFER0) -
                static_cast<uint32_t>(c_soft_dm::e_regs::ABSTRACT_COMMAND0);

            // +1 because we need to consider the jump instruction
            const uint32_t offset_to_progbuf0 =
                offset_to_progbuf0_default + abstract_commands.size() + 1;

            const uint32_t gpr_x1 = 1;
            abstract_commands.emplace_back(
                make_instruction_jump(
                    offset_to_progbuf0, gpr_x1, opcode::jal));
            // Note: Don't forget ebreak in last progbuf!
            return abstract_commands;
        }

        abstract_commands.emplace_back(
            abstract_commands::ebreak);
        return abstract_commands;
    };

    if(!reg_command_reg_access.transfer)
    {
        return add_abstract_commands_ending(
            {abstract_commands::nop});
    }

    static constexpr size_t abstract_registers_upper_bound_csr = 0x0fff;
    static constexpr size_t abstract_registers_upper_bound_gpr = 0x101f;
    static constexpr size_t abstract_registers_upper_bound_floating_point = 0x103f;
    static constexpr size_t abstract_registers_upper_bound_reserved = 0xffff;

    if(reg_command_reg_access.regno <= abstract_registers_upper_bound_csr)
    {
        return add_abstract_commands_ending(
            _make_abstract_command_reg_access_csr(reg_command_reg_access));
    }

    if(reg_command_reg_access.regno <= abstract_registers_upper_bound_gpr)
    {
        return add_abstract_commands_ending(
            _make_abstract_command_reg_access_gpr(reg_command_reg_access));
    }

    if(reg_command_reg_access.regno <= abstract_registers_upper_bound_floating_point)
    {
        // Not implemented
        abstractcs->set_cmderr(c_debug_reg_abstractcs::e_cmderr::NOT_SUPPORTED);
        return {};
    }

    if(reg_command_reg_access.regno <= abstract_registers_upper_bound_reserved)
    {
        // Not implemented
        abstractcs->set_cmderr(c_debug_reg_abstractcs::e_cmderr::NOT_SUPPORTED);
        return {};
    }

    PN_ASSERT(false);
}

std::vector<uint32_t> c_debug_reg_command::_make_abstract_command_reg_access_csr(
    const reg_command_reg_access_t& reg_command_reg_access) const
{
    uint32_t csr_regno = reg_command_reg_access.regno;
    const uint32_t data0_addr = dm->_absolute_reg_address(c_soft_dm::e_regs::DATA0);
    const uint32_t regno_t6 = 31;

    // TODO: Add filter to block access to unimplemented csrs.

    if(reg_command_reg_access.write == 1)
    {
        return {
            // lw t6, data0(zero)
            make_instruction_immediate(
                data0_addr, 0, funct3::lw, regno_t6, opcode::lw),
            // csrrw zero, regno, t6
            make_instruction_immediate(
                csr_regno, regno_t6, funct3::csrrw, 0, opcode::csr)};
    }

    return {
        // csrrs t6, regno, zero
        make_instruction_immediate(
            csr_regno, 0, funct3::csrrs, regno_t6, opcode::csr),
        // sw t6, data0(0)
        make_instruction_store(
            regno_t6, 0, funct3::sw, data0_addr, opcode::sw)};
}

std::vector<uint32_t> c_debug_reg_command::_make_abstract_command_reg_access_gpr(
    const reg_command_reg_access_t& reg_command_reg_access) const
{
    const uint32_t regno = reg_command_reg_access.regno - 0x1000;
    const uint32_t data0_addr = dm->_absolute_reg_address(c_soft_dm::e_regs::DATA0);

    if(reg_command_reg_access.write == 1)
    {
        return {
            // lw regno, data0(zero)
            make_instruction_immediate(
                data0_addr, 0, funct3::lw, regno, opcode::lw)};
    }

    return {
        // sw regno, data0(zero)
        make_instruction_store(
            regno, 0, funct3::sw, data0_addr, opcode::sw)};
}

std::vector<uint32_t> c_debug_reg_command::_make_abstract_command_memory_access() const
{
    reg_command_memory_access_t reg_command_memory_access =
        reg_to_reg<reg_t, reg_command_memory_access_t>(reg);

    PN_ASSERT(e_cmdtype::MEMORY_ACCESS == reg_command_memory_access.cmdtype);

    if(reg_command_memory_access.aamsize == e_cmd_memory_access_aamsize::BITS_64 ||
        reg_command_memory_access.aamsize == e_cmd_memory_access_aamsize::BITS_128)
    {
        // 64 and 128 bits access width is not supported yet.
        abstractcs->set_cmderr(c_debug_reg_abstractcs::e_cmderr::NOT_SUPPORTED);
        return {};
    }

    if(reg_command_memory_access.aamvirtual == e_cmd_memory_access_aamvirtual::VIRTUAL)
    {
        // Virtual addresses not supported yet.
        abstractcs->set_cmderr(c_debug_reg_abstractcs::e_cmderr::NOT_SUPPORTED);
        return {};
    }

    const uint32_t data0_addr = dm->_absolute_reg_address(c_soft_dm::e_regs::DATA0);
    const uint32_t data1_addr = dm->_absolute_reg_address(c_soft_dm::e_regs::DATA1);
    const uint32_t regno_t5 = 30;
    const uint32_t regno_t6 = 31;

    // TODO: Currently is is only possible to read/write words.
    // Adapt to consider given width in aamsize.

    // TODO: implement aampostincrement.

    if(reg_command_memory_access.write == 1)
    {
        return {
            // lw t6, data1(zero)
            make_instruction_immediate(
                data1_addr, 0, funct3::lw, regno_t6, opcode::lw),
            // lw t5, data0(zero)
            make_instruction_immediate(
                data0_addr, 0, funct3::lw, regno_t5, opcode::lw),
            // sw t5, 0(t6)
            make_instruction_store(
                regno_t5, regno_t6, funct3::sw, 0, opcode::sw),
            // TODO: reset increment flag here
            abstract_commands::ebreak};
    }

    return {
        // lw t6, data1(zero)
        make_instruction_immediate(
            data1_addr, 0, funct3::lw, regno_t6, opcode::lw),
        // lw t6 0(t6)
        make_instruction_immediate(
            0, regno_t6, funct3::lw, regno_t6, opcode::lw),
        // sw t6, data0(0)
        make_instruction_store(
            regno_t6, 0, funct3::sw, data0_addr, opcode::sw),
        // TODO: reset increment flag here
        abstract_commands::ebreak};
}
