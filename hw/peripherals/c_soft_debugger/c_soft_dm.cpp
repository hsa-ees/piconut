/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Johannes Hofmann <Johannes.Hofmann1@tha.de>
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

// Note: progbuf only gets executed with at least 2 abstract_command register!

#include "c_soft_dm.h"

#include "reg_cast.h"
#include "debug_handler.h"

#include <base.h>

#define HART_COUNT 1

namespace {
static constexpr size_t module_size = 148; // Size in bytes, exluding debug handler.
static constexpr size_t data_reg_count = 2;
static constexpr size_t progbug_reg_count = 0; // Not supported yet. Else 3;
static constexpr size_t abstract_commands_count = 8;
} // namespace

namespace {
std::vector<uint32_t> make_default_abstract_command_regs(const size_t size)
{
    std::vector<uint32_t> abstract_command_regs{(uint)size};
    return abstract_command_regs;
}

} // namespace

c_soft_dm::c_soft_dm(
    std::function<void(bool)> callback_signal_debug_haltrequest)
    : base_address{0}
    , size{module_size + debug_handler::binary_size}
    , debug_regs{
          this,
          HART_COUNT,
          data_reg_count,
          progbug_reg_count,
          callback_signal_debug_haltrequest}
    , abstract_command_regs_size{abstract_commands_count}
    , abstract_command_regs{//
          make_default_abstract_command_regs(abstract_command_regs_size)}
    , hartcontrol_reg{0}
    , hartstatus_reg{0}
{
}

const char* c_soft_dm::get_info()
{
    static char info[128];
    snprintf(
        info,
        sizeof(info),
        "Name: %s,\nBase Address: 0x%X ,\nSize: %d B\n",
        name,
        base_address,
        (uint32_t)size);
    return info;
}

bool c_soft_dm::is_addressed(uint64_t adr)
{
    return adr >= base_address && adr < size + base_address;
}

uint32_t c_soft_dm::read32(uint64_t adr)
{
    if(!is_addressed(adr))
    {
        return 0;
    }

    uint32_t internal_address = adr - this->base_address;

    // Read only from start of register
    internal_address &= ~0b11;

    if(_is_debug_handler_addressed(internal_address))
    {
        return _debug_handler_instruction(internal_address);
    }

    e_regs selected_register = static_cast<e_regs>(internal_address);
    switch(selected_register)
    {
        case e_regs::DATA0:
            return reg_to_int(
                debug_regs.data[static_cast<size_t>(c_debug_regs::e_data_reg_index::DATA0)]);
        case e_regs::DATA1:
            return reg_to_int(
                debug_regs.data[static_cast<size_t>(c_debug_regs::e_data_reg_index::DATA1)]);
        case e_regs::PROGRAM_BUFFER0:
            return reg_to_int(
                debug_regs.read_reg_progbuf(
                    c_debug_regs::e_progbuf_reg_index::PROGBUF0));
        case e_regs::PROGRAM_BUFFER1:
            return reg_to_int(
                debug_regs.read_reg_progbuf(
                    c_debug_regs::e_progbuf_reg_index::PROGBUF1));
        case e_regs::PROGRAM_BUFFER2:
            return reg_to_int(
                debug_regs.read_reg_progbuf(
                    c_debug_regs::e_progbuf_reg_index::PROGBUF2));
        case e_regs::ABSTRACT_COMMAND0:
            return abstract_command_regs[0];
        case e_regs::ABSTRACT_COMMAND1:
            return abstract_command_regs[1];
        case e_regs::ABSTRACT_COMMAND2:
            return abstract_command_regs[2];
        case e_regs::ABSTRACT_COMMAND3:
            return abstract_command_regs[3];
        case e_regs::ABSTRACT_COMMAND4:
            return abstract_command_regs[4];
        case e_regs::ABSTRACT_COMMAND5:
            return abstract_command_regs[5];
        case e_regs::ABSTRACT_COMMAND6:
            return abstract_command_regs[6];
        case e_regs::ABSTRACT_COMMAND7:
            return abstract_command_regs[7];
        case e_regs::HARTCONTROL:
            return reg_to_int(
                _read_reg_hartcontrol());
        case e_regs::HARTSTATUS:
            // Entire register is write-only.
            return 0;
        case e_regs::DEBUG_HANDLER_START:
            PN_ASSERT(false);
            return 0;
    }

    PN_ERRORF(("Address is not mapped to a register. Address: 0x%x", adr));
    return 0;
}

void c_soft_dm::write32(uint64_t adr, uint32_t data)
{
    if(!is_addressed(adr))
    {
        return;
    }

    uint32_t internal_address = adr - this->base_address;

    // Write only from start of register
    internal_address &= ~0b11;

    if(_is_debug_handler_addressed(internal_address))
    {
        // Debug handler is read-only (obviously look at the name)
        return;
    }

    e_regs selected_register = static_cast<e_regs>(internal_address);
    switch(selected_register)
    {
        case e_regs::DATA0:
            debug_regs.data[static_cast<size_t>(c_debug_regs::e_data_reg_index::DATA0)] =
                int_to_reg<c_debug_regs::data_t>(data);
            return;
        case e_regs::DATA1:
            debug_regs.data[static_cast<size_t>(c_debug_regs::e_data_reg_index::DATA1)] =
                int_to_reg<c_debug_regs::data_t>(data);
            return;
        case e_regs::PROGRAM_BUFFER0:
        case e_regs::PROGRAM_BUFFER1:
        case e_regs::PROGRAM_BUFFER2:
        case e_regs::ABSTRACT_COMMAND0:
        case e_regs::ABSTRACT_COMMAND1:
        case e_regs::ABSTRACT_COMMAND2:
        case e_regs::ABSTRACT_COMMAND3:
        case e_regs::ABSTRACT_COMMAND4:
        case e_regs::ABSTRACT_COMMAND5:
        case e_regs::ABSTRACT_COMMAND6:
        case e_regs::ABSTRACT_COMMAND7:
        case e_regs::HARTCONTROL:
            // Entire register is read-only
            return;
        case e_regs::HARTSTATUS:
            _write_reg_hartstatus(int_to_reg<hartstatus_t>(data));
            return;
        case e_regs::DEBUG_HANDLER_START:
            PN_ASSERT(false);
            return;
    }

    PN_ERRORF(("Address is not mapped to a register. Address: 0x%x", adr));
}

void c_soft_dm::set_signal_debug_haltrequest_ack(bool value)
{
    if(value == 1)
    {
        hartstatus_t hartstatus_reg = {0};
        hartstatus_reg.halted = 1;

        _write_reg_hartstatus(hartstatus_reg);
    }
}

void c_soft_dm::dmi_write(
    uint8_t address,
    uint32_t data)
{
    uint8_t internal_address = address - base_address;
    debug_regs.write_reg(internal_address, data);
}

uint32_t c_soft_dm::dmi_read(
    uint8_t address)
{
    uint8_t internal_address = address - base_address;
    return debug_regs.read_reg(internal_address);
}

void c_soft_dm::dmi_reset()
{
    PN_INFO("c_soft_dm: Dmi reset.");
}

uint32_t c_soft_dm::_absolute_reg_address(e_regs reg) const
{
    const uint32_t internal_address = static_cast<const uint32_t>(reg);
    return internal_address + base_address;
}

void c_soft_dm::_set_abstract_commands(
    std::vector<uint32_t> commands)
{
    if(hartstatus_reg.halted == 0)
    {
        PN_WARNINGF(("Recieved custom commands while hart was not in halted state."));

        debug_regs.abstractcs.set_cmderr(
            c_debug_reg_abstractcs::e_cmderr::HALT_RESUME);
        return;
    }

    if(hartstatus_reg.commands_running == 1)
    {
        PN_WARNINGF(("Commands already executing."));

        debug_regs.abstractcs.set_cmderr(
            c_debug_reg_abstractcs::e_cmderr::BUSY);
        return;
    }

    if(hartcontrol_reg.run_commandsreq == 1)
    {
        PN_WARNINGF(("Commands already in pipeline."));

        debug_regs.abstractcs.set_cmderr(
            c_debug_reg_abstractcs::e_cmderr::BUSY);
        return;
    }

    if(abstract_command_regs_size < commands.size())
    {
        PN_ASSERT(false);
        return;
    }

    if(commands.empty())
    {
        PN_WARNINGF(("Empty commands recieved ignoring..."));
        return;
    }

    abstract_command_regs = std::move(commands);

    hartcontrol_reg.run_commandsreq = 1;
    debug_regs.abstractcs.set_busy(
        c_debug_reg_abstractcs::e_busy::SET);
}

c_soft_dm::hartcontrol_t c_soft_dm::_read_reg_hartcontrol()
{
    // Workaround because resumereq is not reset quick enough when
    // single stepping. This fix ensures that resumereq is only applied once.
    hartcontrol_t ret = hartcontrol_reg;

    if(hartcontrol_reg.resumereq == 1)
    {
        hartcontrol_reg.resumereq = 0;
    }

    return ret;
}

void c_soft_dm::_write_reg_hartstatus(hartstatus_t hartstatus)
{
    hartstatus_reg = hartstatus;

    // Reflect some bits to debug_reg so the debugger can access them
    debug_regs.dmstatus.allhalted = hartstatus_reg.halted;
    debug_regs.dmstatus.anyhalted = hartstatus_reg.halted;

    debug_regs.dmstatus.allrunning = hartstatus_reg.running;
    debug_regs.dmstatus.anyrunning = hartstatus_reg.running;

    if(hartstatus_reg.running == 1 && hartstatus_reg.halted == 0)
    {
        debug_regs.dmstatus.allresumeack = 1;
        debug_regs.dmstatus.anyresumeack = 1;
    }

    if(hartstatus_reg.commands_running == 1)
    {
        hartcontrol_reg.run_commandsreq = 0;
    }

    static bool last_commands_running = false;
    if(last_commands_running &&
        !hartstatus_reg.commands_running)
    {
        // commands finished
        debug_regs.abstractcs.set_busy(
            c_debug_reg_abstractcs::e_busy::RESET);
    }

    last_commands_running = hartstatus_reg.commands_running;
}

bool c_soft_dm::_is_debug_handler_addressed(uint32_t internal_address) const
{
    const uint32_t debug_rom_start_address =
        static_cast<uint32_t>(e_regs::DEBUG_HANDLER_START);
    const uint32_t debug_rom_end_address =
        debug_rom_start_address + debug_handler::binary_size;

    return internal_address >= debug_rom_start_address &&
           internal_address < debug_rom_end_address;
}

uint32_t c_soft_dm::_debug_handler_instruction(uint32_t internal_address) const
{
    const uint32_t register_byte_width = 4;
    const uint32_t debug_rom_start_address =
        static_cast<uint32_t>(e_regs::DEBUG_HANDLER_START);

    const int debug_rom_index =
        (internal_address - debug_rom_start_address) /
        register_byte_width;

    if(debug_rom_index < 0 ||
        debug_rom_index >= debug_handler::binary_wsize)
    {
        PN_ASSERT(false);
        return 0;
    }

    return debug_handler::binary[debug_rom_index];
}
