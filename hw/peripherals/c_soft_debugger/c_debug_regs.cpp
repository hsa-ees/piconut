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

#include "c_debug_regs.h"

#include "c_soft_dm.h"
#include "reg_cast.h"

#include <base.h>

namespace {

std::vector<c_debug_regs::data_t> make_default_regs_data(
    const size_t size)
{
    std::vector<c_debug_regs::data_t> data;
    data.resize(size);
    return data;
}

std::vector<c_debug_regs::progbuf_t> make_default_regs_progbuf(
    const size_t size)
{
    std::vector<c_debug_regs::progbuf_t> progbuf;
    progbuf.resize(size);
    return progbuf;
}

c_debug_regs::dmstatus_t make_default_reg_dmstatus()
{
    c_debug_regs::dmstatus_t dmstatus = {0};
    dmstatus.version = 2; // DM conforms 0.13 version.
    dmstatus.authenticated = 1;
    dmstatus.impebreak = 0;

    return dmstatus;
}

uint8_t make_hartsellen(size_t hart_count)
{
    return hart_count - 1; // TODO: Why -1?
}

uint32_t get_hartsel(const c_debug_regs::dmcontrol_t& dmcontrol)
{
    return (dmcontrol.hartselhi << 10) | dmcontrol.hartsello;
}

void set_hartsel(c_debug_regs::dmcontrol_t& dmcontrol, uint32_t hartsel)
{
    dmcontrol.hartsello = hartsel & 0b1111111111;
    dmcontrol.hartselhi = (hartsel >> 10) & 0b1111111111;
}

} // namespace

c_debug_regs::c_debug_regs(
    c_soft_dm* dm,
    const size_t hart_count,
    const size_t data_size,
    const size_t progbuf_size,
    std::function<void(bool)> callback_signal_debug_haltrequest)
    : dm{dm}
    , hartsellen{make_hartsellen(hart_count)}
    , data_size{data_size}
    , progbuf_size{progbuf_size}
    , callback_signal_debug_haltrequest{callback_signal_debug_haltrequest}
    , data{std::move(make_default_regs_data(data_size))}
    , dmcontrol{0}
    , dmstatus{make_default_reg_dmstatus()}
    , abstractcs{data_size, progbuf_size}
    , command{dm, &abstractcs}
    , abstractauto{0}
    , progbuf{std::move(make_default_regs_progbuf(progbuf_size))}
    , haltsum0{0}
{
    PN_ASSERT(dm != nullptr);
}

void c_debug_regs::write_reg(uint8_t address, uint32_t data)
{
    e_reg_address reg_address = static_cast<e_reg_address>(address);
    switch(reg_address)
    {
        case e_reg_address::data0:
            write_reg_data(
                e_data_reg_index::DATA0,
                int_to_reg<data_t>(data));
            return;
        case e_reg_address::data1:
            write_reg_data(
                e_data_reg_index::DATA1,
                int_to_reg<data_t>(data));
            return;
        case e_reg_address::dmcontrol:
            write_reg_dmcontrol(
                int_to_reg<dmcontrol_t>(data));
            return;
        case e_reg_address::dmstatus:
            write_reg_dmstatus(
                int_to_reg<dmstatus_t>(data));
            return;
        case e_reg_address::abstractcs:
            write_reg_abstractcs(
                int_to_reg<c_debug_reg_abstractcs::reg_t>(data));
            return;
        case e_reg_address::command:
            write_reg_command(
                int_to_reg<c_debug_reg_command::reg_t>(data));
            return;
        case e_reg_address::abstractauto:
            write_reg_abstractauto(
                int_to_reg<abstractauto_t>(data));
            return;
        case e_reg_address::progbuf0:
            write_reg_progbuf(
                e_progbuf_reg_index::PROGBUF0,
                int_to_reg<progbuf_t>(data));
            return;
        case e_reg_address::progbuf1:
            write_reg_progbuf(
                e_progbuf_reg_index::PROGBUF1,
                int_to_reg<progbuf_t>(data));
            return;
        case e_reg_address::progbuf2:
            write_reg_progbuf(
                e_progbuf_reg_index::PROGBUF2,
                int_to_reg<progbuf_t>(data));
            return;
        case e_reg_address::haltsum0:
            write_reg_haltsum0(
                int_to_reg<haltsum0_t>(data));
            return;
    }

    PN_INFOF(("c_debug_regs: Tried to write to uninitilized register: 0x%x", (int)address));
}

uint32_t c_debug_regs::read_reg(uint8_t address)
{
    e_reg_address reg_address = static_cast<e_reg_address>(address);
    switch(reg_address)
    {
        case e_reg_address::data0:
            return reg_to_int(
                read_reg_data(
                    c_debug_regs::e_data_reg_index::DATA0));
        case e_reg_address::data1:
            return reg_to_int(
                read_reg_data(
                    c_debug_regs::e_data_reg_index::DATA1));
        case e_reg_address::dmcontrol:
            return reg_to_int(
                read_reg_dmcontrol());
        case e_reg_address::dmstatus:
            return reg_to_int(
                read_reg_dmstatus());
        case e_reg_address::abstractcs:
            return reg_to_int(
                read_reg_abstractcs());
        case e_reg_address::command:
            return reg_to_int(
                read_reg_command());
        case e_reg_address::abstractauto:
            return reg_to_int(
                read_reg_abstractauto());
        case e_reg_address::progbuf0:
            return reg_to_int(
                read_reg_progbuf(e_progbuf_reg_index::PROGBUF0));
        case e_reg_address::progbuf1:
            return reg_to_int(
                read_reg_progbuf(e_progbuf_reg_index::PROGBUF1));
        case e_reg_address::progbuf2:
            return reg_to_int(
                read_reg_progbuf(e_progbuf_reg_index::PROGBUF2));
        case e_reg_address::haltsum0:
            return reg_to_int(
                read_reg_haltsum0());
    }

    PN_INFOF(("c_debug_regs: Tried to read to uninitilized register: 0x%x", (int)address));
    return 0;
}

void c_debug_regs::write_reg_data(
    e_data_reg_index index,
    data_t reg)
{
    _handle_reg_access_while_command_running();

    if(c_debug_reg_abstractcs::e_busy::SET == abstractcs.busy())
    {
        return;
    }

    data[static_cast<size_t>(index)] = reg;
}

void c_debug_regs::write_reg_dmcontrol(dmcontrol_t reg)
{
    dmcontrol = reg;

    _check_hartsellen_detection();
    _check_hart_exists();
    _check_hart_halt_request();
    _check_hart_resume_request();
}

void c_debug_regs::write_reg_dmstatus(dmstatus_t /*reg*/)
{
    // Entire register is read-only
    return;
}

void c_debug_regs::write_reg_abstractcs(c_debug_reg_abstractcs::reg_t reg)
{
    _handle_reg_access_while_command_running();

    abstractcs.write(reg_to_int(reg));
}

void c_debug_regs::write_reg_command(c_debug_reg_command::reg_t reg)
{
    _handle_reg_access_while_command_running();

    if(c_debug_reg_abstractcs::e_cmderr::NONE != abstractcs.cmderr())
    {
        return;
    }

    command.write(reg_to_int(reg));
}

void c_debug_regs::write_reg_abstractauto(abstractauto_t reg)
{
    _handle_reg_access_while_command_running();
    abstractauto = reg;
}

void c_debug_regs::write_reg_progbuf(
    c_debug_regs::e_progbuf_reg_index index,
    progbuf_t reg)
{
    if(static_cast<size_t>(index) < 0 ||
        static_cast<size_t>(index) >= progbuf.size())
    {
        return;
    }

    _handle_reg_access_while_command_running();

    if(c_debug_reg_abstractcs::e_busy::SET == abstractcs.busy())
    {
        return;
    }

    progbuf[static_cast<size_t>(index)] = reg;
}

void c_debug_regs::write_reg_haltsum0(haltsum0_t /*reg*/)
{
    // Entire register is read-only
    return;
}

// --------------------------------------------------

c_debug_regs::data_t c_debug_regs::read_reg_data(
    e_data_reg_index index)
{
    _handle_reg_access_while_command_running();

    return data[static_cast<size_t>(index)];
}

c_debug_regs::dmcontrol_t c_debug_regs::read_reg_dmcontrol()
{
    return dmcontrol;
}

c_debug_regs::dmstatus_t c_debug_regs::read_reg_dmstatus()
{
    dmstatus.allhalted = dm->hartstatus_reg.halted;
    dmstatus.anyhalted = dm->hartstatus_reg.halted;

    dmstatus.allrunning = dm->hartstatus_reg.running;
    dmstatus.anyrunning = dm->hartstatus_reg.running;

    dmstatus.allhavereset = dm->hartstatus_reg.havereset;
    dmstatus.anyhavereset = dm->hartstatus_reg.havereset;

    return dmstatus;
}

c_debug_reg_abstractcs::reg_t c_debug_regs::read_reg_abstractcs()
{
    return int_to_reg<c_debug_reg_abstractcs::reg_t>(abstractcs.read());
}

c_debug_reg_command::reg_t c_debug_regs::read_reg_command()
{
    // Entire register is write-only
    return {};
}

c_debug_regs::abstractauto_t c_debug_regs::read_reg_abstractauto()
{
    // TODO: Remove comment when this is supported.
    // return abstractauto;
    return {};
}

c_debug_regs::haltsum0_t c_debug_regs::read_reg_haltsum0()
{
    return haltsum0;
}

c_debug_regs::progbuf_t c_debug_regs::read_reg_progbuf(c_debug_regs::e_progbuf_reg_index index)
{
    if(static_cast<size_t>(index) < 0 ||
        static_cast<size_t>(index) >= progbuf.size())
    {
        return {};
    }

    _handle_reg_access_while_command_running();

    return progbuf[static_cast<size_t>(index)];
}

// --------------------------------------------------

void c_debug_regs::_check_hartsellen_detection()
{
    const uint32_t hartsel_maximum = 0b11111111111111111111;
    const uint32_t hartsel = get_hartsel(dmcontrol);

    if(hartsel_maximum == hartsel)
    {
        set_hartsel(dmcontrol, hartsellen);
    }
}

void c_debug_regs::_check_hart_exists()
{
    const uint32_t hartsel = get_hartsel(dmcontrol);

    // TODO: remove magic number here.
    if(0 != hartsel)
    {
        dmstatus.allnonexistent = 1;
        dmstatus.anynonexistent = 1;
    }
    else
    {
        dmstatus.allnonexistent = 0;
        dmstatus.anynonexistent = 0;
    }
}
void c_debug_regs::_check_hart_halt_request()
{
    bool haltreq = dmcontrol.haltreq;
    callback_signal_debug_haltrequest(haltreq);
}

void c_debug_regs::_check_hart_resume_request()
{
    dm->hartcontrol_reg.resumereq = dmcontrol.resumereq;

    if(dmcontrol.resumereq == 1)
    {
        dmstatus.allresumeack = 0;
        dmstatus.anyresumeack = 0;
    }
}

void c_debug_regs::_handle_reg_access_while_command_running()
{
    if(!dm->hartstatus_reg.commands_running)
    {
        return;
    }

    if(c_debug_reg_abstractcs::e_cmderr::NONE != abstractcs.cmderr())
    {
        return;
    }

    abstractcs.set_cmderr(c_debug_reg_abstractcs::e_cmderr::BUSY);
}
