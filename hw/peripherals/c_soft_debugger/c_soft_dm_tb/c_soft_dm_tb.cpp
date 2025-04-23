/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Johannes Hofmann <johannes.hofmann1@tha.de>
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

#include "c_soft_dm.h"

#include "debug_handler.h"
#include "reg_cast.h"

#include <base.h>

#include <array>
#include <cstdint>
#include <functional>
#include <memory>

/////////////// Helpers ///////////////
std::unique_ptr<c_soft_dm> make_default_dm();

/////////////// Tests ///////////////
// hart
void test_hartcontrol_set_haltrequest();
void test_hartcontrol_set_resumerequest();
void test_hartstatus_haltrequest_ack_sets_halted_flag();

// abstract commands
void test_abstract_command_busy_flag_set_during_execution();

void test_abstract_command_reg_access_gpr_write();
void test_abstract_command_reg_access_gpr_read();
void test_abstract_command_reg_access_csr_write();
void test_abstract_command_reg_access_csr_read();
void test_abstract_command_reg_access_memory_write();
void test_abstract_command_reg_access_memory_read();

// debug handler
void test_debug_handler_read_access();
void test_debug_handler_no_write_access();

int sc_main(int argc, char** argv)
{
    PN_INFO("c_dtm_tb: Start unit test ...");

    // hart
    test_hartcontrol_set_haltrequest();
    test_hartcontrol_set_resumerequest();

    test_hartstatus_haltrequest_ack_sets_halted_flag();

    // abstract commands
    test_abstract_command_busy_flag_set_during_execution();

    test_abstract_command_reg_access_gpr_write();
    test_abstract_command_reg_access_gpr_read();
    test_abstract_command_reg_access_csr_write();
    test_abstract_command_reg_access_csr_read();
    test_abstract_command_reg_access_memory_write();
    test_abstract_command_reg_access_memory_read();

    // debug handler
    test_debug_handler_read_access();
    test_debug_handler_no_write_access();

    PN_INFO("c_dtm_tb: All tests passed");

    return 0;
}

/////////////// Helpers ///////////////
std::unique_ptr<c_soft_dm> make_default_dm()
{
    auto callback_signal_debug_haltrequest = [](bool value) {
        PN_ASSERT(false);
    };

    return std::make_unique<c_soft_dm>(
        callback_signal_debug_haltrequest);
}

/////////////// Tests ///////////////
void test_hartcontrol_set_haltrequest()
{
    PN_INFO("c_soft_dm_tb: run test test_hartcontrol_set_haltrequest() ...");

    int count_callback_signal_debug_haltrequest = 0;

    auto callback_signal_debug_haltrequest =
        [&count_callback_signal_debug_haltrequest](bool value) {
            ++count_callback_signal_debug_haltrequest;
        };

    auto dm = std::make_unique<c_soft_dm>(
        callback_signal_debug_haltrequest);

    c_debug_regs::dmcontrol_t dmcontrol_reg = {0};
    dmcontrol_reg.haltreq = 1;

    PN_ASSERT(count_callback_signal_debug_haltrequest == 0);

    dm->dmi_write(
        static_cast<uint8_t>(c_debug_regs::e_reg_address::dmcontrol),
        reg_to_int(dmcontrol_reg));

    PN_ASSERT(count_callback_signal_debug_haltrequest == 1);

    PN_INFO("c_soft_dm_tb: test passed");
}

void test_hartcontrol_set_resumerequest()
{
    PN_INFO("c_soft_dm_tb: run test test_hartcontrol_set_resumerequest() ...");

    auto callback_signal_debug_haltrequest = [](bool value) {
        PN_ASSERT(value == 0);
    };

    auto dm = std::make_unique<c_soft_dm>(
        callback_signal_debug_haltrequest);

    c_debug_regs::dmcontrol_t dmcontrol_reg = {0};
    dmcontrol_reg.resumereq = 1;

    uint32_t hartcontrol = dm->read32(
        static_cast<uint64_t>(c_soft_dm::e_regs::HARTCONTROL));

    PN_ASSERT(hartcontrol == 0);

    dm->dmi_write(
        static_cast<uint8_t>(c_debug_regs::e_reg_address::dmcontrol),
        reg_to_int(dmcontrol_reg));

    hartcontrol = dm->read32(
        static_cast<uint64_t>(c_soft_dm::e_regs::HARTCONTROL));

    PN_ASSERT(hartcontrol == 1);

    PN_INFO("c_soft_dm_tb: test passed");
}

void test_hartstatus_haltrequest_ack_sets_halted_flag()
{
    PN_INFO("c_soft_dm_tb: run test test_hartstatus_haltrequest_ack_sets_halted_flag() ...");

    auto dm = make_default_dm();

    c_debug_regs::dmstatus_t dmstatus_reg =
        int_to_reg<c_debug_regs::dmstatus_t>(
            dm->dmi_read(
                static_cast<uint64_t>(c_debug_regs::e_reg_address::dmstatus)));
    PN_ASSERT(dmstatus_reg.allhalted == 0);
    PN_ASSERT(dmstatus_reg.anyhalted == 0);

    dm->set_signal_debug_haltrequest_ack(1);

    dmstatus_reg =
        int_to_reg<c_debug_regs::dmstatus_t>(
            dm->dmi_read(
                static_cast<uint64_t>(c_debug_regs::e_reg_address::dmstatus)));
    PN_ASSERT(dmstatus_reg.allhalted == 1);
    PN_ASSERT(dmstatus_reg.anyhalted == 1);

    PN_INFO("c_soft_dm_tb: test passed");
}

void test_abstract_command_busy_flag_set_during_execution()
{
    PN_INFO("c_soft_dm_tb: run test test_abstract_command_busy_flag_set_during_execution() ...");

    auto dm = make_default_dm();

    // halt hart before testing abstract command
    c_soft_dm::hartstatus_t hartstatus_reg = {0};
    hartstatus_reg.halted = 1;
    dm->write32(
        static_cast<uint64_t>(c_soft_dm::e_regs::HARTSTATUS),
        reg_to_int(hartstatus_reg));

    // use gpr write command as dummy
    c_debug_reg_command::reg_command_reg_access_t command_reg = {0};
    command_reg.regno = 0x1000 + 0x1;
    command_reg.write = 1;
    command_reg.transfer = 1;
    command_reg.aarsize = c_debug_reg_command::e_cmd_reg_access_aarsize::LOWEST_32;
    command_reg.cmdtype = c_debug_reg_command::e_cmdtype::REG_ACCESS;

    dm->dmi_write(
        static_cast<uint8_t>(c_debug_regs::e_reg_address::command),
        reg_to_int(command_reg));

    c_soft_dm::hartcontrol_t hartcontrol_reg =
        int_to_reg<c_soft_dm::hartcontrol_t>(
            dm->read32(
                static_cast<uint64_t>(c_soft_dm::e_regs::HARTCONTROL)));
    PN_ASSERT(hartcontrol_reg.run_commandsreq == 1);

    c_debug_reg_abstractcs::reg_t abstractcs_reg =
        int_to_reg<c_debug_reg_abstractcs::reg_t>(
            dm->dmi_read(
                static_cast<uint64_t>(c_debug_regs::e_reg_address::abstractcs)));
    PN_ASSERT(abstractcs_reg.busy == c_debug_reg_abstractcs::e_busy::SET);

    // Start executing commands
    hartstatus_reg.halted = 1;
    hartstatus_reg.commands_running = 1;
    dm->write32(
        static_cast<uint64_t>(c_soft_dm::e_regs::HARTSTATUS),
        reg_to_int(hartstatus_reg));

    // Finished executing commands
    hartstatus_reg.halted = 1;
    hartstatus_reg.commands_running = 0;
    dm->write32(
        static_cast<uint64_t>(c_soft_dm::e_regs::HARTSTATUS),
        reg_to_int(hartstatus_reg));

    abstractcs_reg =
        int_to_reg<c_debug_reg_abstractcs::reg_t>(
            dm->read32(
                static_cast<uint64_t>(c_debug_regs::e_reg_address::abstractcs)));
    PN_ASSERT(abstractcs_reg.busy == c_debug_reg_abstractcs::e_busy::RESET);

    PN_INFO("c_soft_dm_tb: test passed");
}

void test_abstract_command_reg_access_gpr_write()
{
    PN_INFO("c_soft_dm_tb: run test test_abstract_command_reg_access_gpr_write() ...");

    auto dm = make_default_dm();

    // halt hart before testing abstract command
    c_soft_dm::hartstatus_t hartstatus_reg = {0};
    hartstatus_reg.halted = 1;
    dm->write32(
        static_cast<uint64_t>(c_soft_dm::e_regs::HARTSTATUS),
        reg_to_int(hartstatus_reg));

    c_debug_reg_command::reg_command_reg_access_t command_reg = {0};
    command_reg.regno = 0x1000 + 0x1;
    command_reg.write = 1;
    command_reg.transfer = 1;
    command_reg.aarsize = c_debug_reg_command::e_cmd_reg_access_aarsize::LOWEST_32;
    command_reg.cmdtype = c_debug_reg_command::e_cmdtype::REG_ACCESS;

    dm->dmi_write(
        static_cast<uint8_t>(c_debug_regs::e_reg_address::command),
        reg_to_int(command_reg));

    c_soft_dm::hartcontrol_t hartcontrol_reg =
        int_to_reg<c_soft_dm::hartcontrol_t>(
            dm->read32(
                static_cast<uint64_t>(c_soft_dm::e_regs::HARTCONTROL)));
    PN_ASSERT(hartcontrol_reg.run_commandsreq == 1);

    std::array<uint32_t, 2> abstract_command_regs;
    abstract_command_regs[0] = dm->read32(
        static_cast<uint64_t>(c_soft_dm::e_regs::ABSTRACT_COMMAND0));
    abstract_command_regs[1] = dm->read32(
        static_cast<uint64_t>(c_soft_dm::e_regs::ABSTRACT_COMMAND1));

    PN_ASSERT(abstract_command_regs[0] == 0x00002083);
    PN_ASSERT(abstract_command_regs[1] == 0x00100073); // ebreak

    PN_INFO("c_soft_dm_tb: test passed");
}

void test_abstract_command_reg_access_gpr_read()
{
    PN_INFO("c_soft_dm_tb: run test test_abstract_command_reg_access_gpr_read() ...");

    auto dm = make_default_dm();

    // halt hart before testing abstract command
    c_soft_dm::hartstatus_t hartstatus_reg = {0};
    hartstatus_reg.halted = 1;
    dm->write32(
        static_cast<uint64_t>(c_soft_dm::e_regs::HARTSTATUS),
        reg_to_int(hartstatus_reg));

    c_debug_reg_command::reg_command_reg_access_t command_reg = {0};
    command_reg.regno = 0x1000 + 0x1;
    command_reg.write = 0;
    command_reg.transfer = 1;
    command_reg.aarsize = c_debug_reg_command::e_cmd_reg_access_aarsize::LOWEST_32;
    command_reg.cmdtype = c_debug_reg_command::e_cmdtype::REG_ACCESS;

    dm->dmi_write(
        static_cast<uint8_t>(c_debug_regs::e_reg_address::command),
        reg_to_int(command_reg));

    c_soft_dm::hartcontrol_t hartcontrol_reg =
        int_to_reg<c_soft_dm::hartcontrol_t>(
            dm->read32(
                static_cast<uint64_t>(c_soft_dm::e_regs::HARTCONTROL)));
    PN_ASSERT(hartcontrol_reg.run_commandsreq == 1);

    std::array<uint32_t, 2> abstract_command_regs;
    abstract_command_regs[0] = dm->read32(
        static_cast<uint64_t>(c_soft_dm::e_regs::ABSTRACT_COMMAND0));
    abstract_command_regs[1] = dm->read32(
        static_cast<uint64_t>(c_soft_dm::e_regs::ABSTRACT_COMMAND1));

    PN_ASSERT(abstract_command_regs[0] == 0x00102023);
    PN_ASSERT(abstract_command_regs[1] == 0x00100073); // ebreak

    PN_INFO("c_soft_dm_tb: test passed");
}

void test_abstract_command_reg_access_csr_write()
{
    PN_INFO("c_soft_dm_tb: run test test_abstract_command_reg_access_csr_write() ...");

    auto dm = make_default_dm();

    // halt hart before testing abstract command
    c_soft_dm::hartstatus_t hartstatus_reg = {0};
    hartstatus_reg.halted = 1;
    dm->write32(
        static_cast<uint64_t>(c_soft_dm::e_regs::HARTSTATUS),
        reg_to_int(hartstatus_reg));

    c_debug_reg_command::reg_command_reg_access_t command_reg = {0};
    command_reg.regno = 0x1;
    command_reg.write = 1;
    command_reg.transfer = 1;
    command_reg.aarsize = c_debug_reg_command::e_cmd_reg_access_aarsize::LOWEST_32;
    command_reg.cmdtype = c_debug_reg_command::e_cmdtype::REG_ACCESS;

    dm->dmi_write(
        static_cast<uint8_t>(c_debug_regs::e_reg_address::command),
        reg_to_int(command_reg));

    c_soft_dm::hartcontrol_t hartcontrol_reg =
        int_to_reg<c_soft_dm::hartcontrol_t>(
            dm->read32(
                static_cast<uint64_t>(c_soft_dm::e_regs::HARTCONTROL)));
    PN_ASSERT(hartcontrol_reg.run_commandsreq == 1);

    std::array<uint32_t, 3> abstract_command_regs;
    abstract_command_regs[0] = dm->read32(
        static_cast<uint64_t>(c_soft_dm::e_regs::ABSTRACT_COMMAND0));
    abstract_command_regs[1] = dm->read32(
        static_cast<uint64_t>(c_soft_dm::e_regs::ABSTRACT_COMMAND1));
    abstract_command_regs[2] = dm->read32(
        static_cast<uint64_t>(c_soft_dm::e_regs::ABSTRACT_COMMAND2));

    PN_ASSERT(abstract_command_regs[0] == 0x00002f83);
    PN_ASSERT(abstract_command_regs[1] == 0x001f9073);
    PN_ASSERT(abstract_command_regs[2] == 0x00100073); // ebreak

    PN_INFO("c_soft_dm_tb: test passed");
}

void test_abstract_command_reg_access_csr_read()
{
    PN_INFO("c_soft_dm_tb: run test test_abstract_command_reg_access_csr_read() ...");

    auto dm = make_default_dm();

    // halt hart before testing abstract command
    c_soft_dm::hartstatus_t hartstatus_reg = {0};
    hartstatus_reg.halted = 1;
    dm->write32(
        static_cast<uint64_t>(c_soft_dm::e_regs::HARTSTATUS),
        reg_to_int(hartstatus_reg));

    c_debug_reg_command::reg_command_reg_access_t command_reg = {0};
    command_reg.regno = 0x1;
    command_reg.write = 0;
    command_reg.transfer = 1;
    command_reg.aarsize = c_debug_reg_command::e_cmd_reg_access_aarsize::LOWEST_32;
    command_reg.cmdtype = c_debug_reg_command::e_cmdtype::REG_ACCESS;

    dm->dmi_write(
        static_cast<uint8_t>(c_debug_regs::e_reg_address::command),
        reg_to_int(command_reg));

    c_soft_dm::hartcontrol_t hartcontrol_reg =
        int_to_reg<c_soft_dm::hartcontrol_t>(
            dm->read32(
                static_cast<uint64_t>(c_soft_dm::e_regs::HARTCONTROL)));
    PN_ASSERT(hartcontrol_reg.run_commandsreq == 1);

    std::array<uint32_t, 3> abstract_command_regs;
    abstract_command_regs[0] = dm->read32(
        static_cast<uint64_t>(c_soft_dm::e_regs::ABSTRACT_COMMAND0));
    abstract_command_regs[1] = dm->read32(
        static_cast<uint64_t>(c_soft_dm::e_regs::ABSTRACT_COMMAND1));
    abstract_command_regs[2] = dm->read32(
        static_cast<uint64_t>(c_soft_dm::e_regs::ABSTRACT_COMMAND2));

    PN_ASSERT(abstract_command_regs[0] == 0x00102ff3);
    PN_ASSERT(abstract_command_regs[1] == 0x01f02023);
    PN_ASSERT(abstract_command_regs[2] == 0x00100073); // ebreak

    PN_INFO("c_soft_dm_tb: test passed");
}

void test_abstract_command_reg_access_memory_write()
{
    PN_INFO("c_soft_dm_tb: run test test_abstract_command_reg_access_memory_write() ...");

    auto dm = make_default_dm();

    // halt hart before testing abstract command
    c_soft_dm::hartstatus_t hartstatus_reg = {0};
    hartstatus_reg.halted = 1;
    dm->write32(
        static_cast<uint64_t>(c_soft_dm::e_regs::HARTSTATUS),
        reg_to_int(hartstatus_reg));

    c_debug_reg_command::reg_command_memory_access_t command_reg = {0};
    command_reg.write = 1;
    command_reg.aamsize = c_debug_reg_command::e_cmd_memory_access_aamsize::BITS_32;
    command_reg.cmdtype = c_debug_reg_command::e_cmdtype::MEMORY_ACCESS;

    dm->dmi_write(
        static_cast<uint8_t>(c_debug_regs::e_reg_address::command),
        reg_to_int(command_reg));

    c_soft_dm::hartcontrol_t hartcontrol_reg =
        int_to_reg<c_soft_dm::hartcontrol_t>(
            dm->read32(
                static_cast<uint64_t>(c_soft_dm::e_regs::HARTCONTROL)));
    PN_ASSERT(hartcontrol_reg.run_commandsreq == 1);

    std::array<uint32_t, 4> abstract_command_regs;
    abstract_command_regs[0] = dm->read32(
        static_cast<uint64_t>(c_soft_dm::e_regs::ABSTRACT_COMMAND0));
    abstract_command_regs[1] = dm->read32(
        static_cast<uint64_t>(c_soft_dm::e_regs::ABSTRACT_COMMAND1));
    abstract_command_regs[2] = dm->read32(
        static_cast<uint64_t>(c_soft_dm::e_regs::ABSTRACT_COMMAND2));
    abstract_command_regs[3] = dm->read32(
        static_cast<uint64_t>(c_soft_dm::e_regs::ABSTRACT_COMMAND3));

    PN_ASSERT(abstract_command_regs[0] == 0x00402f83);
    PN_ASSERT(abstract_command_regs[1] == 0x00002f03);
    PN_ASSERT(abstract_command_regs[2] == 0x01efa023);
    PN_ASSERT(abstract_command_regs[3] == 0x00100073); // ebreak

    PN_INFO("c_soft_dm_tb: test passed");
}

void test_abstract_command_reg_access_memory_read()
{
    PN_INFO("c_soft_dm_tb: run test test_abstract_command_reg_access_memory_read() ...");

    auto dm = make_default_dm();

    // halt hart before testing abstract command
    c_soft_dm::hartstatus_t hartstatus_reg = {0};
    hartstatus_reg.halted = 1;
    dm->write32(
        static_cast<uint64_t>(c_soft_dm::e_regs::HARTSTATUS),
        reg_to_int(hartstatus_reg));

    c_debug_reg_command::reg_command_memory_access_t command_reg = {0};
    command_reg.write = 0;
    command_reg.aamsize = c_debug_reg_command::e_cmd_memory_access_aamsize::BITS_32;
    command_reg.cmdtype = c_debug_reg_command::e_cmdtype::MEMORY_ACCESS;

    dm->dmi_write(
        static_cast<uint8_t>(c_debug_regs::e_reg_address::command),
        reg_to_int(command_reg));

    c_soft_dm::hartcontrol_t hartcontrol_reg =
        int_to_reg<c_soft_dm::hartcontrol_t>(
            dm->read32(
                static_cast<uint64_t>(c_soft_dm::e_regs::HARTCONTROL)));
    PN_ASSERT(hartcontrol_reg.run_commandsreq == 1);

    std::array<uint32_t, 4> abstract_command_regs;
    abstract_command_regs[0] = dm->read32(
        static_cast<uint64_t>(c_soft_dm::e_regs::ABSTRACT_COMMAND0));
    abstract_command_regs[1] = dm->read32(
        static_cast<uint64_t>(c_soft_dm::e_regs::ABSTRACT_COMMAND1));
    abstract_command_regs[2] = dm->read32(
        static_cast<uint64_t>(c_soft_dm::e_regs::ABSTRACT_COMMAND2));
    abstract_command_regs[3] = dm->read32(
        static_cast<uint64_t>(c_soft_dm::e_regs::ABSTRACT_COMMAND3));

    PN_ASSERT(abstract_command_regs[0] == 0x00402f83);
    PN_ASSERT(abstract_command_regs[1] == 0x000faf83);
    PN_ASSERT(abstract_command_regs[2] == 0x01f02023);
    PN_ASSERT(abstract_command_regs[3] == 0x00100073); // ebreak

    PN_INFO("c_soft_dm_tb: test passed");
}

void test_debug_handler_read_access()
{
    PN_INFO("c_soft_dm_tb: run test test_debug_handler_read_access() ...");

    auto dm = make_default_dm();

    const uint64_t debug_rom_start_address =
        static_cast<uint64_t>(c_soft_dm::e_regs::DEBUG_HANDLER_START);

    for(int i = 0; i < debug_handler::binary_wsize; ++i)
    {
        const uint32_t expected = debug_handler::binary[i];

        const uint64_t address = debug_rom_start_address + i * 4;
        const uint32_t actual = dm->read32(address);

        PN_ASSERT(actual == expected);
    }

    PN_INFO("c_soft_dm_tb: test passed");
}

void test_debug_handler_no_write_access()
{
    PN_INFO("c_soft_dm_tb: run test test_debug_handler_no_write_access() ...");

    auto dm = make_default_dm();

    const uint32_t dummy = 0;
    dm->write32(dummy, CFG_DEBUG_DEBUG_HANDLER_START_ADDRESS);

    const uint32_t expected = debug_handler::binary[0];
    const uint32_t actual = dm->read32(CFG_DEBUG_DEBUG_HANDLER_START_ADDRESS);

    PN_ASSERT(actual == expected);

    PN_INFO("c_soft_dm_tb: test passed");
}
