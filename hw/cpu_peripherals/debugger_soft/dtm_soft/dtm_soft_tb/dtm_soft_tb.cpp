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

#include "../dtm_soft.h"

#include "c_fake_jtag_adapter.h"

#include <piconut.h>
#include <piconut-config.h>

#include <cstdint>

/////////////// Tests ///////////////
void test_jtag_reset_calls_dmi_reset();
void test_instruction_reg();
void test_data_reg_bypass();
void test_data_reg_idcode();
void test_data_reg_dtmcs();
void test_data_reg_dmi_write();
void test_data_reg_dmi_read();

int sc_main(int argc, char** argv)
{
    PN_INFO("c_dtm_tb: Start unit test ...");

    test_jtag_reset_calls_dmi_reset();
    test_instruction_reg();
    test_data_reg_bypass();
    test_data_reg_idcode();
    test_data_reg_dtmcs();
    test_data_reg_dmi_write();
    test_data_reg_dmi_read();

    PN_INFO("c_dtm_tb: All tests passed");

    return 0;
}

void test_jtag_reset_calls_dmi_reset()
{
    PN_INFO("c_dtm_tb: run test test_jtag_reset_calls_dmi_reset() ...");

    int callback_dmi_reset_callcount = 0;

    auto callback_dmi_write = [](uint8_t address, uint32_t data) {
        PN_ASSERT(false);
    };

    auto callback_dmi_read = [](uint8_t address) -> uint32_t {
        PN_ASSERT(false);
        return 0;
    };

    auto callback_dmi_reset = [&callback_dmi_reset_callcount]() {
        ++callback_dmi_reset_callcount;
    };

    c_dtm dtm{
        callback_dmi_write,
        callback_dmi_read,
        callback_dmi_reset};

    dtm.jtag_reset();

    PN_ASSERT(callback_dmi_reset_callcount == 1);

    PN_INFO("c_dtm_tb: test passed");
}

void test_instruction_reg()
{
    PN_INFO("c_dtm_tb: run test test_instruction_reg() ...");

    auto callback_dmi_write = [](uint8_t address, uint32_t data) {
        PN_ASSERT(false);
    };

    auto callback_dmi_read = [](uint8_t address) -> uint32_t {
        PN_ASSERT(false);
        return 0;
    };

    auto callback_dmi_reset = []() {
        PN_ASSERT(false);
    };

    c_dtm dtm{
        callback_dmi_write,
        callback_dmi_read,
        callback_dmi_reset};

    c_fake_jtag_adapter fake_jtag_adapter{&dtm};

    uint64_t default_instruction = c_dtm::e_data_reg_address::IDCODE;
    uint64_t input_instruction = c_dtm::e_data_reg_address::DMI;
    uint64_t output_instruction = 0;

    output_instruction =
        fake_jtag_adapter.shift_jtag_instruction_register(
            input_instruction);
    PN_ASSERT(default_instruction == output_instruction);

    output_instruction =
        fake_jtag_adapter.shift_jtag_instruction_register(
            default_instruction);
    PN_ASSERT(input_instruction == output_instruction);

    output_instruction =
        fake_jtag_adapter.shift_jtag_instruction_register(
            default_instruction);
    PN_ASSERT(default_instruction == output_instruction);

    PN_INFO("c_dtm_tb: test passed");
}

void test_data_reg_bypass()
{
    PN_INFO("c_dtm_tb: run test test_data_reg_bypass() ...");

    auto callback_dmi_write = [](uint8_t address, uint32_t data) {
        PN_ASSERT(false);
    };

    auto callback_dmi_read = [](uint8_t address) -> uint32_t {
        PN_ASSERT(false);
        return 0;
    };

    auto callback_dmi_reset = []() {
        PN_ASSERT(false);
    };

    c_dtm dtm{
        callback_dmi_write,
        callback_dmi_read,
        callback_dmi_reset};

    c_fake_jtag_adapter fake_jtag_adapter{&dtm};

    uint64_t default_bypass = 0;
    uint64_t input_bypass = 1;
    uint64_t output_bypass = 0;

    c_dtm::e_data_reg_address data_reg = c_dtm::e_data_reg_address::BYPASS;

    output_bypass =
        fake_jtag_adapter.shift_jtag_data_register(
            data_reg,
            input_bypass);
    PN_ASSERT(default_bypass == output_bypass);

    output_bypass =
        fake_jtag_adapter.shift_jtag_data_register(
            data_reg,
            default_bypass);
    PN_ASSERT(default_bypass == output_bypass);

    PN_INFO("c_dtm_tb: test passed");
}

void test_data_reg_idcode()
{
    PN_INFO("c_dtm_tb: run test test_data_reg_idcode() ...");

    auto callback_dmi_write = [](uint8_t address, uint32_t data) {
        PN_ASSERT(false);
    };

    auto callback_dmi_read = [](uint8_t address) -> uint32_t {
        PN_ASSERT(false);
        return 0;
    };

    auto callback_dmi_reset = []() {
        PN_ASSERT(false);
    };

    c_dtm dtm{
        callback_dmi_write,
        callback_dmi_read,
        callback_dmi_reset};

    c_fake_jtag_adapter fake_jtag_adapter{&dtm};

    uint64_t default_data = 0xdeadbeef;
    uint64_t input_data = 1;
    uint64_t output_data = 0;

    c_dtm::e_data_reg_address data_reg = c_dtm::e_data_reg_address::IDCODE;

    output_data =
        fake_jtag_adapter.shift_jtag_data_register(
            data_reg,
            input_data);
    PN_ASSERT(default_data == output_data);

    output_data =
        fake_jtag_adapter.shift_jtag_data_register(
            data_reg,
            default_data);
    PN_ASSERT(default_data == output_data);

    PN_INFO("c_dtm_tb: test passed");
}

void test_data_reg_dtmcs()
{
    PN_INFO("c_dtm_tb: run test test_data_reg_dtmcs() ...");

    auto callback_dmi_write = [](uint8_t address, uint32_t data) {
        PN_ASSERT(false);
    };

    auto callback_dmi_read = [](uint8_t address) -> uint32_t {
        PN_ASSERT(false);
        return 0;
    };

    auto callback_dmi_reset = []() {
        PN_ASSERT(false);
    };

    c_dtm dtm{
        callback_dmi_write,
        callback_dmi_read,
        callback_dmi_reset};

    c_fake_jtag_adapter fake_jtag_adapter{&dtm};

    uint64_t default_data = (4U << 12U) | (PN_CFG_DEBUG_DMI_BUS_ADR_WIDTH << 4U) | 1U;
    uint64_t input_data = 1;
    uint64_t output_data = 0;

    c_dtm::e_data_reg_address data_reg = c_dtm::e_data_reg_address::DTMCS;

    output_data =
        fake_jtag_adapter.shift_jtag_data_register(
            data_reg,
            input_data);
    PN_ASSERT(default_data == output_data);

    output_data =
        fake_jtag_adapter.shift_jtag_data_register(
            data_reg,
            default_data);
    PN_ASSERT(default_data == output_data);

    PN_INFO("c_dtm_tb: test passed");
}

void test_data_reg_dmi_write()
{
    PN_INFO("c_dtm_tb: run test test_data_reg_dmi_write() ...");

    auto callback_dmi_write = [](uint8_t address, uint32_t data) {
        PN_ASSERT(address == 0x15);
        PN_ASSERT(data == 0xaffeaffe);
    };

    auto callback_dmi_read = [](uint8_t address) -> uint32_t {
        PN_ASSERT(false);
        return 0;
    };

    auto callback_dmi_reset = []() {
        PN_ASSERT(false);
    };

    c_dtm dtm{
        callback_dmi_write,
        callback_dmi_read,
        callback_dmi_reset};

    c_fake_jtag_adapter fake_jtag_adapter{&dtm};

    uint64_t default_data = 0;
    uint64_t input_data =
        0x15UL << 34 |      // address
        0xaffeaffeUL << 2 | // data
        2UL;                // opcode write
    uint64_t output_data = 0;

    c_dtm::e_data_reg_address data_reg = c_dtm::e_data_reg_address::DMI;

    output_data =
        fake_jtag_adapter.shift_jtag_data_register(
            data_reg,
            input_data);
    PN_ASSERT(default_data == output_data);

    output_data =
        fake_jtag_adapter.shift_jtag_data_register(
            data_reg,
            default_data);
    PN_ASSERT(default_data == output_data);

    PN_INFO("c_dtm_tb: test passed");
}

void test_data_reg_dmi_read()
{
    PN_INFO("c_dtm_tb: run test test_data_reg_dmi_read() ...");

    auto callback_dmi_write = [](uint8_t address, uint32_t data) {
        PN_ASSERT(false);
    };

    auto callback_dmi_read = [](uint8_t address) -> uint32_t {
        PN_ASSERT(address == 0x15);
        return 0xaffeaffe;
    };

    auto callback_dmi_reset = []() {
        PN_ASSERT(false);
    };

    c_dtm dtm{
        callback_dmi_write,
        callback_dmi_read,
        callback_dmi_reset};

    c_fake_jtag_adapter fake_jtag_adapter{&dtm};

    uint64_t default_data = 0;
    uint64_t input_data =
        0x15UL << 34 | // address
        1UL;           // opcode read
    uint64_t output_data = 0;

    c_dtm::e_data_reg_address data_reg = c_dtm::e_data_reg_address::DMI;

    output_data =
        fake_jtag_adapter.shift_jtag_data_register(
            data_reg,
            input_data);
    PN_ASSERT(default_data == output_data);

    output_data =
        fake_jtag_adapter.shift_jtag_data_register(
            data_reg,
            default_data);
    PN_ASSERT(0xaffeaffe == output_data >> 2U);

    PN_INFO("c_dtm_tb: test passed");
}
