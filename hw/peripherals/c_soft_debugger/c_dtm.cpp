/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Johannes Hofmann <johannes.hofmann1@tha.de>
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

#include "c_dtm.h"

#include <base.h>

namespace op_code {
static constexpr uint nop = 0;
static constexpr uint read = 1;
static constexpr uint write = 2;
static constexpr uint reserved = 3;
} // namespace op_code

c_dtm::c_dtm(
    std::function<void(uint8_t, uint32_t)> callback_dmi_write,
    std::function<uint32_t(uint8_t)> callback_dmi_read,
    std::function<void(void)> callback_dmi_reset)
    : jtag_tck{false}
    , jtag_tms{false}
    , jtag_tdi{false}
    , jtag_tdo{false}
    , callback_dmi_write{callback_dmi_write}
    , callback_dmi_read{callback_dmi_read}
    , callback_dmi_reset{callback_dmi_reset}
    , tap_state{e_tap_state::TEST_LOGIC_RESET}
    , instruction_reg{0}
    , selected_data_reg{0}
    , selected_data_reg_width{0}
{
}

c_dtm::~c_dtm()
{
}

void c_dtm::jtag_set_input_pins(jtag_input_pins_t input_pins)
{
    jtag_tck = input_pins.tck;
    jtag_tms = input_pins.tms;
    jtag_tdi = input_pins.tdi;

    _update_tap();
}

bool c_dtm::jtag_get_output_pin() const
{
    return jtag_tdo;
}

void c_dtm::jtag_reset()
{
    callback_dmi_reset();
}

size_t c_dtm::get_instruction_reg_width() const
{
    return instruction_reg_width;
}

size_t c_dtm::get_data_reg_width(e_data_reg_address data_reg_address) const
{
    switch(data_reg_address)
    {
        case e_data_reg_address::BYPASS:
            return data_reg_width_bypass;
        case e_data_reg_address::IDCODE:
            return data_reg_width_idcode;
        case e_data_reg_address::DTMCS:
            return data_reg_width_dtmcs;
        case e_data_reg_address::DMI:
            return data_reg_width_dmi;
    }

    PN_ASSERT(false);
    return 0;
}

void c_dtm::_update_tap()
{
    // clang-format off
    static constexpr e_tap_state next[16][2] = {
      /* TEST_LOGIC_RESET */    {e_tap_state::RUN_TEST_IDLE, e_tap_state::TEST_LOGIC_RESET },
      /* RUN_TEST_IDLE */       {e_tap_state::RUN_TEST_IDLE, e_tap_state::SELECT_DR_SCAN },
      /* SELECT_DR_SCAN */      {e_tap_state::CAPTURE_DR, e_tap_state::SELECT_IR_SCAN },
      /* CAPTURE_DR */          {e_tap_state::SHIFT_DR, e_tap_state::EXIT1_DR },
      /* SHIFT_DR */            {e_tap_state::SHIFT_DR, e_tap_state::EXIT1_DR },
      /* EXIT1_DR */            {e_tap_state::PAUSE_DR, e_tap_state::UPDATE_DR },
      /* PAUSE_DR */            {e_tap_state::PAUSE_DR, e_tap_state::EXIT2_DR },
      /* EXIT2_DR */            {e_tap_state::SHIFT_DR, e_tap_state::UPDATE_DR },
      /* UPDATE_DR */           {e_tap_state::RUN_TEST_IDLE, e_tap_state::SELECT_DR_SCAN },
      /* SELECT_IR_SCAN */      {e_tap_state::CAPTURE_IR, e_tap_state::TEST_LOGIC_RESET },
      /* CAPTURE_IR */          {e_tap_state::SHIFT_IR, e_tap_state::EXIT1_IR },
      /* SHIFT_IR */            {e_tap_state::SHIFT_IR, e_tap_state::EXIT1_IR },
      /* EXIT1_IR */            {e_tap_state::PAUSE_IR, e_tap_state::UPDATE_IR },
      /* PAUSE_IR */            {e_tap_state::PAUSE_IR, e_tap_state::EXIT2_IR },
      /* EXIT2_IR */            {e_tap_state::SHIFT_IR, e_tap_state::UPDATE_IR },
      /* UPDATE_IR */           {e_tap_state::RUN_TEST_IDLE, e_tap_state::SELECT_DR_SCAN }
    };
    // clang-format on

    // Check for rising edge on jtag_tck (clock)
    static bool last_jtag_tck = false;
    if(!(!last_jtag_tck && jtag_tck))
    {
        last_jtag_tck = jtag_tck;
        return;
    }
    last_jtag_tck = jtag_tck;

    switch(tap_state)
    {
        case e_tap_state::SHIFT_DR:
            _tap_shift_dr();
            break;
        case e_tap_state::SHIFT_IR:
            _tap_shift_ir();
            break;

        default:
            break;
    }

    tap_state = next[static_cast<int>(tap_state)][jtag_tms];

    switch(tap_state)
    {
        case e_tap_state::TEST_LOGIC_RESET:
            _tap_test_logic_reset();
            break;

        case e_tap_state::RUN_TEST_IDLE:
            break;

        // Data register path
        case e_tap_state::SELECT_DR_SCAN:
            break;

        case e_tap_state::CAPTURE_DR:
            _tap_capture_dr();
            break;

        case e_tap_state::SHIFT_DR:
            jtag_tdo = selected_data_reg & 1U;
            break;

        case e_tap_state::EXIT1_DR:
            break;

        case e_tap_state::PAUSE_DR:
            break;

        case e_tap_state::EXIT2_DR:
            break;

        case e_tap_state::UPDATE_DR:
            _tap_update_dr();
            break;

        // Instruction register path
        case e_tap_state::SELECT_IR_SCAN:
            break;

        case e_tap_state::CAPTURE_IR:
            break;

        case e_tap_state::SHIFT_IR:
            jtag_tdo = instruction_reg & 1U;
            break;

        case e_tap_state::EXIT1_IR:
            break;

        case e_tap_state::PAUSE_IR:
            break;

        case e_tap_state::EXIT2_IR:
            break;

        case e_tap_state::UPDATE_IR:
            break;

        default:
            break;
    }
}

void c_dtm::_tap_test_logic_reset()
{
    instruction_reg = e_data_reg_address::IDCODE;
}

void c_dtm::_tap_capture_dr()
{
    switch(instruction_reg)
    {
        case e_data_reg_address::BYPASS:
            selected_data_reg = 0;
            selected_data_reg_width = data_reg_width_bypass;
            return;
        case e_data_reg_address::IDCODE:
            selected_data_reg = 0xdeadbeef;
            selected_data_reg_width = data_reg_width_idcode;
            return;

        case e_data_reg_address::DTMCS:
            selected_data_reg = (4U << 12U) | (CFG_DEBUG_DMI_BUS_ADR_WIDTH << 4U) | 1U;
            selected_data_reg_width = data_reg_width_dtmcs;
            return;

        case e_data_reg_address::DMI:
            selected_data_reg = (uint64_t)(dmi_last_read) << 2U;
            selected_data_reg_width = data_reg_width_dmi;
            return;

        default:
            PN_WARNINGF(("c_dtm: Unsupported IR: 0x%02x", instruction_reg));
            return;
    }

    PN_ASSERT(false);
}

void c_dtm::_tap_shift_dr()
{
    // Shift right and insert tdi
    selected_data_reg = (selected_data_reg >> 1) | ((uint64_t)jtag_tdi << (selected_data_reg_width - 1));
}

void c_dtm::_tap_update_dr()
{
    if(instruction_reg == e_data_reg_address::DMI)
    {
        static const size_t address_offset = data_reg_width_dmi - CFG_DEBUG_DMI_BUS_ADR_WIDTH;

        uint8_t op = selected_data_reg & 0b11;
        uint8_t address = selected_data_reg >> address_offset;

        if(op_code::read == op)
        {
            // Read from dmi
            dmi_last_read = callback_dmi_read(address);
        }
        else if(op_code::write == op)
        {
            // Write to dmi
            uint32_t data = (selected_data_reg >> 2U) & 0xffffffff;
            callback_dmi_write(address, data);
        }
        else if(op_code::nop == op ||
                op_code::reserved == op)
        {
            // do nothing
        }
        else
        {
            PN_ASSERT(false);
        }
    }

    // Here is no need to update other registers bc they have
    // no functionality.
}

void c_dtm::_tap_shift_ir()
{
    // Shift right and insert tdi
    instruction_reg = (instruction_reg >> 1) | ((uint64_t)jtag_tdi << (instruction_reg_width - 1));
}
