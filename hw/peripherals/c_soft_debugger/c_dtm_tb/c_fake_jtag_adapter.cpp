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

#include "c_fake_jtag_adapter.h"

#include <base.h>

namespace {

void add_jtag_inputs_rising_edge(
    std::queue<jtag_input_pins_t>& sequence,
    bool tms,
    bool tdi)
{
    sequence.emplace(jtag_input_pins_t{false, tms, tdi});
    sequence.emplace(jtag_input_pins_t{true, tms, tdi});
}

} // namespace

c_fake_jtag_adapter::c_fake_jtag_adapter(c_dtm* dtm)
    : dtm{dtm}
{
    PN_ASSERT(nullptr != dtm);

    reset_tap_machine();
}

c_fake_jtag_adapter::~c_fake_jtag_adapter()
{
}

void c_fake_jtag_adapter::reset_tap_machine()
{
    std::queue<jtag_input_pins_t> jtag_input_pins_sequence;
    add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 1, 0);
    add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 1, 0);
    add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 1, 0);
    add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 1, 0);
    add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 1, 0);
    _apply_jtag_input_pins_sequence(jtag_input_pins_sequence);

    // TAP should be in TEST_LOGIC_RESET state here
}

uint64_t c_fake_jtag_adapter::shift_jtag_instruction_register(
    uint64_t input_instruction)
{
    std::queue<jtag_input_pins_t> jtag_input_pins_sequence;

    // Setup
    add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 0, 0); // to RUN_TEST_IDLE
    add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 1, 0); // to SELECT_DR_SCAN
    add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 1, 0); // to SELECT_IR_SCAN
    add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 0, 0); // to CAPTURE_IR
    add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 0, 0); // to SHIFT_IR
    _apply_jtag_input_pins_sequence(jtag_input_pins_sequence);

    // Shift
    size_t instruction_reg_width = dtm->get_instruction_reg_width();
    for(size_t index = 0; index < instruction_reg_width; ++index)
    {
        bool data_lsb = (input_instruction >> index) & 0b1U;
        // Last shift leaves shift state
        if(index != instruction_reg_width - 1)
        {
            add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 0, data_lsb);
        }
        else
        {
            add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 1, data_lsb); // to EXIT_IR
        }
    }

    uint64_t output_instruction =
        _apply_jtag_input_pins_sequence(jtag_input_pins_sequence);

    // Cleanup
    add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 1, 0); // to UPDATE_IR
    add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 0, 0); // to RUN_TEST_IDLE
    _apply_jtag_input_pins_sequence(jtag_input_pins_sequence);

    return output_instruction;
}

uint64_t c_fake_jtag_adapter::shift_jtag_data_register(
    c_dtm::e_data_reg_address data_reg,
    uint64_t input_data)
{
    shift_jtag_instruction_register(data_reg);

    // TAP should be in run test idle state here

    std::queue<jtag_input_pins_t> jtag_input_pins_sequence;

    // Setup
    add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 1, 0); // to SELECT_DR_SCAN
    add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 0, 0); // to CAPTURE_DR
    add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 0, 0); // to SHIFT_DR
    _apply_jtag_input_pins_sequence(jtag_input_pins_sequence);

    // Shift
    size_t data_reg_width = dtm->get_data_reg_width(data_reg);
    for(size_t index = 0; index < data_reg_width; ++index)
    {
        bool data_lsb = (input_data >> index) & 0b1U;
        // Last shift leaves shift state
        if(index != data_reg_width - 1)
        {
            add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 0, data_lsb);
        }
        else
        {
            add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 1, data_lsb); // to EXIT_DR
        }
    }

    uint64_t output_data =
        _apply_jtag_input_pins_sequence(jtag_input_pins_sequence);

    // Cleanup
    add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 1, 0); // to UPDATE_DR
    add_jtag_inputs_rising_edge(jtag_input_pins_sequence, 0, 0); // to RUN_TEST_IDLE
    _apply_jtag_input_pins_sequence(jtag_input_pins_sequence);

    return output_data;
}

uint64_t c_fake_jtag_adapter::_apply_jtag_input_pins_sequence(
    std::queue<jtag_input_pins_t>& jtag_input_pins_sequence)
{
    PN_ASSERT(jtag_input_pins_sequence.size() < 64 * 2);

    uint64_t output = 0;
    size_t output_index = 0;

    while(!jtag_input_pins_sequence.empty())
    {
        const auto jtag_input_pins = jtag_input_pins_sequence.front();
        jtag_input_pins_sequence.pop();

        if(jtag_input_pins.tck == 1)
        {
            output |= (uint64_t)dtm->jtag_get_output_pin() << output_index;
            ++output_index;
        }

        dtm->jtag_set_input_pins(jtag_input_pins);
    }

    return output;
}