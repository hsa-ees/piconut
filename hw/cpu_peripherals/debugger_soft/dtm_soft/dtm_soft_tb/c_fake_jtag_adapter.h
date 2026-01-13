/**
 * @file c_fake_jtag_adapter.h
 * @brief This file contains the definition of the c_fake_jtag_adapter class.
 *        For simulation ONLY.
 * @author Johannes Hofmann
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Johannes Hofmann <johannes.hofmann@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the implementation of the c_fake_jtag_adapter class.
      For simulation ONLY.

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

/**
 * This module is a simple socket client that connects to a socket on localhost
 * to send and receive data.
 *
 * Note: The module is not synthesizable and is only used for simulation purposes.
 *
 */

#ifndef __C_FAKE_JTAG_H__
#define __C_FAKE_JTAG_H__

#include "../dtm_soft.h"

#include <cstdint>
#include <queue>

class c_fake_jtag_adapter
{
public:
    /**
     * @brief Constructor.
     *
     * @param dtm Dtm module.
     */
    c_fake_jtag_adapter(c_dtm* dtm);

    /**
     * @brief Destructor.
     */
    ~c_fake_jtag_adapter();

    /**
     * @brief Performs necassary actions to bring dtm's tap machine in state
     *        TEST_LOGIC_RESET.
     */
    void reset_tap_machine();

    /**
     * @brief Shifts the instruction register of the dtm until it is completly
     *        replaced.
     *
     * @param input_instruction The instruction that is shifted into the
     *        instruction register.
     * @return Output of shift register
     */
    uint64_t shift_jtag_instruction_register(uint64_t input_instruction);

    /**
     * @brief Shifts the selected data register of the dtm until it is completly
     *        replaced.
     *
     * @param data_reg Select which data register should be shifted.
     * @param input_data The data that is shifted into the selected data register.
     * @return Output of shift register
     */
    uint64_t shift_jtag_data_register(
        c_dtm::e_data_reg_address data_reg,
        uint64_t input_data);

private:
    uint64_t _apply_jtag_input_pins_sequence(
        std::queue<jtag_input_pins_t>& jtag_input_pins_sequence);

private:
    c_dtm* dtm;
};

#endif