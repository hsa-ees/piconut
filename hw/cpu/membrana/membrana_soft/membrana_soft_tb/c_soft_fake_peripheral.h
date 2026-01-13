/**
 * @file c_soft_timer.h
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Christian Zellinger <Christian.Zellinger1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: This file contains the implementation of the c_soft_fake_peripheral for simulation ONLY

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
 * @addtogroup c_soft_fake
 * @brief Soft peripheral implementation fake to test the membrana_soft
 * @author Christian Zellinger
 *
 * This module is used test the on_rising_edge_clock functionality of the membrana_soft
 * The module is implemented as a soft peripheral and can be used in the
 * simulation environment. The module has one register
 * They are:
 * - `output`: 32-bit register - incremented every time the function on_rising_edge_clock is called
 *
 * The module has a 32-bit memory interface and can be accessed by the
 * soft peripheral interface.
 *
 * Note: The module is not synthesizable and is only used for simulation purposes.
 *
 */

#ifndef __SOFTtimer_H__
#define __SOFTtimer_H__

#include "c_soft_peripheral.h"
#include <piconut.h>

#include <cstdint> // For fixed width integer types

class c_soft_fake_peripheral : public c_soft_peripheral
{
private:
    // Registers
    struct regs_t
    {
        uint32_t on_rising_edge_counter = 0;    // increment every time the function on_rising_edge_clock is called
    };


public:
    // Register address offsets
    enum e_soft_regs
    {
        COUNTER             = 0x00,
    };

    static constexpr uint64_t   PERIPHERAL_SIZE = sizeof(regs_t);
    static constexpr uint8_t    REGISTER_BIT_WIDTH = 32;


    /**
     * @brief Constructor
     *
     * @param size address space of the peripheral
     * @param base_address base address of the peripheral in address space of the simulation
     */
    c_soft_fake_peripheral(uint64_t size, uint64_t base_address);

    /**
     * @brief Destructor
     */
    ~c_soft_fake_peripheral();

    /**
     * @brief Performs all internal functions depending on the system clock. In this case, increments the output counter register
     *
     * This function is automatically called at the rising edge of every simulated clock cycle
     */
    void on_rising_edge_clock() override ;


    // c_soft_peripheral
    const char* get_info() override;
    bool is_addressed(uint64_t adr) override;

    uint8_t read8(uint64_t adr) override;
    void write8(uint64_t adr, uint8_t data) override;

    uint32_t read32(uint64_t adr) override;
    void write32(uint64_t adr, uint32_t data) override;

private:

    // c_soft_peripheral
    char name[32] = "c_soft_fake";
    regs_t registers = {0};
    uint64_t size;
    uint64_t base_address;

};

#endif