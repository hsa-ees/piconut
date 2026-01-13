/**
 * @file clint_soft.h
 * @brief This file contains the definition of the m_clint_soft class.
 *        For simulation ONLY.
 * @author [Alexander Beck, Christian Zellinger]
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Alexander Beck
                2025 Christian Zellinger
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the implementation of the m_clint_soft class.
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
 * @addtogroup clint_soft
 * @brief soft peripheral implementation of the RISC-V Core Local Interruptor (CLINT) for simulation
 * @author [Alexander Beck, Christian Zellinger]
 *
 * This module implements the RISC-V Core Local Interruptor (CLINT).
 * It is responsible for generating timer and software interrupts.
 * The module is implemented as a soft peripheral and can be used in the
 * simulation environment. The module has the same registers as the RISC-V
 * CLINT specification. They are:
 * - `MSIP`: 32-bit register for machine software interrupt pending
 * - `MTIMECMP`: 64-bit register for machine timer compare (split into low/high 32-bit registers)
 * - `MTIME`: 64-bit register for machine timer (split into low/high 32-bit registers)
 *
 * The CLINT provides memory-mapped registers:
 * - MTIMECMP (Machine Timer Compare): Used to trigger timer interrupts when mtime >= mtimecmp
 * - MTIME (Machine Timer): Increments continuously with the system clock
 * - MSIP (Machine Software Interrupt Pending): Used to trigger software interrupts
 *
 * The module has a 32-bit memory interface and can be accessed by the
 * soft peripheral interface.
 *
 * Note: The module is not synthesizable and is only used for simulation purposes.
 *
 */

#ifndef __CLINT_SOFT_H__
#define __CLINT_SOFT_H__

#include "c_soft_peripheral.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

class m_clint_soft : public c_soft_peripheral
{
public:
    /**
     * @brief The available registers in the CLINT
     */
    enum class e_regs : uint64_t
    {
        // MSIP registers (per hart)
        MSIP = 0x0, // Machine Software Interrupt Pending

        // MTIMECMP registers (per hart)
        MTIMECMP_LOW = 0x4000,  // Machine Time Compare Lower 32 bits
        MTIMECMP_HIGH = 0x4004, // Machine Time Compare Upper 32 bits

        // MTIME registers (shared)
        MTIME_LOW = 0xBFF8, // Machine Time Lower 32 bits
        MTIME_HIGH = 0xBFFC // Machine Time Upper 32 bits
    };

    /**
     * @brief Constructor
     *
     * @param callback_signal_sw_interrupt Callback function to trigger software interrupt
     * @param callback_signal_timer_interrupt Callback function to trigger timer interrupt
     */
    m_clint_soft(
        std::function<void(bool)> callback_signal_sw_interrupt = nullptr,
        std::function<void(bool)> callback_signal_timer_interrupt = nullptr);

    /**
     * @brief Destructor
     */
    ~m_clint_soft() = default;

    /**
     * @brief Get the peripheral info.
     * @return The peripheral info.
     */
    const char* get_info() override;

    /**
     * @brief Check if the address is within this peripheral's range.
     * @param adr The address to check.
     * @return True if the address is within this peripheral's range.
     */
    bool is_addressed(uint64_t adr) override;

    /**
     * @brief Read a 32-bit value from a register.
     * @param adr The address of the register to read.
     * @return The 32-bit value read from the register.
     */
    uint32_t read32(uint64_t adr) override;

    /**
     * @brief Write a 32-bit value to a register.
     * @param adr The address of the register to write.
     * @param data The 32-bit value to write to the register.
     */
    void write32(uint64_t adr, uint32_t data) override;

    /**
     * @brief Read a single byte (8-bit) value from a register.
     * @param adr The address of the byte to read.
     * @return The 8-bit value read from the register.
     */
    uint8_t read8(uint64_t adr) override;

    /**
     * @brief Write a single byte (8-bit) value to a register.
     * @param adr The address of the byte to write.
     * @param data The 8-bit value to write to the register.
     */
    void write8(uint64_t adr, uint8_t data) override;

    /**
     * @brief Write a 64-bit value to a register with optimized CLINT-specific ordering.
     * @param adr The address of the register to write.
     * @param data The 64-bit value to write to the register.
     *
     * This override ensures correct HIGH->LOW write order for CLINT registers
     * to avoid race conditions during 64-bit timer register updates.
     */
    void write64(uint64_t adr, uint64_t data) override;

    /**
     * @brief Updates the timer and checks for interrupts
     *        This function is automatically called on every rising edge of the clock
     */
    void on_rising_edge_clock() override;

    /**
     * @brief Register a callback function for software interrupt changes.
     *
     * @param callback The callback function
     */
    void register_msip_callback(std::function<void(bool)> callback);

    /**
     * @brief Register a callback function for timer interrupt changes.
     *
     * @param callback The callback function
     */
    void register_mtip_callback(std::function<void(bool)> callback);

private:
    // Module name
    static const char* MODULE_NAME;

    // Internal peripheral name buffer
    char peripheral_name[32];

    // Registers
    uint32_t msip;     // Software interrupt pending register
    uint64_t mtimecmp; // Timer compare register
    uint64_t mtime;    // Timer register

    bool mtimecmp_high_written;  // Flag to track if MTIMECMP_HIGH was written
    bool mtime_high_written;     // Flag to track if MTIME_HIGH was written
    bool mtime_update_suspended; // Flag to indicate if MTIME updates are suspended during writes

    // Timer control
    bool mtip_pending;                   // Timer interrupt pending flag
    bool timer_interrupt_update_pending; // Flag for pending timer interrupt updates

    // Callback functions
    std::function<void(bool)> callback_signal_sw_interrupt;
    std::function<void(bool)> callback_signal_timer_interrupt;

    /**
     * @brief Update the timer interrupt based on mtime and mtimecmp values.
     */
    void update_timer_interrupt();
};

#endif // __CLINT_SOFT_H__