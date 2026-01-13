/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Marco Milenkovic <Marco.Milenkovic@hs-augsburg.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: This file contains the interface to manage and access peripherals in the system

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

#ifndef __PERIPHERAL_INTERFACE_H__
#define __PERIPHERAL_INTERFACE_H__

#include <piconut.h>
#include <unordered_map>
#include <memory>
#include "c_soft_peripheral.h" // This should be the base class for your peripherals, assumed to be CSoftPeripheral

class c_soft_peripheral_container
{
private:
    // Map with address as key to store peripherals as unique pointers
    std::unordered_map<uint64_t, std::unique_ptr<c_soft_peripheral>> peripherals;

public:
    /**
     * @brief Check if a peripheral exists at the given base address
     * @param address c_soft_memory address to search for
     */
    bool has_peripheral_at(uint64_t address);

    /** @brief Add a peripheral to the system
     *
     * @param base_adr Base address of the peripheral
     * @param peripheral Pointer to the peripheral object
     */
    void add_peripheral(uint64_t base_adr, std::unique_ptr<c_soft_peripheral> peripheral);

    /** @brief Find a peripheral by its memory address
     *
     * @param address c_soft_memory address to search for
     * @return Pointer to the found peripheral, or nullptr if not found
     */
    c_soft_peripheral* find_peripheral(uint64_t address);

    /** @brief Write data to a peripheral using byte select
     *
     * @param address Address at which to write
     * @param data Data to write
     * @param bsel Byte select
     */
    void write_peripheral(uint64_t address, uint32_t data, uint8_t bsel);

    /** @brief Write 8-bit data to a peripheral
     *
     * @param address Address at which to write
     * @param data Data to write
     * @param bsel Byte select
     */
    void write8_peripheral(uint64_t address, uint32_t data, uint8_t bsel);

    /** @brief Write 16-bit data to a peripheral
     *
     * @param address Address at which to write
     * @param data Data to write
     * @param bsel Byte select
     */
    void write16_peripheral(uint64_t address, uint32_t data, uint8_t bsel);

    /** @brief Write 32-bit data to a peripheral
     *
     * @param address Address at which to write
     * @param data Data to write
     * @param bsel Byte select
     */
    void write32_peripheral(uint64_t address, uint32_t data);

    /** @brief Read data from a peripheral
     *
     * @param address Address from which to read
     * @return Data read from the peripheral
     */
    uint32_t read_peripheral(uint64_t address, uint8_t bsel);

    /** @brief Read 8-bit data from a peripheral using bsel
     *
     * @param address Address from which to read
     * @return Data read from the peripheral
     * @param bsel Byte select
     */
    uint32_t read8_peripheral(uint64_t address, uint8_t bsel);

    /** @brief Read 16-bit data from a peripheral
     *
     * @param address Address from which to read
     * @return Data read from the peripheral
     */
    uint32_t read16_peripheral(uint64_t address, uint8_t bsel);

    /** @brief Read 32-bit data from a peripheral
     *
     * @param address Address from which to read
     * @return Data read from the peripheral
     */
    uint32_t read32_peripheral(uint64_t address);

    /** @brief List all peripherals and their information
     */
    void list_all_peripherals();

    /** @brief return the number of peripherals
     */
    int get_num_peripherals();

    /**
     * @brief Get the peripheral object by number
     *
     * @param index
     * @return const c_soft_peripheral*
     */
    c_soft_peripheral* get_peripheral(int index);

    /**
     * @brief Updates all peripherals on each clock cycle
     *
     * This function iterates through all registered peripherals in the container and calls
     * their individual on_rising_edge_clock() methods. It simulates the hardware behavior where
     * peripherals update their internal state on the rising edge of the system clock.
     *
     * This function is typically called from the main system clock process (proc_clk())
     * in m_membrana_soft.
     *
     * @see c_soft_peripheral::on_rising_edge_clock()
     */
    void on_rising_edge_clock_all_peripherals();
};

#endif // __PERIPHERAL_INTERFACE_H__
