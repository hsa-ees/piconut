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

#include "c_peripheral_interface.h"
#include <iostream>

void c_soft_peripheral_interface::add_peripheral(uint64_t base_adr, std::unique_ptr<c_soft_peripheral> peripheral)
{
    c_soft_peripheral *test_peripheral = find_peripheral(base_adr);

    // Check if 'find_peripheral' returned a valid peripheral
    if (test_peripheral != nullptr)
    {
        // Perform the overlap check
        if (test_peripheral->base_address <= base_adr && base_adr < test_peripheral->base_address + test_peripheral->size)
        {
            std::cerr << "Error: Peripheral at base address 0x" << std::hex << base_adr << " overlaps with an existing peripheral." << std::endl;
            return; // Exit the function to avoid adding overlapping peripherals
        }
    }

    // If no overlap, add the new peripheral
    peripherals[base_adr] = std::move(peripheral);
    std::cout << "Peripheral successfully added at base address 0x" << std::hex << base_adr << std::endl;
}

c_soft_peripheral *c_soft_peripheral_interface::find_peripheral(uint64_t address)
{
    // Iterate through the map to find the peripheral
    for (const auto &[base_addr, peripheral] : peripherals)
    {
        // Check if the address falls within the peripheral's range
        if (peripheral->is_addressed(address))
        {
            return peripheral.get();
        }
    }
    std::cerr << "No peripheral found for the given address 0x" << std::hex << address << std::endl;
    return nullptr;
}

void c_soft_peripheral_interface::write_peripheral(uint64_t address, uint32_t data, uint8_t bsel)
{
    c_soft_peripheral *peripheral = find_peripheral(address);
    // write data shifted by 8 bits for each active byte select. Mask with 0xFF to get the lower 8 bits
    if (peripheral)
    {
        if (bsel & 0x1)
            peripheral->write8(address, data & 0xFF);
        if (bsel & 0x2)
            peripheral->write8(address + 1, (data >> 8) & 0xFF);
        if (bsel & 0x4)
            peripheral->write8(address + 2, (data >> 16) & 0xFF);
        if (bsel & 0x8)
            peripheral->write8(address + 3, (data >> 24) & 0xFF);
    }
    else
    {
        std::cerr << "No peripheral found for write address 0x" << std::hex << address << std::endl;
    }
}

void c_soft_peripheral_interface::write8_peripheral(uint64_t address, uint32_t data, uint8_t bsel)
{
    // Just call write_peripheral with the given byte select because they are the same
    write_peripheral(address, data, bsel);
}

void c_soft_peripheral_interface::write16_peripheral(uint64_t address, uint32_t data, uint8_t bsel)
{
    c_soft_peripheral *peripheral = find_peripheral(address);
    // write data shifted by 16 bits for each active byte select. Mask with 0xFFFF to get the lower 16 bits
    if (peripheral)
    {
        if (bsel == 0x3)
            peripheral->write16(address, data & 0xFFFF);
        else if (bsel == 0xC)
            peripheral->write16(address + 2, (data >> 16) & 0xFFFF);
        else
            std::cerr << "Invalid bsel for 16-bit write at address 0x" << std::hex << address << std::endl;
    }
    else
    {
        std::cerr << "No peripheral found for write address 0x" << std::hex << address << std::endl;
    }
}

void c_soft_peripheral_interface::write32_peripheral(uint64_t address, uint32_t data)
{
    c_soft_peripheral *peripheral = find_peripheral(address);
    if (peripheral)
    {
        peripheral->write32(address, data);
    }
    else
    {
        std::cerr << "No peripheral found for write address 0x" << std::hex << address << std::endl;
    }
}

uint32_t c_soft_peripheral_interface::read_peripheral(uint64_t address, uint8_t bsel)
{
    c_soft_peripheral *peripheral = find_peripheral(address);
    if (peripheral)
    {
        // read data from each active byte select and combine them without overwriting
        uint32_t data = 0;
        if (bsel & 0x1)
            data |= peripheral->read8(address);
        if (bsel & 0x2)
            data |= peripheral->read8(address + 1) << 8;
        if (bsel & 0x4)
            data |= peripheral->read8(address + 2) << 16;
        if (bsel & 0x8)
            data |= peripheral->read8(address + 3) << 24;
        return data;
    }
    else
    {
        std::cerr << "No peripheral found for read address 0x" << std::hex << address << std::endl;
        return 0;
    }
}

uint32_t c_soft_peripheral_interface::read8_peripheral(uint64_t address, uint8_t bsel)
{
    // Just call read_peripheral with the given byte select because they are the same
    return read_peripheral(address, bsel);
}

uint32_t c_soft_peripheral_interface::read16_peripheral(uint64_t address, uint8_t bsel)
{
    c_soft_peripheral *peripheral = find_peripheral(address);
    if (peripheral)
    {
        uint32_t data = 0;
        if (bsel == 0x3)
            data |= peripheral->read16(address);
        else if (bsel == 0xC)
            data |= peripheral->read16(address + 2) << 16;
        else
            std::cerr << "Invalid bsel for 16-bit read at address 0x" << std::hex << address << std::endl;
        return data;
    }
    else
    {
        std::cerr << "No peripheral found for read address 0x" << std::hex << address << std::endl;
        return 0;
    }
}

uint32_t c_soft_peripheral_interface::read32_peripheral(uint64_t address)
{
    c_soft_peripheral *peripheral = find_peripheral(address);
    if (peripheral)
    {
        return peripheral->read32(address);
    }
    else
    {
        std::cerr << "No peripheral found for read address 0x" << std::hex << address << std::endl;
        return 0;
    }
}

void c_soft_peripheral_interface::list_all_peripherals()
{
    for (const auto &[base_addr, peripheral] : peripherals)
    {
        std::cout << "Peripheral at base address 0x" << std::hex << base_addr << ": Info: " << peripheral->get_info() << std::endl;
    }
}

int c_soft_peripheral_interface::get_num_peripherals()
{
    return peripherals.size();
}

c_soft_peripheral *c_soft_peripheral_interface::get_peripheral(int index)
{
    if (index < peripherals.size())
    {
        auto it = peripherals.begin();
        std::advance(it, index);
        return it->second.get();
    }
    else
    {
        std::cerr << "No peripheral found for index " << index << std::endl;
        return nullptr;
    }
}
