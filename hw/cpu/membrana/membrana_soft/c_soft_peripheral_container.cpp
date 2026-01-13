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

#include "c_soft_peripheral_container.h"
#include <iostream>

void c_soft_peripheral_container::add_peripheral(uint64_t base_adr, std::unique_ptr<c_soft_peripheral> peripheral)
{
    if(has_peripheral_at(base_adr))
    {
        c_soft_peripheral* found_peripheral = find_peripheral(base_adr);
        // Check if 'find_peripheral' returned a valid peripheral
        if(found_peripheral != nullptr)
        {
            // Perform the overlap check
            if(found_peripheral->base_address <= base_adr && base_adr < found_peripheral->base_address + found_peripheral->size)
            {
                PN_ERRORF(("Peripheral at base address 0x%llx overlaps with an existing peripheral.\n", (unsigned long long)base_adr));
                return; // Exit the function to avoid adding overlapping peripherals
            }
        }
    }

    peripherals[base_adr] = std::move(peripheral);
}

bool c_soft_peripheral_container::has_peripheral_at(uint64_t address)
{
    for(const auto& [base_addr, peripheral] : peripherals)
    {
        // Check if the address falls within the peripheral's range
        if(peripheral->is_addressed(address))
        {
            return true;
        }
    }
    return false;
}

c_soft_peripheral* c_soft_peripheral_container::find_peripheral(uint64_t address)
{
    // Iterate through the map to find the peripheral
    for(const auto& [base_addr, peripheral] : peripherals)
    {
        // Check if the address falls within the peripheral's range
        if(peripheral->is_addressed(address))
        {
            return peripheral.get();
        }
    }
    PN_ERRORF(("No peripheral found for the given address 0x%llx\n", (unsigned long long)address));
    return nullptr;
}

void c_soft_peripheral_container::write_peripheral(uint64_t address, uint32_t data, uint8_t bsel)
{
    c_soft_peripheral* peripheral = find_peripheral(address);
    // write data shifted by 8 bits for each active byte select. Mask with 0xFF to get the lower 8 bits
    if(peripheral)
    {
        if(bsel & 0x1)
            peripheral->write8(address, data & 0xFF);
        if(bsel & 0x2)
            peripheral->write8(address + 1, (data >> 8) & 0xFF);
        if(bsel & 0x4)
            peripheral->write8(address + 2, (data >> 16) & 0xFF);
        if(bsel & 0x8)
            peripheral->write8(address + 3, (data >> 24) & 0xFF);
    }
    else
    {
        PN_ERRORF(("No peripheral found for write address 0x%llx\n", (unsigned long long)address));
    }
}

void c_soft_peripheral_container::write8_peripheral(uint64_t address, uint32_t data, uint8_t bsel)
{
    // Just call write_peripheral with the given byte select because they are the same
    write_peripheral(address, data, bsel);
}

void c_soft_peripheral_container::write16_peripheral(uint64_t address, uint32_t data, uint8_t bsel)
{
    c_soft_peripheral* peripheral = find_peripheral(address);
    // write data shifted by 16 bits for each active byte select. Mask with 0xFFFF to get the lower 16 bits
    if(peripheral)
    {
        if(bsel == 0x3)
            peripheral->write16(address, data & 0xFFFF);
        else if(bsel == 0xC)
            peripheral->write16(address + 2, (data >> 16) & 0xFFFF);
        else
            PN_ERRORF(("Invalid bsel for 16-bit write at address 0x%llx\n", (unsigned long long)address));
    }
    else
    {
        PN_ERRORF(("No peripheral found for write address 0x%llx\n", (unsigned long long)address));
    }
}

void c_soft_peripheral_container::write32_peripheral(uint64_t address, uint32_t data)
{
    c_soft_peripheral* peripheral = find_peripheral(address);
    if(peripheral)
    {
        peripheral->write32(address, data);
    }
    else
    {
        PN_ERRORF(("No peripheral found for write address 0x%llx\n", (unsigned long long)address));
    }
}

uint32_t c_soft_peripheral_container::read_peripheral(uint64_t address, uint8_t bsel)
{
    c_soft_peripheral* peripheral = find_peripheral(address);
    if(peripheral)
    {
        // read data from each active byte select and combine them without overwriting
        uint32_t data = 0;
        switch(bsel)
        {
            case 0b0001: // 8-bit read
                data = peripheral->read8(address);
                break;
            case 0b0010:
                data = peripheral->read8(address + 1) << 8;
                break;
            case 0b0100:
                data = peripheral->read8(address + 2) << 16;
                break;
            case 0b1000:
                data = peripheral->read8(address + 3) << 24;
                break;
            case 0b0011: // 16-bit read
                data = peripheral->read16(address);
                break;
            case 0b1100:
                data = peripheral->read16(address + 2) << 16;
                break;
            case 0b1111: // 32-bit read
                data = peripheral->read32(address);
                break;
        }
        return data;
    }
    else
    {
        PN_ERRORF(("No peripheral found for read address 0x%llx\n", (unsigned long long)address));
        return 0;
    }
}

uint32_t c_soft_peripheral_container::read8_peripheral(uint64_t address, uint8_t bsel)
{
    // Just call read_peripheral with the given byte select because they are the same
    return read_peripheral(address, bsel);
}

uint32_t c_soft_peripheral_container::read16_peripheral(uint64_t address, uint8_t bsel)
{
    c_soft_peripheral* peripheral = find_peripheral(address);
    if(peripheral)
    {
        uint32_t data = 0;
        if(bsel == 0x3)
            data |= peripheral->read16(address);
        else if(bsel == 0xC)
            data |= peripheral->read16(address + 2) << 16;
        else
            PN_ERRORF(("Invalid bsel for 16-bit read at address 0x%llx\n", (unsigned long long)address));
        return data;
    }
    else
    {
        PN_ERRORF(("No peripheral found for read address 0x%llx\n", (unsigned long long)address));
        return 0;
    }
}

uint32_t c_soft_peripheral_container::read32_peripheral(uint64_t address)
{
    c_soft_peripheral* peripheral = find_peripheral(address);
    if(peripheral)
    {
        return peripheral->read32(address);
    }
    else
    {
        PN_ERRORF(("No peripheral found for read address 0x%llx\n", (unsigned long long)address));
        return 0;
    }
}

void c_soft_peripheral_container::list_all_peripherals()
{
    std::string info = "\n";
    for(const auto& [base_addr, peripheral] : peripherals)
    {
        char buf[128];
        snprintf(buf, sizeof(buf), "Peripheral at base address 0x%llX:\n", (uint32_t)base_addr);
        info += buf;
        info += "Info: ";
        info += peripheral->get_info();
        info += "\n";
    }
    info.pop_back();
    if(!info.empty() && info.back() == '\n')
        info.pop_back();
    PN_INFOF(("%s", info.c_str()));
    PN_INFO(""); // close the info block
}

int c_soft_peripheral_container::get_num_peripherals()
{
    return peripherals.size();
}

c_soft_peripheral* c_soft_peripheral_container::get_peripheral(int index)
{
    if(index < peripherals.size())
    {
        auto it = peripherals.begin();
        std::advance(it, index);
        return it->second.get();
    }
    else
    {
        PN_ERRORF(("No peripheral found for index %d\n", index));
        return nullptr;
    }
}

void c_soft_peripheral_container::on_rising_edge_clock_all_peripherals()
{
    for(const auto& [base_addr, peripheral] : peripherals)
    {
        peripheral->on_rising_edge_clock();
    }
}
