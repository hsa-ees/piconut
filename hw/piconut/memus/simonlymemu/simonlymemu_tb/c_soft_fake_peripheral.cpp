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

#include "c_soft_fake_peripheral.h"

#include <stdio.h> // For snprintf
#include <cstring> // For memcpy, if needed
#include <cstdlib>
#include <fcntl.h>


#include <unistd.h>

c_soft_fake_peripheral::c_soft_fake_peripheral(uint64_t size, uint64_t base_address)
    : size{size}
    , registers{registers}
    , base_address{base_address}
{
    fflush(stdout);
}

c_soft_fake_peripheral::~c_soft_fake_peripheral()
{

}


void c_soft_fake_peripheral::on_rising_edge_clock()
{
    registers.on_rising_edge_counter++;
}



const char* c_soft_fake_peripheral::get_info()
{
    static char info[128]; // Static to avoid memory management issues
    snprintf(info, sizeof(info), "Name: %s,\nBase Address: 0x%llX ,\nSize: %d B\n", name, base_address, size);
    return info;
}

bool c_soft_fake_peripheral::is_addressed(uint64_t adr)
{
    return adr >= base_address && adr < size + base_address;
}

uint8_t c_soft_fake_peripheral::read8(uint64_t adr)
{
    return c_soft_peripheral::read8_using_read32(adr);
}

void c_soft_fake_peripheral::write8(uint64_t adr, uint8_t data)
{
    c_soft_peripheral::write8_using_write32(adr, data);
}

uint32_t c_soft_fake_peripheral::read32(uint64_t adr)
{
    //std::clog << "read32" << endl;
    uint8_t internal_address = adr - this->base_address;
    uint32_t data_value = 0;

    // Read only from start of register
    internal_address &= ~3;

    switch((c_soft_fake_peripheral::e_soft_regs) internal_address)
    {
        case c_soft_fake_peripheral::e_soft_regs::COUNTER:
            data_value = registers.on_rising_edge_counter;
            break;

        default:
            PN_ASSERT(false);
            break;
    }

    return data_value;
}

void c_soft_fake_peripheral::write32(uint64_t adr, uint32_t data)
{
    uint8_t internal_address = adr - this->base_address;

    // Write only from start of register
    internal_address &= ~3;

    switch((c_soft_fake_peripheral::e_soft_regs) internal_address)
    {
        case c_soft_fake_peripheral::e_soft_regs::COUNTER:
            registers.on_rising_edge_counter = data;
            break;

        default:
            PN_ASSERT(false);
            break;
    }

}



