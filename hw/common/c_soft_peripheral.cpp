/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Marco Milenkovic <Marco.Milenkovic@hs-augsburg.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: This file contains the implementation of the Software Interface Class for custom peripherals

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

#include "c_soft_peripheral.h"

const char *c_soft_peripheral::get_info()
{
    return "You should implement this function in the derived class!";
}

uint8_t c_soft_peripheral::read8_using_read32(uint64_t adr)
{
    uint32_t data = read32(adr);
    return (data >> ((adr & 3) * 8)) & 0xFF; // get the byte from the 32bit word
}

uint16_t c_soft_peripheral::read16_using_read8(uint64_t adr)
{
    uint16_t data = 0;
    for (int i = 0; i < 2; i++)
    {
        data |= read8(adr + i) << (i * 8);
    }
    return data;
}

uint32_t c_soft_peripheral::read32_using_read8(uint64_t adr)
{
    uint32_t data = 0;
    for (int i = 0; i < 4; i++)
    {
        data |= read8(adr + i) << (i * 8);
    }
    return data;
}

uint64_t c_soft_peripheral::read64_using_read32(uint64_t adr)
{
    uint64_t data = 0;
    for (int i = 0; i < 2; i++)
    {
        data |= read32(adr + i * 4) << (i * 32);
    }
    return data;
}

void c_soft_peripheral::write8_using_write32(uint64_t adr, uint8_t data)
{
    uint32_t old_data = read32(adr & ~3);
    old_data &= ~(0xFF << ((adr & 3) * 8)); // clear the byte
    old_data |= data << ((adr & 3) * 8);    // set the byte
    write32(adr & ~3, old_data);
}

void c_soft_peripheral::write16_using_write8(uint64_t adr, uint16_t data)
{
    for (int i = 0; i < 2; i++)
    {
        write8(adr + i, (data >> (i * 8)) & 0xFF);
    }
}

void c_soft_peripheral::write32_using_write8(uint64_t adr, uint32_t data)
{
    for (int i = 0; i < 4; i++)
    {
        write8(adr + i, (data >> (i * 8)) & 0xFF);
    }
}

void c_soft_peripheral::write64_using_write32(uint64_t adr, uint64_t data)
{
    for (int i = 0; i < 2; i++)
    { // write 32bit words to the peripheral (little endian)
        write32(adr + i * 4, (data >> (i * 32)) & 0xFFFFFFFF);
    }
}