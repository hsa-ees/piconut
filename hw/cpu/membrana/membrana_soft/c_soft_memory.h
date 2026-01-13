/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Marco Milenkovic <Marco.Milenkovic@hs-augsburg.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: This file contains the implementation of the c_soft_memory for simulation ONLY

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

#ifndef _MEMORY_
#define _MEMORY_

#include "c_soft_peripheral.h"

#include <vector>
#include <stdio.h> // For snprintf
#include <cstdint> // For fixed width integer types
#include <fstream>
#include <iomanip>
#include <iostream>

class c_soft_memory : public c_soft_peripheral
{
public:
    char name[32] = "Memory";
    std::vector<uint8_t> memory;
    uint64_t size;
    uint64_t base_address; // Starting address of memory

    /**
     * @brief Construct a new c_soft_memory object
     *
     * @param size Memory size in bytes
     * @param baseAdress base address of the memory
     */
    c_soft_memory(uint64_t size, uint64_t baseAdress)
        : size(size)
        , memory(size)
        , base_address(baseAdress)
    {
        // The memory vector is initialized to the specified size with zero-initialized values
    }

    /**
     * @brief Destroy the c_soft_memory object
     *
     */
    ~c_soft_memory() {}

    /**
     * @brief Get the information about the memory peripheral
     *
     * @return const char* with Name, Base Address and Size of the memory
     */
    const char* get_info() override
    {
        static char info[128]; // Static to avoid memory management issues
        snprintf(info, sizeof(info), "Name: %s, Size: %d B", name, size);
        return info;
    }

    /**
     * @brief returns a byte from the memory at the specified address
     *
     * @param adr address to read from
     * @return uint8_t
     */
    uint8_t read8(uint64_t adr) override
    {
        if(adr < base_address || adr > size + base_address)
        {
            // Handle out-of-range adr
            std::cerr << "Last handled address is: " << std::hex << adr << std::endl;
            throw std::runtime_error("Error: Address out of range in read8");
        }
        else
        {
            return memory[adr - base_address];
        }
    }

    /**
     * @brief write a byte to the memory at the specified address
     *
     * @param adr address to write to
     * @param data contains the byte to write
     */
    void write8(uint64_t adr, uint8_t data) override
    {
        // std::cout << "write32 called: Address = 0x" << std::hex << adr << ", Data = 0x" << data << std::endl;
        if(adr < base_address || adr > size + base_address)
        {
            // TODO:: Handle out-of-range adr
            std::cerr << "Last handled address is: " << std::hex << adr << std::endl;
            throw std::runtime_error("Error: Address out of range in write8");
        }
        else
        {
            memory[adr - base_address] = data;
        }
    }

    /**
     * @brief checks if a peripheral is addressed
     *
     * @param adr address to search for
     * @return true if a peripheral is found
     * @return false if no peripheral is found
     */
    bool is_addressed(uint64_t adr) override
    {
        return adr >= base_address && adr < size + base_address;
    }

    bool copy_into_memory(char* data, uint64_t size);

    /***
     * @brief Generates a c_soft_memory Dump in the specified file
     * The dump is in the format:
     * Address    Data (Hexadecimal)   ASCII
     * 0x1000000: 00000000 00000000 00000000 00000000 |................|
     * empty lines are not printed for performance reasons
     */
    void dump_memory(const std::string& filename) const;

    void dump_signature(const std::string& signature_path, uint64_t begin, uint64_t end) const;
};

#endif