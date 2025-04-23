/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Marco Milenkovic <Marco.Milenkovic@hs-augsburg.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: This file contains the implementation of the c_soft_memory  simulation ONLY

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

#include "c_soft_memory.h"

void c_soft_memory::dump_memory(const std::string &filename) const
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file for memory dump." << std::endl;
        return;
    }

    file << std::hex << std::setfill('0');
    // Print a header for the columns for better readability
    file << "Address    " << "Data (Hexadecimal)   " << "ASCII" << std::endl;

    for (size_t i = 0; i < memory.size(); i += 16)
    {
        // Initialize a flag to check if the entire line is zero
        bool has_non_zero = false;

        // Buffer for ASCII representation
        char ascii[17];
        ascii[16] = '\0'; // Ensure null-terminated string for ASCII output

        // Check each byte in this line to not print empty lines (because of performance)
        for (int j = 0; j < 16; j++)
        {
            if (i + j < memory.size() && memory[i + j] != 0)
            {
                has_non_zero = true; // Set flag if any byte is non-zero
                break;               // No need to continue checking once a non-zero is found
            }
        }

        // Only print this line if a non-zero byte was found
        if (has_non_zero)
        {
            // Print the starting address of the line
            file << "0x" << std::setw(8) << (base_address + i) << ": ";

            // Process each byte in the block for printing
            for (int j = 0; j < 16; j++)
            {
                if (i + j < memory.size())
                {
                    uint8_t byte = memory[i + j];
                    file << std::setw(2) << static_cast<unsigned>(byte) << " ";

                    // Convert byte to ASCII if printable, use '.' otherwise
                    ascii[j] = (byte >= 32 && byte <= 126) ? byte : '.';
                }
                else
                {
                    file << "   ";  // Align non-existent bytes
                    ascii[j] = ' '; // Fill ASCII buffer with spaces for alignment
                }
            }

            // Print ASCII representation at the end of each line
            file << " |" << ascii << "|" << std::endl;
        }
    }

    file.close();
}

bool c_soft_memory::copy_into_memory(char *data, uint64_t size)
{
#ifdef debug
    std::cout << "Copying " << size << " bytes to memory starting at base address 0x" << std::hex << base_address << std::endl;
#endif
    // Ensure that size to be copied does not exceed the allocated memory size
    if (size > this->size)
    {
        std::cerr << "Error: Copy size exceeds allocated memory size." << std::endl;
        return false;
    }

    // Log the expected memory size and data size to compare
#ifdef debug
    std::cout << "Allocated memory size: " << this->size << ", Data size to copy: " << size << std::endl;
#endif
    // Copy the data into memory using the memory vector
    for (uint64_t i = 0; i < size; i++)
    {
        uint64_t mem_address = base_address + i;

        // Ensure the address does not exceed the bounds of the memory vector
        if (base_address + i >= base_address + this->size)
        {
            std::cerr << "Error: Memory address out of bounds at address 0x" << std::hex << mem_address << std::endl;
            return false;
        }

        // Write data to the memory vector
        memory[i] = static_cast<uint8_t>(data[i]);
    }
    if (size < this->size)
    {
        std::fill(memory.begin() + size, memory.end(), 0); // Zero out remaining memory
    }

#ifdef debug
    std::cout << "Copy completed successfully." << std::endl;
#endif
    return true;
}
