/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Marco Milenkovic <Marco.Milenkovic@hs-augsburg.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: This file contains the implementation of the c_elf_reader Class for loading ELF flies into memory for simulation ONLY

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

#ifndef ELF_READER_H
#define ELF_READER_H

#include <piconut.h>
#include <cstdint>
#include <elf.h>
#include <cstring>
#include <map>
#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>

class c_elf_reader
{
public:
    uint32_t size = 0;
    uint32_t start_adr = UINT32_MAX;

    std::vector<Elf32_Phdr> program_headers;
    std::vector<Elf32_Shdr> section_headers;
    std::string section_names;

    /**
     * @brief Construct a new c_elf_reader object initialized with the contents of an ELF file
     * Calls the initialize_from_file function to load the ELF file into memory
     * @param filename name of the ELF file to load
     */
    c_elf_reader(const std::string& filename);

    /**
     * @brief Destroy the c_elf_reader object
     *
     */
    ~c_elf_reader();

    /**
     * @brief calls the functions needed to laod a ELF file into memory
     * makes size and star address calculations and stores them.
     */
    bool initialize_from_file();

    /**
     * @brief get the modified memory representation with the contents of the ELF File
     *
     * @return char*
     */
    char* get_memory() const
    {
        return memory_to_load;
    }

    /**
     * @brief dump the memory with the symbols for debug purposes
     *
     * @param filename name of the file to dump the memory to
     */
    void dump_memory_with_symbols(const std::string& filename) const;

private:
    std::string filename;
    std::map<uint32_t, std::string> symbol_table;
    char* memory_to_load; // Memory representation of the ELF file

    /**
     * @brief parse the headers of the ELF file
     * also parse the sections and symbolic table
     * @return true if the headers were parsed successfully
     * @param file file to parse
     * @param header header of the ELF file
     */
    bool parse_headers(FILE* file, unsigned char data_encoding);

    /**
     * @brief load the symbols from the ELF file
     *
     * @param file file to load the symbols from
     * @param symtab symbol table
     */
    bool parse_sections(FILE* file, unsigned char data_encoding);

    /**
     * @brief convert the data from the ELF file to the correct endianess
     *
     * @param val value to convert
     * @param data_encoding data encoding of the ELF file
     * @return uint32_t
     */
    uint32_t convert32(uint32_t val, unsigned char data_encoding) const;

    /**
     * @brief convert 16 bit data to the correct endianess
     *
     * @param val value to convert
     * @param data_encoding data encoding of the ELF file
     * @return uint16_t
     */
    uint16_t convert16(uint16_t val, unsigned char data_encoding) const;

    /**
     * @brief makes a memory dump of the loaded memory for debugging purposes
     *
     * @param filename name of the output txt file
     */
    void dump_memory(const std::string& filename) const;

    /**
     * @brief returns the smybol table as map
     *
     * @return can be accessed with the address of the symbol as key
     */
    std::map<uint32_t, std::string> get_symbol_table()
    {
        return symbol_table;
    }
};

#endif // ELF_READER_H
