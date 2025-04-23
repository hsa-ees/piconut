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

#include "elf_reader.h"

c_elf_reader::c_elf_reader(const std::string &filename) : filename(filename), size(0), start_adr(UINT32_MAX)
{
    // Initialize the ELF file and sets size and start_adr
    if (!initialize_from_file())
    {
        std::cerr << "Initialization failed during the construction of c_elf_reader." << std::endl;
    }
}

c_elf_reader::~c_elf_reader() {}

/***
 * Initializes the ELF file and sets the size and start address of the PT_LOAD content.
 * @return True if initialization is successful, false otherwise.
 */
bool c_elf_reader::initialize_from_file()
{
    FILE *file = fopen(filename.c_str(), "rb");
    if (!file)
    {
        std::cerr << "Failed to open ELF file: " << filename << std::endl;
        return false;
    }

    Elf32_Ehdr header;
    if (fread(&header, sizeof(header), 1, file) != 1)
    {
        std::cerr << "Failed to read ELF header from file." << std::endl;
        fclose(file);
        return false;
    }

    // Validate ELF magic number and ensure it's 32-bit ELF
    if (header.e_ident[EI_MAG0] != ELFMAG0 || header.e_ident[EI_MAG1] != ELFMAG1 ||
        header.e_ident[EI_MAG2] != ELFMAG2 || header.e_ident[EI_MAG3] != ELFMAG3 ||
        header.e_ident[EI_CLASS] != ELFCLASS32)
    {
        std::cerr << "Invalid ELF file." << std::endl;
        fclose(file);
        return false;
    }

    unsigned char data_encoding = header.e_ident[EI_DATA]; // Little/big-endian

    uint32_t max_segment_end = 0;  // Track the highest segment end
    uint32_t max_mem_end = 0;      // Track the highest memory end
    start_adr = UINT32_MAX;        // Track the lowest starting address
    uint32_t prev_segment_end = 0; // For tracking gaps

    fseek(file, header.e_phoff, SEEK_SET);
    for (int i = 0; i < header.e_phnum; ++i)
    {
        Elf32_Phdr phdr;
        if (fread(&phdr, sizeof(phdr), 1, file) != 1)
        {
            std::cerr << "Failed to read program header in section." << std::endl;
            fclose(file);
            return false;
        }

        if (convert32(phdr.p_type, data_encoding) == PT_LOAD || convert32(phdr.p_type, data_encoding) == PT_TLS)
        {
            uint32_t segment_start = convert32(phdr.p_paddr, data_encoding);
            uint32_t segment_memsz = convert32(phdr.p_memsz, data_encoding);
            uint32_t segment_filesz = convert32(phdr.p_filesz, data_encoding);
            uint32_t segment_align = convert32(phdr.p_align, data_encoding);

            // If no alignment is specified, assume 0x1000 (default alignment)
            if (segment_align == 0)
            {
                segment_align = 0x1000;
            }

            uint32_t segment_end = segment_start + segment_memsz;

            // Track the minimum starting address
            if (segment_start < start_adr)
            {
                start_adr = segment_start;
            }

            // Track the maximum memory end address
            if (segment_end > max_mem_end)
            {
                max_mem_end = segment_end;
            }

            // Detect gaps between segments
            if (prev_segment_end != 0 && segment_start > prev_segment_end)
            {
                uint32_t gap_size = segment_start - prev_segment_end;
#ifdef debug
                std::cout << "Found gap of size: " << gap_size << " between segments at 0x"
                          << std::hex << prev_segment_end << " and 0x" << segment_start << std::endl;
#endif
                max_segment_end += gap_size; // Add the gap to the total memory size
            }

            // Update the previous segment end for the next iteration
            prev_segment_end = segment_end;

            // Align the segment end after calculating the gaps
            if (segment_end % segment_align != 0)
            {
                segment_end += segment_align - (segment_end % segment_align);
            }

            // Update the maximum segment end with the aligned end
            if (segment_end > max_segment_end)
            {
                max_segment_end = segment_end;
            }
        }
    }

    // Calculate the total size needed for memory
    size = max_segment_end - start_adr;
#ifdef debug
    std::cout << "Calculated total memory size (aligned): " << size << " bytes" << std::endl;
#endif
    // Allocate memory based on the calculated size
    memory_to_load = new char[size];
    if (!memory_to_load)
    {
        std::cerr << "Failed to allocate memory." << std::endl;
        delete[] memory_to_load;
        fclose(file);
        return false;
    }

    // Zero out the allocated memory
    memset(memory_to_load, 0, size);

    // Parse and load headers
    if (!parse_headers(file, data_encoding))
    {
        std::cerr << "Failed to parse ELF headers." << std::endl;
        delete[] memory_to_load;
        fclose(file);
        return false;
    }
    if (!parse_sections(file, data_encoding))
    {
        std::cerr << "Failed to parse ELF sections." << std::endl;
        fclose(file);
        return false;
    }

    // Dump memory with symbols for debugging
    dump_memory_with_symbols("ELF_memory_with_symbols_dump.txt");
    // Dump memory for debugging
    dump_memory("ELF_memory_dump.txt");

    fclose(file);

    return true;
}

bool c_elf_reader::parse_headers(FILE *file, unsigned char data_encoding)
{
    Elf32_Ehdr header;
    fseek(file, 0, SEEK_SET);
    fread(&header, sizeof(header), 1, file);

    uint32_t phoff = convert32(header.e_phoff, data_encoding);         // Program header offset
    uint16_t phnum = convert16(header.e_phnum, data_encoding);         // Number of program headers
    uint16_t phentsize = convert16(header.e_phentsize, data_encoding); // Size of each program header

    if (phnum == 0)
    {
        std::cerr << "No program headers to process." << std::endl;
        return false;
    }

    // Allocate memory for program headers and read them from the file
    Elf32_Phdr *program_headers = new Elf32_Phdr[phnum];
    fseek(file, phoff, SEEK_SET);
    if (fread(program_headers, phentsize, phnum, file) != phnum)
    {
        std::cerr << "Error reading program headers." << std::endl;
        delete[] program_headers;
        return false;
    }

    for (int i = 0; i < phnum; ++i)
    {
        Elf32_Phdr &phdr = program_headers[i];
        uint32_t p_type = convert32(phdr.p_type, data_encoding);

        // Only process loadable segments (PT_LOAD)
        if (p_type == PT_LOAD)
        {
            uint32_t p_paddr = convert32(phdr.p_paddr, data_encoding);
            uint32_t p_filesz = convert32(phdr.p_filesz, data_encoding);
            uint32_t p_memsz = convert32(phdr.p_memsz, data_encoding);
            uint32_t p_offset = convert32(phdr.p_offset, data_encoding);

#ifdef debug
            std::cout << "Processing PT_LOAD segment: Addr " << std::hex << p_paddr
                      << ", Offset " << p_offset << ", File size " << p_filesz
                      << ", Mem size " << p_memsz << std::dec << std::endl;
#endif

            // Read segment into a buffer
            fseek(file, p_offset, SEEK_SET);
            uint8_t *buffer = new uint8_t[p_filesz];
            if (fread(buffer, 1, p_filesz, file) != p_filesz)
            {
                std::cerr << "Failed to read segment data from file.\n";
                delete[] buffer;
                continue;
            }

            // Write the read data to the memory_to_load (since it's just a char*)
            for (uint32_t j = 0; j < p_filesz; j += 4)
            {
                if (j + 4 <= p_filesz)
                {
                    // Read 4 bytes and convert it to the correct endianness
                    uint32_t word = *reinterpret_cast<uint32_t *>(buffer + j);
                    word = convert32(word, data_encoding);
                    // Write the 32-bit word into memory_to_load
                    memcpy(memory_to_load + (p_paddr - start_adr) + j, &word, sizeof(word));
                }
            }

            // Zero out remaining memory if mem size is greater than file size
            uint32_t padding_size = p_memsz - p_filesz;
            if (padding_size > 0)
            {
                memset(memory_to_load + (p_paddr - start_adr) + p_filesz, 0, padding_size);
            }

            delete[] buffer;
        }
    }

#ifdef debug
    std::cout << "Determined size: " << size << " B , start address: 0x" << std::hex << start_adr << std::endl;
#endif

    delete[] program_headers;
    return true;
}

bool c_elf_reader::parse_sections(FILE *file, unsigned char data_encoding)
{
    // Reset to the start of the file and read the ELF header again
    Elf32_Ehdr header;
    fseek(file, 0, SEEK_SET);
    if (fread(&header, sizeof(header), 1, file) != 1)
    {
        std::cerr << "Failed to read ELF header." << std::endl;
        return false;
    }

    uint32_t shoff = convert32(header.e_shoff, data_encoding);         // Section header table offset
    uint16_t shnum = convert16(header.e_shnum, data_encoding);         // Number of section headers
    uint16_t shentsize = convert16(header.e_shentsize, data_encoding); // Size of each section header

    if (shnum == 0)
    {
        std::cerr << "No section headers to process." << std::endl;
        return true; // No sections is not an error in some cases
    }

    // Allocate memory for section headers
    Elf32_Shdr *section_headers = new Elf32_Shdr[shnum];
    fseek(file, shoff, SEEK_SET);
    if (fread(section_headers, shentsize, shnum, file) != shnum)
    {
        std::cerr << "Failed to read section headers." << std::endl;
        delete[] section_headers;
        return false;
    }

    // Look for the string table section header (for symbol names)
    char *str_tab = nullptr;
    for (int i = 0; i < shnum; ++i)
    {
        if (section_headers[i].sh_type == SHT_STRTAB)
        {
            uint32_t str_offset = convert32(section_headers[i].sh_offset, data_encoding);
            uint32_t str_size = convert32(section_headers[i].sh_size, data_encoding);

            // Read the string table into memory
            str_tab = new char[str_size];
            fseek(file, str_offset, SEEK_SET);
            if (fread(str_tab, 1, str_size, file) != str_size)
            {
                std::cerr << "Failed to read string table." << std::endl;
                delete[] section_headers;
                delete[] str_tab;
                return false;
            }
            break; // Assume the first string table is the one we need
        }
    }

    // Now look for the symbol table and populate the symbol table map
    for (int i = 0; i < shnum; ++i)
    {
        if (section_headers[i].sh_type == SHT_SYMTAB && str_tab != nullptr)
        {
            uint32_t sym_offset = convert32(section_headers[i].sh_offset, data_encoding);
            uint32_t sym_size = convert32(section_headers[i].sh_size, data_encoding);
            uint32_t sym_entry_size = convert32(section_headers[i].sh_entsize, data_encoding);

            // Calculate the number of symbols
            int num_symbols = sym_size / sym_entry_size;

            // Allocate memory for symbols
            Elf32_Sym *symbols = new Elf32_Sym[num_symbols];
            fseek(file, sym_offset, SEEK_SET);
            if (fread(symbols, sym_entry_size, num_symbols, file) != num_symbols)
            {
                std::cerr << "Failed to read symbols." << std::endl;
                delete[] section_headers;
                delete[] str_tab;
                delete[] symbols;
                return false;
            }

            // Populate the symbol table
            for (int j = 0; j < num_symbols; ++j)
            {
                uint32_t name_index = convert32(symbols[j].st_name, data_encoding);
                if (name_index == 0)
                    continue; // No symbol name

                const char *name = &str_tab[name_index];
                uint32_t addr = convert32(symbols[j].st_value, data_encoding);

                // Insert into the symbol table map
                symbol_table[addr] = name;
            }

            delete[] symbols;
        }
    }

    delete[] section_headers;
    if (str_tab)
        delete[] str_tab;

    return true;
}

uint32_t c_elf_reader::convert32(uint32_t val, unsigned char data_encoding) const
{
    if (data_encoding == ELFDATA2LSB)
    {
        return val;
    }
    else
    {
        return __builtin_bswap32(val);
    }
}

uint16_t c_elf_reader::convert16(uint16_t val, unsigned char data_encoding) const
{
    if (data_encoding == ELFDATA2LSB)
    {
        return val;
    }
    else
    {
        return __builtin_bswap16(val);
    }
}

void c_elf_reader::dump_memory_with_symbols(const std::string &filename) const
{
    if (!memory_to_load || size == 0)
    {
        std::cerr << "Memory not loaded or size is zero." << std::endl;
        return;
    }

    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file for memory dump." << std::endl;
        return;
    }

    file << std::hex << std::setfill('0');
    file << "Address    " << "Data (Hexadecimal)                                 " << "ASCII" << "             " << "Symbol" << std::endl;
    file << "--------------------------------------------------------------------------------------------------" << std::endl;

    for (size_t i = 0; i < size; i += 16)
    {
        uint32_t address = start_adr + i;

        // Check if the address is within valid bounds
        if (address < start_adr || address >= start_adr + size)
        {
            std::cerr << "Address out of bounds: 0x" << std::hex << address << std::endl;
            continue;
        }

        char ascii[17] = {};
        bool hasNonZero = false;

        for (int j = 0; j < 16 && (i + j) < size; ++j)
        {
            uint8_t byte = memory_to_load[i + j]; // Use memory_to_load instead of memory_with_symbols
            ascii[j] = (byte >= 32 && byte <= 126) ? byte : '.';

            if (byte != 0)
                hasNonZero = true;
        }

        if (hasNonZero)
        {
            // Print the starting address of the line
            file << "0x" << std::setw(8) << address << ": ";

            // Print the memory in hexadecimal
            for (int j = 0; j < 16 && (i + j) < size; ++j)
            {
                uint8_t byte = memory_to_load[i + j]; // Use memory_to_load for hex dump
                file << std::setw(2) << static_cast<unsigned>(byte) << " ";
            }

            // Print the ASCII representation
            file << " |" << ascii << "|";

            // Check if there's a symbol for the current address
            auto it = symbol_table.find(address);
            if (it != symbol_table.end())
            {
                file << " " << it->second; // Add the symbol if it exists
            }

            file << std::endl;
        }
    }

    file.close();
}

void c_elf_reader::dump_memory(const std::string &filename) const
{
    if (!memory_to_load || size == 0)
    {
        std::cerr << "Memory not loaded or size is zero." << std::endl;
        return;
    }

    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file for memory dump." << std::endl;
        return;
    }

    file << std::hex << std::setfill('0');
    // Print a header for the columns for better readability
    file << "Address    " << "Data (Hexadecimal)                                 " << "ASCII" << std::endl;
    file << "------------------------------------------------------------------------------------" << std::endl;

    for (size_t i = 0; i < size; i += 16)
    {
        // Initialize a flag to check if the entire line is zero
        bool hasNonZero = false;

        // Buffer for ASCII representation
        char ascii[17];
        ascii[16] = '\0'; // Ensure null-terminated string for ASCII output

        // Check each byte in this line to not print empty lines (for performance)
        for (int j = 0; j < 16; j++)
        {
            if (i + j < size && memory_to_load[i + j] != 0)
            {
                hasNonZero = true; // Set flag if any byte is non-zero
                break;             // No need to continue checking once a non-zero is found
            }
        }

        // Only print this line if a non-zero byte was found
        if (hasNonZero)
        {
            // Print the starting address of the line
            file << "0x" << std::setw(8) << (start_adr + i) << ": ";

            // Process each byte in the block for printing
            for (int j = 0; j < 16; j++)
            {
                if (i + j < size)
                {
                    uint8_t byte = static_cast<uint8_t>(memory_to_load[i + j]);
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
