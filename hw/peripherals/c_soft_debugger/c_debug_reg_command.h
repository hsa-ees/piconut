/**
 * @file c_debug_reg_command.h
 * @brief This file contains the definition of the c_debug_reg_command class.
 * For simulation ONLY.
 * @author Johannes Hofmann
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Johannes Hofmann <johannes.hofmann@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the implementation of the c_debug_reg_command.
      For simulation ONLY

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
 * @addtogroup c_debug_reg_command
 *
 * The `c_debug_reg_command` module is a submodule of the `c_debug_regs` module.
 * It implements the `command` register exposed to the host according to the
 * `RISC-V External Debug Support` specification. The register is accessed by
 * the `DMI-bus`.
 *
 * Note: The module is not synthesizable and is only used for simulation purposes.
 *
 */

#ifndef __C_DEBUG_REG_COMMAND_H__
#define __C_DEBUG_REG_COMMAND_H__

#include <cstdint>
#include <vector>

class c_soft_dm;
class c_debug_reg_abstractcs;

class c_debug_reg_command
{
public:
    enum class e_cmdtype : uint32_t
    {
        REG_ACCESS = 0,    // Access Register Command
        QUICK_ACCESS = 1,  // Quick Access
        MEMORY_ACCESS = 2, // Access Memory Command
    };

    // Abstract Command (command, at 0x17)
    struct reg_t
    {
        uint32_t control : 24;
        e_cmdtype cmdtype : 8;
    };

    enum class e_cmd_reg_access_aarsize : uint32_t
    {
        LOWEST_32 = 2,
        LOWEST_64 = 3,
        LOWEST_128 = 4,
    };

    struct reg_command_reg_access_t
    {
        uint32_t regno : 16;
        uint32_t write : 1;
        uint32_t transfer : 1;
        uint32_t postexec : 1;
        uint32_t aarpostincrement : 1;
        e_cmd_reg_access_aarsize aarsize : 3;
        uint32_t reserved0 : 1;
        e_cmdtype cmdtype : 8;
    };

    struct reg_command_quick_access_t
    {
        uint32_t reserved0 : 24;
        e_cmdtype cmdtype : 8;
    };

    enum class e_cmd_memory_access_aamsize : uint32_t
    {
        BITS_8 = 0,
        BITS_16 = 1,
        BITS_32 = 2,
        BITS_64 = 3,
        BITS_128 = 4,
    };

    enum class e_cmd_memory_access_aamvirtual : uint32_t
    {
        PHYSICAL = 0,
        VIRTUAL = 1,
    };

    struct reg_command_memory_access_t
    {
        uint32_t reserved0 : 14;
        uint32_t target_specific : 2;
        uint32_t write : 1;
        uint32_t reserved1 : 2;
        uint32_t aampostincrement : 1;
        e_cmd_memory_access_aamsize aamsize : 3;
        e_cmd_memory_access_aamvirtual aamvirtual : 1;
        e_cmdtype cmdtype : 8;
    };

public:
    /**
     * @brief Constructor
     */
    c_debug_reg_command(
        c_soft_dm* dm,
        c_debug_reg_abstractcs* abstractcs);

    /**
     * @brief Destructor
     */
    ~c_debug_reg_command();

    /**
     * @brief Write 32-bit integer to register.
     * @param data Data that will be written into the register.
     */
    void write(uint32_t data);

    /**
     * @brief Read register as 32-bit integer.
     * @return Register data.
     */
    uint32_t read();

    void set_control(uint32_t control);
    void set_cmdtype(e_cmdtype cmdtype);

    uint32_t control();
    e_cmdtype cmdtype();

private:
    void _handle_new_command();
    void _handle_reg_access_command();
    void _handle_quick_access_command();
    void _handle_memory_access_command();

    std::vector<uint32_t> _make_abstract_command_reg_access() const;
    std::vector<uint32_t> _make_abstract_command_reg_access_csr(
        const reg_command_reg_access_t& reg_command_reg_access) const;
    std::vector<uint32_t> _make_abstract_command_reg_access_gpr(
        const reg_command_reg_access_t& reg_command_reg_access) const;

    std::vector<uint32_t> _make_abstract_command_memory_access() const;

private:
    c_soft_dm* dm;
    c_debug_reg_abstractcs* abstractcs;

    reg_t reg;
};

#endif
