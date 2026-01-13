/**
 * @file c_debug_reg_abstractcs.h
 * @brief This file contains the declaration of the c_debug_reg_abstractcs class.
 * For simulation ONLY.
 * @author Johannes Hofmann
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the declaration of the c_debug_reg_abstractcs class.
      For simulation ONLY.

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
 * @addtogroup c_debug_reg_abstractcs
 *
 * The `c_debug_reg_abstractcs` module is a submodule of the `c_debug_regs` module.
 * It implements the `abstractcs` register exposed to the host according to the
 * `RISC-V External Debug Support` specification. The register is accessed by
 * the `DMI-bus`.
 *
 * Note: The module is not synthesizable and is only used for simulation purposes.
 *
 */

#ifndef __C_DEBUG_REG_ABSTRACTCS_H__
#define __C_DEBUG_REG_ABSTRACTCS_H__

#include <cstddef>
#include <cstdint>

class c_soft_dm;

class c_debug_reg_abstractcs
{
    friend class c_soft_dm;

public:
    enum class e_cmderr : uint32_t
    {
        NONE = 0,
        BUSY = 1,
        NOT_SUPPORTED = 2,
        EXCEPTION = 3,
        HALT_RESUME = 4,
        BUS = 5,
        OTHER = 7
    };

    enum class e_busy : uint32_t
    {
        RESET = 0,
        SET = 1,
    };

    // Abstract Control and Status (reg, at 0x16)
    struct reg_t
    {
        uint32_t datacount : 4;
        uint32_t reserved0 : 4; // Reserved
        e_cmderr cmderr : 3;
        uint32_t reserved1 : 1; // Reserved
        e_busy busy : 1;
        uint32_t reserved2 : 11; // Reserved
        uint32_t progbufsize : 5;
        uint32_t reserved3 : 3;
    };

public:
    /**
     * @brief Constructor
     */
    c_debug_reg_abstractcs(
        const size_t data_size,
        const size_t progbuf_size);

    /**
     * @brief Destructor
     */
    ~c_debug_reg_abstractcs();

    /**
     * @brief Write 32-bit integer to register.
     * @param data Data that will be written into the register.
     */
    void write(uint32_t data);

    /**
     * @brief Read register as 32-bit integer.
     * @return Register data.
     */
    uint32_t read() const;

    /**
     * @brief Set datacount
     * @param datacount New datacount
     */
    void set_datacount(uint32_t datacount);
    void set_cmderr(e_cmderr cmderr);
    void set_busy(e_busy busy);
    void set_progbufsize(uint32_t progbufsize);

    /**
     * @brief Get datacount
     * @return datacount
     */
    uint32_t datacount();
    e_cmderr cmderr();
    e_busy busy();
    uint32_t progbufsize();

private:
    reg_t reg;
};

#endif