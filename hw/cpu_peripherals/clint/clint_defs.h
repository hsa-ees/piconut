/*******************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Niklas Sirch <niklas.sirch1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

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

 ******************************************************************************/

#ifndef __CLINT_DEFS_H__
#define __CLINT_DEFS_H__

#ifndef PN_CFG_CLINT_BASE_ADDRESS
/** @brief Base address of the clint in the system (Default 0x2000000 is RISCV-Standard)
 */
#define PN_CFG_CLINT_BASE_ADDRESS 0x2000000UL
#endif

/** @brief Size of the clint in the system (RISCV-Standard)
 */
#define CLINT_SIZE 0x000c0000UL

/** @brief CLINT register addresses */
enum e_clint_regs
{
    CLINT_REG_MSIP_ADDR = PN_CFG_CLINT_BASE_ADDRESS + 0x0,      // Machine Software Interrupt Pending register
    CLINT_REG_MTIMECMP_LO = PN_CFG_CLINT_BASE_ADDRESS + 0x4000, // Timer Compare Low
    CLINT_REG_MTIMECMP_HI = PN_CFG_CLINT_BASE_ADDRESS + 0x4004, // Timer Compare High
    CLINT_REG_MTIME_LO = PN_CFG_CLINT_BASE_ADDRESS + 0xBFF8,    // Timer Low
    CLINT_REG_MTIME_HI = PN_CFG_CLINT_BASE_ADDRESS + 0xBFFC     // Timer High
};

#endif // __CLINT_DEFS_H__