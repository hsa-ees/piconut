/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lorenz Sommer <lorenz.sommer@tha.de>
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

 *************************************************************************/

/* This file is used for enumerations and other declarations to be used across multiple modules. */

#ifndef __TYPEDEF_H__
#define __TYPEDEF_H__

typedef enum
{
    ALU_MODE_REG_REG = 0x0,
    ALU_MODE_REG_IMM = 0x1,
    ALU_MODE_MUL = 0x2,
} e_alu_mode;

typedef enum
{
    FUNCT7_RV32I_BASE = 0x0,
    FUNCT7_RV32I_SRA_SUB = 0x20,
    FUNCT7_M_EXTENSION = 0x1,
} e_funct7;

typedef enum
{
    FUNCT3_ADD_SUB = 0x0,
    FUNCT3_SLL = 0x1,
    FUNCT3_SLT = 0x2,
    FUNCT3_SLTU = 0x3,
    FUNCT3_XOR = 0x4,
    FUNCT3_SRL_SRA = 0x5,
    FUNCT3_OR = 0x6,
    FUNCT3_AND = 0x7,
} e_alu_funct3;

typedef enum
{
    FUNCT3_CSR_CSRRW = 0x1,
    FUNCT3_CSR_CSRRS = 0x2,
    FUNCT3_CSR_CSRRC = 0x3,
    FUNCT3_CSR_CSRRWI = 0x5,
    FUNCT3_CSR_CSRRSI = 0x6,
    FUNCT3_CSR_CSRRCI = 0x7,
} e_funct3_csr;

typedef enum
{
    OP_ALU = 0x0C,
    OP_ALUI = 0x4,
    OP_LOAD = 0x0,
    OP_STORE = 0x8,
    OP_BRANCH = 0x18,
    OP_JAL = 0x1B,
    OP_JALR = 0x19,
    OP_LUI = 0xD,
    OP_AUIPC = 0x5,
    OP_FENCE = 0x3,
    OP_SYSTEM = 0x1C,
} e_opcode;

typedef enum
{
    BYTE_0 = 0x1,
    BYTE_1 = 0x2,
    BYTE_2 = 0x4,
    BYTE_3 = 0x8,
    HALF_LOWER = 0x3,
    HALF_UPPER = 0xC,
    WORD = 0xF,
} e_bsel;

typedef enum
{
    CSR_WRITE_ALL = 0x0,
    CSR_WRITE_SET = 0x1,
    CSR_WRITE_CLEAR = 0x2,
} e_csr_write_mode;

#endif // __TYPEDEF_H__