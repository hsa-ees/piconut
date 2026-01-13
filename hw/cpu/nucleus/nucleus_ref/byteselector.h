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

/** Byteselector
 *
 * The byteselector module generates the 'bsel' signal for the DPort-interface.
 *
 * Input ports:
 *  - adr_in<2>:           The lowest two bits of the adress signal (alu_out).
 *  - funct3_in<3>:         The funct3 block of the current instruction. (IR[14:12])
 *
 * Output ports:
 *  - byteselect<4>:    The byteselect signal.
 *  - invalid<1>:       Set when the current adress alignment is invalid.
 *
 * This module is necessary in order for the Nucleus to support byte and halfword load/store instructions.
 * The RISC-V architecture dictates the use of byte-addressable RAM.
 * Because this implementation is a 32-bit architecture, addresses are naturally 4-byte aligned.
 * This leads to the lowest two bits of the address-bus being negligible for addressing memory.
 * However, in order to support byte and halfword load/store-instructions, these bits are essential
 * to determining the byteselect signal (bsel) of the memory interface.
 *
 * The lowest two bits of the address-bus are used to determine the byte- or halfword-alignedness of the corresponding
 * load/store-request.
 *
 * Example:
 *  - The source register holds the value 0x12345678.
 *  - The address-bus holds the value 0x00000001. (Calculated by the ALU)
 *  - The lowest two bits are discarded for memory addressing. The effective address to the memory
 *    interface is 0x00000000.
 *  - The instruction is a 'lb' instruction.
 *  - This module evaluates the lowest two bits of the raw address and the funct3 block of the instruction.
 *  - '01' in the raw address in combination with an instruction 'lb' yields a byteselect signal of '0010'.
 *
 * Table:
 *   adr_in[1:0] | funct3      | byteselect  | invalid
 *   ------------|----------------|----------|---------
 *   00          | lb/lbu/sb      | 0001     | 0
 *   01          | lb/lbu/sb      | 0010     | 0
 *   10          | lb/lbu/sb      | 0100     | 0
 *   11          | lb/lbu/sb      | 1000     | 0
 *
 *   00          | lh/lhu/sh      | 0011      | 0
 *   10          | lh/lhu/sh      | 1100      | 0
 *   01          | lh/lhu/sh      | 0000      | 1
 *   11          | lh/lhu/sh      | 0000      | 1
 *
 *   00          | lw             | 1111      | 0
 *   01          | lw             | 0000      | 1
 *   10          | lw             | 0000      | 1
 *   11          | lw             | 0000      | 1
 */

#ifndef __BYTESELECTOR_H__
#define __BYTESELECTOR_H__

#include <systemc.h>
#include <piconut.h>

#include <stdint.h>

#include "typedef.h"


typedef enum
{
    LB_SB = 0x0,
    LH_SH = 0x1,
    LW_SW = 0x2,
    LBU = 0x4,
    LHU = 0x5,
} e_byteselector_funct3;

typedef enum
{
    BYTE_0_ALIGNED = 0x0,
    BYTE_1_ALIGNED = 0x1,
    BYTE_2_ALIGNED = 0x2,
    BYTE_3_ALIGNED = 0x3,
} e_bysteselector_adr_align;

SC_MODULE(m_byteselector)
{
public:
    sc_in<sc_uint<2>> PN_NAME(adr_in);
    sc_in<sc_uint<3>> PN_NAME(funct3_in);

    sc_out<sc_uint<4>> PN_NAME(bsel_out);
    sc_out<bool> PN_NAME(invalid_out);

    /* Constructor... */
    SC_CTOR(m_byteselector)
    {
        SC_METHOD(proc_cmb_byteselector);
        sensitive << funct3_in << adr_in;
    }

    void pn_trace(sc_trace_file * tf, int level = 1);

    /* Process functions */
    void proc_cmb_byteselector();

protected:
};

#endif //__BYTESELECTOR_H__
