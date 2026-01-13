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

 /** immgen (Immediate Generator)
 * The immgen module is responsible for decoding the immediate value of an instruction.
 *
 * Input ports:
 *  - data_in<32>:    The instruction word to be decoded.
 *
 * Output ports:
 *  - imm_out<32>:   The decoded immediate value.
 *
 * The instruction type is encoded in the opcode of the instruction word.
 * The opcode is defined as IR[6:0].
 *
 * Because the lowest two bits of the opcode are always '11',
 * it is sufficient to evaluate the upper 5 bits of the opcode to determine the instruction type.
 * For the sake of clarity, these 5 bits are called the "instruction identifier".
 *
 * The relevant instruction types are: I-Type, S-Type, B-Type, U-Type, and J-Type.
 *
 * The R-Type instruction format is omitted in the immediate decoding process,
 * because it does not contain any immediate values.
 *
 * Note: Instructions of the same type may have different identifiers.
 *       This has to be taken into account when decoding the immediate values.
 *
 * The resulting immediate value of an instruction is generated from a combination
 * and rearrangement of bits in the instruction word as seen in the following table:
 *
 * Type   | --------------------------- Immediate Value ----------------------------------
 * I-Type | (inst[31])[31:11] + inst[30:25] + inst[24:21] + inst[20]
 * S-Type | (inst[31])[31:11] + inst[30:25] + inst[11:8]  + inst[7]
 * B-Type | (inst[31])[31:12] + inst[7]     + inst[30:25] + inst[11:8]  + 0
 * U-Type | inst[31]          + inst[30:20] + inst[19:12] + 0[11:0]
 * J-Type | (inst[31])[31:20] + inst[19:12] + inst[20]    + inst[30:25] + inst[24:21] + 0
 *
 * For a detailed explanation of the immediate generation process, please refer to the RISC-V specification section
 * "Immediate Encoding Variants".
 */

#ifndef __IMMGEN_H__
#define __IMMGEN_H__

#include <systemc.h>
#include <piconut.h>

#include <stdint.h>

#include "typedef.h"


typedef enum
{
    IMMG_I_type,
    IMMG_S_Type,
    IMMG_B_Type,
    IMMG_U_type,
    IMMG_J_type
} e_immgen_instruction_type;

SC_MODULE(m_immgen)
{
public:
    sc_in<sc_uint<32>> PN_NAME(data_in);
    sc_out<sc_uint<32>> PN_NAME(imm_out);

    /* Constructor... */
    SC_CTOR(m_immgen)
    {
        SC_METHOD(proc_cmb_immgen);
        sensitive << data_in;
    }

    void pn_trace(sc_trace_file * tf, int level = 1);

    /* Process functions */
    void proc_cmb_immgen();

protected:
    sc_signal<sc_uint<3>> type_format;
};

#endif //__IMMGEN_H__
