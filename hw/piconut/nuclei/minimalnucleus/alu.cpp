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
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR a_in PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *************************************************************************/

#include "alu.h"

void m_alu::Trace(sc_trace_file* tf, int level)
{
    /* Ports ... */
    PN_TRACE(tf, a_in);
    PN_TRACE(tf, b_in);
    PN_TRACE(tf, y_out);
    PN_TRACE(tf, funct3_in);
    PN_TRACE(tf, force_add_in);
    PN_TRACE(tf, funct7_in);

    PN_TRACE(tf, equal_out);
    PN_TRACE(tf, less_out);
    PN_TRACE(tf, lessu_out);
}


void m_alu::proc_cmb_alu_rv32i()
{
    /* Local variables */
    sc_uint<64> shift_temp = 0x0;
    sc_uint<32> shamt = b_in.read().range(4, 0);

    /* Default */
    add_sub_result = 0x0;
    and_result = 0x0;
    or_result = 0x0;
    xor_result = 0x0;
    sll_result = 0x0;
    slt_result = 0x0;
    sltu_result = 0x0;
    srl_sra_result = 0x0;

    less_out = 0x0;
    lessu_out = 0x0;

    /* Perform arithmetic/logical right shift operations */

    /** Arithmetic right shift
     * The maximum amount a_in value can be shifted by is 32.
     * The "shamt" value is derived from b_in[4:0] leading a_in possible 2^5 = 32 values.
     * (This remains true for I-Type register-immediate instructions)
     *
     * The process is as follows:
     *  - Create a static 32-bit value and append the operand a_in to it.
     *  - Shift the value to the right by the amount specified in b_in[4:0].
     *  - This leads to a_in sign extension of the value because at most 32 1's will be shifted right.
     *  - The lower 32-bit half of the resulting 64-bit value is the result of the operation.
     *
     * This process is the same for the SRA and SRL instructions, with the only
     * difference being the value appended to the left of the operand.
     */

    /* ---------------------------------------------------------------------------------- */
    if(funct7_in.read() == FUNCT7_RV32I_SRA_SUB)
    {
        if(a_in.read()[31] == 0x1) // SRA when funct7 is 0x1
        {
            // Create 0xFFFFFFFF and append a_in to it -> leading 1's
            shift_temp = (sc_uint<32>(0xFFFFFFFF), a_in.read().range(31, 0));
        }
        else
        {
            // Create 0x0 and append a_in to it -> leading 0's
            shift_temp = (sc_uint<32>(0x0), a_in.read().range(31, 0));
        }
    }
    else
    {
        // Create 0x0 and append a_in to it -> leading 0's
        shift_temp = (sc_uint<32>(0x0), a_in.read().range(31, 0));
    }

    // Shift right by the amount specified in b_in[4:0]
    shift_temp = shift_temp >> shamt;

    // Select the lower 32-bit half of the 64-bit value and write to srl_sra_result
    srl_sra_result = shift_temp.range(31, 0);

    /* ---------------------------------------------------------------------------------- */
    /* Perform logical left shift operation */
    sll_result = a_in.read() << shamt;

    /* Perform addition/subtraction operation */
    if((funct7_in.read() == FUNCT7_RV32I_SRA_SUB) && alu_mode_in.read() == ALU_MODE_REG_REG && funct3_in.read() == FUNCT3_ADD_SUB)
    {
        add_sub_result = a_in.read() - b_in.read();
    }
    else
    {
        add_sub_result = a_in.read() + b_in.read();
    }

    /* ---------------------------------------------------------------------------------- */
    /* Perform bitwise AND operation */
    and_result = a_in.read() & b_in.read();

    /* ---------------------------------------------------------------------------------- */
    /* Perform bitwise OR operation */
    or_result = a_in.read() | b_in.read();

    /* ---------------------------------------------------------------------------------- */
    /* Perform bitwise XOR operation */
    xor_result = a_in.read() ^ b_in.read();

    /* ---------------------------------------------------------------------------------- */
    /* Set less_out than - output is 1 if a_in < b_in, else 0 */
    if(((int32_t)a_in.read()) < ((int32_t)b_in.read()))
    {
        slt_result = 0x1;
        // Set less_out status signal high.
        less_out = 0x1;
    }
    else
    {
        // less_out status signal remains 0x0.
        slt_result = 0x0;
    }

    /* ---------------------------------------------------------------------------------- */
    /* Set less_out than unsigned - output is 1 if a_in < b_in, else 0 */
    if((uint32_t)a_in.read() < (uint32_t)b_in.read())
    {
        sltu_result = 0x1;
        // Set less_out status signal 0x1.
        lessu_out = 0x1;
    }
    else
    {
        // less_out status signal remains 0x0.
        sltu_result = 0x0;
    }
}

void m_alu::proc_cmb_alu_output()
{

    equal_out = 0x0;
    y_out = 0x0;

    switch(alu_mode_in.read())
    {
        case ALU_MODE_REG_REG:
            switch(funct3_in.read())
            {
                case FUNCT3_ADD_SUB:
                    y_out = add_sub_result;
                    break;

                case FUNCT3_AND:
                    y_out = and_result;
                    break;

                case FUNCT3_OR:
                    y_out = or_result;
                    break;

                case FUNCT3_XOR:
                    y_out = xor_result;
                    break;

                case FUNCT3_SLT:
                    y_out = slt_result;
                    break;

                case FUNCT3_SLTU:
                    y_out = sltu_result;
                    break;

                case FUNCT3_SRL_SRA:
                    y_out = srl_sra_result;
                    break;

                case FUNCT3_SLL:
                    y_out = sll_result;
                    break;

                default:
                    y_out = 0x0;
                    PN_ERROR("ALU: Unknown operation");
                    break;
            }
            break;

        case ALU_MODE_REG_IMM:
            switch(funct3_in.read())
            {
                case FUNCT3_ADD_SUB:
                    y_out = add_sub_result;
                    break;

                case FUNCT3_AND:
                    y_out = and_result;
                    break;

                case FUNCT3_OR:
                    y_out = or_result;
                    break;

                case FUNCT3_XOR:
                    y_out = xor_result;
                    break;

                case FUNCT3_SLT:
                    y_out = slt_result;
                    break;

                case FUNCT3_SLTU:
                    y_out = sltu_result;
                    break;

                case FUNCT3_SRL_SRA:
                    y_out = srl_sra_result;
                    break;

                case FUNCT3_SLL:
                    y_out = sll_result;
                    break;

                default:
                    y_out = 0x0;
                    PN_ERROR("ALU: Unknown operation");
                    break;
            }
            break;

        case ALU_MODE_MUL: {
            /* M-Extension selection here */
            switch(funct3_in.read())
            {

                default:
                    y_out = 0x0;
                    PN_ERROR("ALU: Unknown operation");
                    break;
            }
            break;
        }
        default:
            y_out = 0x0;
            PN_ERROR("ALU: Unknown operation");
            break;
    }

    /* Set equal_out 1 if a_in == b_in, else 0 */
    if((a_in.read()) == (b_in.read()))
    {
        // Set equal_out status signal high.
        equal_out = 0x1;
    }

    /* Force add Flag - if set, force add operation */
    if(force_add_in.read() == 0x1)
    {
        y_out = a_in.read() + b_in.read();
    }
}