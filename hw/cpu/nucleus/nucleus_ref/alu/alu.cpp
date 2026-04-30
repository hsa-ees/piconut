/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lorenz Sommer <lorenz.sommer@tha.de>
                     2025 Daniel Sommerfeldt <daniel.sommerfeldt1@tha.de>
                     2026 Tristan Kundrat <tristan.kundrat@tha.de>
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

#include "scalar_crypto/scalar_crypto.h"

// change the following import to change out the multiplication module
#include "m_extension/mul/shift_and_add.h"
// change the following import to change out the division module
#include "m_extension/div/shift_and_subtract.h"

void m_alu::pn_trace(sc_trace_file* tf, int level)
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

    PN_TRACE(tf, add_sub_mul_result);
    PN_TRACE(tf, and_remu_result);
    PN_TRACE(tf, or_rem_result);
    PN_TRACE(tf, xor_div_result);
    PN_TRACE(tf, slt_mulhsu_result);
    PN_TRACE(tf, sltu_mulhu_result);
    PN_TRACE(tf, sll_mulh_result);
    PN_TRACE(tf, srl_sra_divu_result);

    PN_TRACE(tf, max_result);
    PN_TRACE(tf, min_result);
    PN_TRACE(tf, maxu_result);
    PN_TRACE(tf, minu_result);

#if PN_CFG_ALU_ENABLE_ZMMUL_EXTENSION == 1
    PN_TRACE(tf, mul_result);
    PN_TRACE(tf, mul_valid);
    PN_TRACE(tf, mul_start);
#endif
#if PN_CFG_ALU_ENABLE_M_EXTENSION == 1
    PN_TRACE(tf, div_rem_result);
    PN_TRACE(tf, div_valid);
    PN_TRACE(tf, div_start);
#endif

#if PN_CFG_ALU_ENABLE_ZMMUL_EXTENSION == 1
    multiplication_module->pn_trace(tf, level);
#endif
#if PN_CFG_ALU_ENABLE_M_EXTENSION == 1
    division_module->pn_trace(tf, level);
#endif

#if PN_CFG_ENABLE_ZKNE_ZKND == 1
    scalar_crypto->pn_trace(tf, level);
    PN_TRACE(tf, crypto_result);
    PN_TRACE(tf, crypto_valid);
#endif
}

bool i_is_crypto(sc_uint<3> funct3, sc_uint<7> funct7)
{
    sc_uint<5> funct7_lower = funct7.range(4, 0);

    // 000 e.g. for aes32 but aes64 has 001
    return funct3 == 0b000 && (funct7_lower == FUNCT7L_AES32DSI || funct7_lower == FUNCT7L_AES32ESI ||
                                  funct7_lower == FUNCT7L_AES32DSMI || funct7_lower == FUNCT7L_AES32ESMI);
}

void m_alu::proc_cmb_alu_rv32im()
{
    /* Local variables */
    sc_uint<64> shift_temp = 0x0;
    sc_uint<32> shamt = b_in.read().range(4, 0);

    /* Default */
    add_sub_mul_result = 0x0;
    and_remu_result = 0x0;
    or_rem_result = 0x0;
    xor_div_result = 0x0;
    max_result = 0x0;
    min_result = 0x0;
    maxu_result = 0x0;
    minu_result = 0x0;
    sll_mulh_result = 0x0;
    slt_mulhsu_result = 0x0;
    sltu_mulhu_result = 0x0;
    srl_sra_divu_result = 0x0;

    less_out = 0x0;
    lessu_out = 0x0;

    // valid_out 1 per default. mul*/div*/rem* overrides this.
    valid_out = 0x1;

#if PN_CFG_ALU_ENABLE_ZMMUL_EXTENSION == 1
    mul_start = 0x0;
#endif
#if PN_CFG_ALU_ENABLE_M_EXTENSION == 1
    div_start = 0x0;
#endif
#if PN_CFG_ENABLE_ZKNE_ZKND == 1
    crypto_start = 0x0;
#endif

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

#if PN_CFG_ALU_ENABLE_M_EXTENSION == 1
    // run divu calculation in division_module
    if(funct7_in.read() == FUNCT7_M_EXTENSION && alu_mode_in.read() == ALU_MODE_REG_REG && funct3_in.read() == FUNCT3_SRL_SRA_DIVU)
    {
        div_start = 0x1;
        valid_out = div_valid.read();
        srl_sra_divu_result = div_rem_result.read();
    }
    else
#endif
    // Select the lower 32-bit half of the 64-bit value and write to srl_sra_divu_result
    {
        srl_sra_divu_result = shift_temp.range(31, 0);
    }

    /* ---------------------------------------------------------------------------------- */
#if PN_CFG_ALU_ENABLE_ZMMUL_EXTENSION == 1
    // run mulh calculation in multiplication_module
    if(funct7_in.read() == FUNCT7_M_EXTENSION && alu_mode_in.read() == ALU_MODE_REG_REG && funct3_in.read() == FUNCT3_SLL_MULH)
    {
        mul_start = 0x1;
        valid_out = mul_valid.read();
        sll_mulh_result = mul_result.read();
    }
    else
#endif
    /* Perform logical left shift operation */
    {
        sll_mulh_result = a_in.read() << shamt;
    }

#if PN_CFG_ALU_ENABLE_ZMMUL_EXTENSION == 1
    // run mul calculation in multiplication_module
    if(funct7_in.read() == FUNCT7_M_EXTENSION && alu_mode_in.read() == ALU_MODE_REG_REG && funct3_in.read() == FUNCT3_ADD_SUB_MUL)
    {
        mul_start = 0x1;
        valid_out = mul_valid.read();
        add_sub_mul_result = mul_result.read();
    }
    else
#endif
    /* Perform addition/subtraction operation */
    {
        if(funct7_in.read() == FUNCT7_RV32I_SRA_SUB && alu_mode_in.read() == ALU_MODE_REG_REG && funct3_in.read() == FUNCT3_ADD_SUB_MUL)
        {
            add_sub_mul_result = a_in.read() - b_in.read();
        }
        else
        {
            add_sub_mul_result = a_in.read() + b_in.read();
        }
    }

    /* ---------------------------------------------------------------------------------- */
#if PN_CFG_ALU_ENABLE_M_EXTENSION == 1
    // run remu calculation in division_module
    if(funct7_in.read() == FUNCT7_M_EXTENSION && alu_mode_in.read() == ALU_MODE_REG_REG && funct3_in.read() == FUNCT3_AND_REMU)
    {
        div_start = 0x1;
        valid_out = div_valid.read();
        and_remu_result = div_rem_result.read();
    }
    else
#endif
    /* Perform bitwise AND operation */
    {
        and_remu_result = a_in.read() & b_in.read();
    }

    /* ---------------------------------------------------------------------------------- */
#if PN_CFG_ALU_ENABLE_M_EXTENSION == 1
    // run rem calculation in division_module
    if(funct7_in.read() == FUNCT7_M_EXTENSION && alu_mode_in.read() == ALU_MODE_REG_REG && funct3_in.read() == FUNCT3_OR_REM)
    {
        div_start = 0x1;
        valid_out = div_valid.read();
        or_rem_result = div_rem_result.read();
    }
    else
#endif
    /* Perform bitwise OR operation */
    {
        or_rem_result = a_in.read() | b_in.read();
    }

    /* ---------------------------------------------------------------------------------- */
#if PN_CFG_ALU_ENABLE_M_EXTENSION == 1
    // run div calculation in division_module
    if(funct7_in.read() == FUNCT7_M_EXTENSION && alu_mode_in.read() == ALU_MODE_REG_REG && funct3_in.read() == FUNCT3_XOR_DIV)
    {
        div_start = 0x1;
        valid_out = div_valid.read();
        xor_div_result = div_rem_result.read();
    }
    else
#endif
    /* Perform bitwise XOR operation */
    {
        xor_div_result = a_in.read() ^ b_in.read();
    }

    /* ---------------------------------------------------------------------------------- */
#if PN_CFG_ALU_ENABLE_ZMMUL_EXTENSION == 1
    // run mulhsu calculation in multiplication_module
    if(funct7_in.read() == FUNCT7_M_EXTENSION && alu_mode_in.read() == ALU_MODE_REG_REG && funct3_in.read() == FUNCT3_SLT_MULHSU)
    {
        mul_start = 0x1;
        valid_out = mul_valid.read();
        slt_mulhsu_result = mul_result.read();
    }
    else
#endif
    /* Set less_out than - output is 1 if a_in < b_in, else 0 */
    {
        if(((int32_t)a_in.read()) < ((int32_t)b_in.read()))
        {
            slt_mulhsu_result = 0x1;
            // Set less_out status signal high.
            less_out = 0x1;

            min_result = a_in.read();
            max_result = b_in.read();
        }
        else
        {
            // less_out status signal remains 0x0.
            slt_mulhsu_result = 0x0;

            min_result = b_in.read();
            max_result = a_in.read();
        }
    }

    /* ---------------------------------------------------------------------------------- */
#if PN_CFG_ALU_ENABLE_ZMMUL_EXTENSION == 1
    // run mulhu calculation in multiplication_module
    if(funct7_in.read() == FUNCT7_M_EXTENSION && alu_mode_in.read() == ALU_MODE_REG_REG && funct3_in.read() == FUNCT3_SLTU_MULHU)
    {
        mul_start = 0x1;
        valid_out = mul_valid.read();
        sltu_mulhu_result = mul_result.read();
    }
    else
#endif
    /* Set less_out than unsigned - output is 1 if a_in < b_in, else 0 */
    {
        if((uint32_t)a_in.read() < (uint32_t)b_in.read())
        {
            sltu_mulhu_result = 0x1;
            // Set less_out status signal 0x1.
            lessu_out = 0x1;

            minu_result = a_in.read();
            maxu_result = b_in.read();
        }
        else
        {
            // less_out status signal remains 0x0.
            sltu_mulhu_result = 0x0;

            minu_result = b_in.read();
            maxu_result = a_in.read();
        }
    }

#if PN_CFG_ENABLE_ZKNE_ZKND == 1
    if(i_is_crypto(funct3_in.read(), funct7_in.read()))
    {
        crypto_start = 0x1;
        valid_out = crypto_valid.read();
    }
#endif
}

void m_alu::proc_cmb_alu_output()
{

    equal_out = 0x0;
    y_out = 0x0;

    switch(alu_mode_in.read())
    {
        case ALU_MODE_IDLE:
            // do nothing
            break;

        case ALU_MODE_REG_REG:
            switch(funct3_in.read())
            {
                case FUNCT3_ADD_SUB_MUL:
#if PN_CFG_ENABLE_ZKNE_ZKND == 1
                    if(i_is_crypto(funct3_in.read(), funct7_in.read()))
                    {
                        y_out = crypto_result.read();
                        break;
                    }
                    else
#endif
                    {
                        y_out = add_sub_mul_result.read();
                        break;
                    }

                case FUNCT3_AND_REMU:
                    y_out = and_remu_result.read();
                    break;

                case FUNCT3_OR_REM:
                    y_out = or_rem_result.read();
                    break;

                case FUNCT3_XOR_DIV:
                    y_out = xor_div_result.read();
                    break;

                case FUNCT3_SLT_MULHSU:
                    y_out = slt_mulhsu_result.read();
                    break;

                case FUNCT3_SLTU_MULHU:
                    y_out = sltu_mulhu_result.read();
                    break;

                case FUNCT3_SRL_SRA_DIVU:
                    y_out = srl_sra_divu_result.read();
                    break;

                case FUNCT3_SLL_MULH:
                    y_out = sll_mulh_result.read();
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
                case FUNCT3_ADD_SUB_MUL:
                    y_out = add_sub_mul_result.read();
                    break;

                case FUNCT3_AND_REMU:
                    y_out = and_remu_result.read();
                    break;

                case FUNCT3_OR_REM:
                    y_out = or_rem_result.read();
                    break;

                case FUNCT3_XOR_DIV:
                    y_out = xor_div_result.read();
                    break;

                case FUNCT3_SLT_MULHSU:
                    y_out = slt_mulhsu_result.read();
                    break;

                case FUNCT3_SLTU_MULHU:
                    y_out = sltu_mulhu_result.read();
                    break;

                case FUNCT3_SRL_SRA_DIVU:
                    y_out = srl_sra_divu_result.read();
                    break;

                case FUNCT3_SLL_MULH:
                    y_out = sll_mulh_result.read();
                    break;

                default:
                    y_out = 0x0;
                    PN_ERROR("ALU: Unknown operation");
                    break;
            }
            break;

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

    /* Force AMO -> use function decoded in funct5 */
    if(force_amo_in.read() == 0x1)
    {
        // This switch is separated from the other switch as min(u), max(u) operations are unique to AMOs
        switch(funct7_in.read().range(6, 2))
        {
            case FUNCT5A_SWAP:
                y_out = b_in.read();
                break;
            case FUNCT5A_ADD:
                y_out = add_sub_mul_result.read();
                break;

            case FUNCT5A_AND:
                y_out = and_remu_result.read();
                break;

            case FUNCT5A_OR:
                y_out = or_rem_result.read();
                break;

            case FUNCT5A_XOR:
                y_out = xor_div_result.read();
                break;

            case FUNCT5A_MAX:
                y_out = max_result.read();
                break;

            case FUNCT5A_MIN:
                y_out = min_result.read();
                break;

            case FUNCT5A_MAXU:
                y_out = maxu_result.read();
                break;

            case FUNCT5A_MINU:
                y_out = minu_result.read();
                break;

            default:
                break;
        }
    }

    /* Force add Flag - if set, force add operation */
    if(force_add_in.read() == 0x1)
    {
        y_out = a_in.read() + b_in.read();
    }
}

void m_alu::init_submodules()
{
#if PN_CFG_ALU_ENABLE_ZMMUL_EXTENSION == 1
    // change this assign according to the manual, if you want to replace
    // the multiplication module.
    multiplication_module = sc_new<m_shift_and_add>("shift_and_add");
    multiplication_module->clk(clk);
    multiplication_module->reset(reset);
    multiplication_module->start_in(mul_start);
    multiplication_module->a_in(a_in);
    multiplication_module->b_in(b_in);
    multiplication_module->funct3_in(funct3_in);
    multiplication_module->valid_out(mul_valid);
    multiplication_module->y_out(mul_result);
#endif
#if PN_CFG_ALU_ENABLE_M_EXTENSION == 1
    // change this assign according to the manual, if you want to replace
    // the division module.
    division_module = sc_new<m_shift_and_subtract>("shift_and_subtract");
    division_module->clk(clk);
    division_module->reset(reset);
    division_module->start_in(div_start);
    division_module->a_in(a_in);
    division_module->b_in(b_in);
    division_module->funct3_in(funct3_in);
    division_module->valid_out(div_valid);
    division_module->y_out(div_rem_result);
#endif
#if PN_CFG_ENABLE_ZKNE_ZKND == 1
    scalar_crypto = sc_new<m_scalar_crypto>("scalar_crypto");
    scalar_crypto->clk(clk);
    scalar_crypto->reset(reset);
    scalar_crypto->start_in(crypto_start);
    scalar_crypto->rs1_in(a_in);
    scalar_crypto->rs2_in(b_in);
    scalar_crypto->funct3_in(funct3_in);
    scalar_crypto->funct7_in(funct7_in);
    scalar_crypto->valid_out(crypto_valid);
    scalar_crypto->res_out(crypto_result);
#endif
}