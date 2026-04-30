/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Niklas Sirch <niklas.sirch1@tha.de>
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

#include "scalar_crypto.h"
#include "aes_helpers.h"
#include "../../nucleus_ref_defs.h"

void m_scalar_crypto::pn_trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, rs1_in);
    PN_TRACE(tf, rs2_in);
    PN_TRACE(tf, state);
    PN_TRACE(tf, next_state);
    PN_TRACE(tf, valid_out);
    PN_TRACE(tf, res_out);
    PN_TRACE(tf, rs2_sel);
    PN_TRACE(tf, rotated_res);
    PN_TRACE(tf, reg_aes_raw_out);
    PN_TRACE(tf, next_aes_raw_out);
}

/**
 * @brief Extract byte-select from funct7
 */
sc_uint<2> i_bs(sc_uint<7> funct7)
{
    return funct7.range(6, 5);
}

/**
 * @brief Check if AES decrypt operation
 */
bool i_aes_is_dec(sc_uint<7> funct7)
{
    return funct7[2];
}

/**
 * @brief Check if AES middle round operation
 */
bool i_aes_is_middle(sc_uint<7> funct7)
{
    return funct7[1];
}

bool i_is_aes(sc_uint<7> funct7)
{
    sc_uint<5> funct7_lower = funct7.range(4, 0);

    return (funct7_lower == FUNCT7L_AES32ESMI || funct7_lower == FUNCT7L_AES32ESI ||
            funct7_lower == FUNCT7L_AES32DSMI || funct7_lower == FUNCT7L_AES32DSI);
}

void m_scalar_crypto::proc_cmb_state()
{
    next_state = state.read();
    res_out = blk_res.read();
    valid_out = 0;

    switch(state.read())
    {
        case S_IDLE:
            if(start_in.read())
            {
                next_state = S_BUSY;
            }
            break;

        case S_BUSY:
            next_state = S_DONE;
            valid_out = 1;
            break;

        case S_DONE:
            // done and already overwritten by regfile in next cycle; so no valid
            next_state = S_IDLE;
            break;

        default:
            next_state = S_IDLE;
            break;
    }
}

void m_scalar_crypto::proc_clk_crypto()
{
    state = S_IDLE;
    reg_aes_raw_out = 0;

    while(true)
    {
        wait();

        state = next_state;
        reg_aes_raw_out = next_aes_raw_out;
    }
}

void m_scalar_crypto::proc_cmb_rs2_sel()
{
    switch(funct7_in.read().range(6, 5))
    {
        case 0b00:
            rs2_sel = rs2_in.read().range(7, 0);
            break;
        case 0b01:
            rs2_sel = rs2_in.read().range(15, 8);
            break;
        case 0b10:
            rs2_sel = rs2_in.read().range(23, 16);
            break;
        default: // 0b11
            rs2_sel = rs2_in.read().range(31, 24);
            break;
    }
}

void m_scalar_crypto::proc_cmb_rotate()
{
    sc_uint<32> rol_in = reg_aes_raw_out.read();

    switch(i_bs(funct7_in))
    {
        case 0b00:
            rotated_res = rol_in;
            break;
        case 0b01:
            rotated_res = (rol_in.range(23, 0), rol_in.range(31, 24));
            break;
        case 0b10:
            rotated_res = (rol_in.range(15, 0), rol_in.range(31, 16));
            break;
        default: // 0b11
            rotated_res = (rol_in.range(7, 0), rol_in.range(31, 8));
            break;
    }
}

/**
 * @brief AES S-Box lookup or inverse S-Box lookup
 */
sc_uint<8> aes_sbox_lookup(sc_uint<8> rs2_sel, bool is_decrypt)
{
    sc_uint<9> idx = (is_decrypt, rs2_sel);

    return aes_sbox[idx];
}

/**
 * @brief MixColumns transformation for AES
 */
sc_uint<32> aes_mix_columns(sc_uint<8> aes_so, bool is_decrypt)
{
    sc_uint<32> m;

    if(is_decrypt)
    {
        /**
         * Multiply by fixed matrix |0E 09 0D 0B| for each column
         */
        m.range(31, 24) = gf256_mul(aes_so, 0xb);
        m.range(23, 16) = gf256_mul(aes_so, 0xd);
        m.range(15, 8) = gf256_mul(aes_so, 0x9);
        m.range(7, 0) = gf256_mul(aes_so, 0xe);
    }
    else
    {
        /**
         * Multiply by fixed matrix |02 03 01 01| for each column
         */
        m.range(31, 24) = gf256_mul(aes_so, 0x3);
        m.range(23, 16) = aes_so;
        m.range(15, 8) = aes_so;
        m.range(7, 0) = gf256_mul(aes_so, 0x2);
    }
    return m;
}

void m_scalar_crypto::proc_cmb_aes()
{
    bool is_decrypt = i_aes_is_dec(funct7_in);
    sc_uint<8> aes_so = aes_sbox_lookup(rs2_sel, is_decrypt);
    sc_uint<32> aes_mixed = aes_mix_columns(aes_so, is_decrypt);

    if(i_aes_is_middle(funct7_in))
    {
        next_aes_raw_out = aes_mixed;
    }
    else
    {
        next_aes_raw_out = (sc_uint<32>(aes_so));
    }
}

void m_scalar_crypto::proc_cmb_block_res()
{
    blk_res = rs1_in.read() ^ rotated_res.read();
}
