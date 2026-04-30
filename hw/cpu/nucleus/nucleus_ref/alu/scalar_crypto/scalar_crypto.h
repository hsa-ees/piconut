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

#ifndef __CRYPTO_H__
#define __CRYPTO_H__

#include <systemc.h>
#include <piconut.h>

/**
 * @fn SC_MODULE(m_scalar_crypto)
 * @brief Scalar Cryptography Module implementing parts of the RISC-V Scalar Cryptography Extension.
 *
 * At the moment, this module supports AES encryption and decryption instructions as defined as
 *  Zkne and Zknd extensions for RV32.
 *  This module can be extended to support additional scalar cryptographic instructions.
 *
 * This module implements a simple state machine to interact with the ALU similar to other ALU modules
 *  like multiplication.
 *
 * @par Ports:
 * @param[in] clk Clock of the module.
 * @param[in] reset Reset of the module.
 * @param[in] rs1_in First source register input.
 * @param[in] rs2_in Second source register input.
 * @param[in] funct7_in Function 7 field input.
 * @param[in] funct3_in Function 3 field input.
 * @param[in] start_in Start signal input.
 * @param[out] valid_out Output signal indicating valid result.
 * @param[out] res_out Result output.
 *
 */
SC_MODULE(m_scalar_crypto)
{
public:
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    sc_in<sc_uint<32>> PN_NAME(rs1_in);
    sc_in<sc_uint<32>> PN_NAME(rs2_in);
    sc_in<sc_uint<7>> PN_NAME(funct7_in);
    sc_in<sc_uint<3>> PN_NAME(funct3_in);
    sc_in<bool> PN_NAME(start_in);

    sc_out<bool> PN_NAME(valid_out);
    sc_out<sc_uint<32>> PN_NAME(res_out);

    sc_signal<sc_uint<32>> PN_NAME(blk_res);

    /* Constructor... */
    SC_CTOR(m_scalar_crypto)
    {
        SC_CTHREAD(proc_clk_crypto, clk.pos());
        reset_signal_is(reset, true);

        SC_METHOD(proc_cmb_state);
        sensitive << state << funct7_in << start_in << blk_res;

        SC_METHOD(proc_cmb_rs2_sel);
        sensitive << rs2_in << funct7_in;

        SC_METHOD(proc_cmb_aes);
        sensitive << funct7_in << rs2_sel << funct7_in;

        SC_METHOD(proc_cmb_block_res);
        sensitive << rs1_in << rotated_res;

        SC_METHOD(proc_cmb_rotate);
        sensitive << reg_aes_raw_out << funct7_in;
    }

    void pn_trace(sc_trace_file * tf, int level = 1);

protected:

    void proc_clk_crypto();
    /**
     * @brief Combinatorial state machine process
     */
    void proc_cmb_state();
    /**
     * @brief aes32*s up to xor with rs1
     */
    void proc_cmb_aes();
    /**
     * @brief Select byte from rs2 based on byte-select in funct7
     */
    void proc_cmb_rs2_sel();
    /**
     * @brief Rotate the block cipher result (back)
     */
    void proc_cmb_rotate();
    /**
     * @brief Final block result after xor with rs1
     */
    void proc_cmb_block_res();

    enum crypto_state_t
    {
        S_IDLE = 0,
        S_BUSY,
        S_DONE,
    };

    sc_signal<sc_uint<2>> state;
    sc_signal<sc_uint<2>> next_state;

    sc_signal<sc_uint<32>> reg_aes_raw_out;
    sc_signal<sc_uint<32>> next_aes_raw_out;

    // Block ciphers
    sc_signal<sc_uint<8>> rs2_sel;
    sc_signal<sc_uint<32>> rotated_res;
};

#endif //__CRYPTO_H__