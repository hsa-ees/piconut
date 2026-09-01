/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2026 Daniel Sommerfeldt <daniel.sommerfeldt1@tha.de>
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

#ifndef __MUL_ADD_FLOAT_H__
#define __MUL_ADD_FLOAT_H__

#include <systemc.h>
#include <piconut.h>

/**
 * @fn SC_MODULE(m_mul_add_float)
 * @brief Floating-point fused multiply-add module.
 * 
 * This module performs floating-point fused multiply-add operations on 
 * three 32-bit floating-point numbers, complying with IEEE 754 standards and 
 * configurable rounding modes.
 * @par Ports:
 * @param[in] clk Clock of the module.
 * @param[in] reset Reset of the module.
 * @param[in] stb_in Strobe signal; when high, it initiates the floating-point operation.
 * @param[in] a_in 32-bit floating-point operand A.
 * @param[in] b_in 32-bit floating-point operand B.
 * @param[in] c_in 32-bit floating-point operand C.
 * @param[in] rounding_mode_in 3-bit code specifying the IEEE 754 rounding mode (e.g., Round to Nearest).
 * @param[out] result_out 32-bit floating-point result of the fused multiply-add.
 * @param[out] ready_out Status signal indicating the module is free to accept a new operation.
 * @param[out] done_out Status signal indicating the current operation is complete and the result is valid.
 */
SC_MODULE(m_mul_add_float) {
    public:
        sc_in_clk PN_NAME(clk);
        sc_in<bool> PN_NAME(reset);

        sc_in<bool> PN_NAME(stb_in);

        sc_in<sc_uint<32>> PN_NAME(a_in);
        sc_in<sc_uint<32>> PN_NAME(b_in);
        sc_in<sc_uint<32>> PN_NAME(c_in);
        sc_in<bool> PN_NAME(negate_in);
        sc_in<sc_uint<3>> PN_NAME(rounding_mode_in);

        sc_out<sc_uint<32>> PN_NAME(result_out);

        sc_out<bool> PN_NAME(ready_out);
        sc_out<bool> PN_NAME(done_out);

        SC_CTOR(m_mul_add_float) {
            SC_CTHREAD(proc_clk_mul_add_float, clk.pos());
            reset_signal_is(reset, true);
        }

        sc_uint<1> sign(sc_uint<32> float_in) { return float_in.range(31, 31); }
        sc_uint<8> exponent(sc_uint<32> float_in) { return float_in.range(30, 23); }
        sc_uint<23> mantissa(sc_uint<32> float_in) { return float_in.range(22, 0); }

        void pn_trace(sc_trace_file *tf, int level = 1);
        void proc_clk_mul_add_float();

    protected:
        sc_signal<sc_uint<4>> PN_NAME(state);

        sc_signal<sc_uint<1>>  PN_NAME(reg_init_a_sign);
        sc_signal<sc_uint<1>>  PN_NAME(reg_init_b_sign);
        sc_signal<sc_uint<1>>  PN_NAME(reg_init_c_sign);
        sc_signal<sc_uint<8>>  PN_NAME(reg_init_a_exp);
        sc_signal<sc_uint<8>>  PN_NAME(reg_init_b_exp);
        sc_signal<sc_uint<8>>  PN_NAME(reg_init_c_exp);
        sc_signal<sc_uint<23>> PN_NAME(reg_init_a_mant);
        sc_signal<sc_uint<23>> PN_NAME(reg_init_b_mant);
        sc_signal<sc_uint<23>> PN_NAME(reg_init_c_mant);
        sc_signal<sc_uint<3>>  PN_NAME(reg_init_rm);
        sc_signal<bool>        PN_NAME(reg_init_negate);

        sc_signal<sc_uint<1>>  PN_NAME(reg_a_sign);
        sc_signal<sc_uint<1>>  PN_NAME(reg_b_sign);
        sc_signal<sc_uint<1>>  PN_NAME(reg_c_sign);
        sc_signal<sc_uint<8>>  PN_NAME(reg_a_exp);
        sc_signal<sc_uint<8>>  PN_NAME(reg_b_exp);
        sc_signal<sc_uint<8>>  PN_NAME(reg_c_exp);
        sc_signal<sc_uint<24>> PN_NAME(reg_a_mant);
        sc_signal<sc_uint<24>> PN_NAME(reg_b_mant);
        sc_signal<sc_uint<24>> PN_NAME(reg_c_mant);
        
        sc_signal<sc_uint<48>> PN_NAME(reg_mul_res);
        sc_signal<int>         PN_NAME(digit);

        sc_signal<sc_biguint<128>> reg_pre_align_mul;
        sc_signal<sc_biguint<128>> reg_pre_align_c;
        sc_signal<int>             reg_align_shift_amt;
        sc_signal<int>             reg_align_target; // 0: none, 1: shift C, 2: shift Mul
        sc_signal<bool>            reg_align_sticky_pre;

        sc_signal<int>  reg_shift_val;
        sc_signal<bool> reg_add_sticky_pass;

        // --- Alignment Stage Output Pipeline Registers ---
        sc_signal<sc_biguint<128>> PN_NAME(reg_aligned_mul);
        sc_signal<sc_biguint<128>> PN_NAME(reg_aligned_c);
        sc_signal<sc_int<10>>  PN_NAME(reg_common_exp);
        sc_signal<bool>        PN_NAME(reg_sticky_bit);

        // --- Addition Stage Output Pipeline Registers ---
        sc_signal<sc_biguint<129>> PN_NAME(reg_sum_res);
        sc_signal<bool>        PN_NAME(reg_final_sign);
        sc_signal<sc_int<10>>  PN_NAME(reg_final_exp);
        sc_signal<bool>        PN_NAME(reg_add_sticky);

        // --- Normalization Stage Output Pipeline Registers ---
        sc_signal<sc_uint<23>> PN_NAME(reg_norm_mant);
        sc_signal<sc_uint<3>>  PN_NAME(reg_round_bits); // Stores packed {G, R, S}
        sc_signal<sc_int<10>>  PN_NAME(reg_final_exp_out);
};

#endif // __MUL_ADD_FLOAT_H__