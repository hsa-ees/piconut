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

#ifndef __CONVERTER_FLOAT_H__
#define __CONVERTER_FLOAT_H__

#include <systemc.h>
#include <piconut.h>

typedef enum {
    UI32_TO_F32,
    I32_TO_F32,
    F32_TO_UI32,
    F32_TO_I32,
    FSGNJ
} e_converter_float_mode;

/**
 * @fn SC_MODULE(m_converter_float)
 * @brief Floating-point conversion and sign-injection module.
 * This module handles data conversions between 32-bit integers (signed/unsigned) 
 * and 32-bit single-precision floating-point numbers, as well as floating-point 
 * sign-injection operations, complying with IEEE 754 standards.
 * @par Ports:
 * @param[in] clk Clock of the module.
 * @param[in] reset Reset of the module.
 * @param[in] stb_in Strobe signal; when high, it initiates the conversion or sign-injection process.
 * @param[in] a_in 32-bit input operand A (interpreted as an integer or float based on cvt_mode_in).
 * @param[in] b_in 32-bit input operand B (primarily used as the sign source during FSGNJ operations).
 * @param[in] cvt_mode_in 3-bit selection flag to dictate the operation type.
 * @param[in] rounding_mode_in 3-bit code specifying the IEEE 754 rounding mode for integer-to-float or float-to-integer conversions.
 * @param[out] result_out 32-bit output containing the converted result or sign-injected floating-point value.
 * @param[out] ready_out Status signal indicating the converter is free to accept a new operation request.
 * @param[out] done_out Status signal indicating the conversion is complete and the output data is valid.
 */
SC_MODULE(m_converter_float) {
    public:
        sc_in_clk PN_NAME(clk);
        sc_in<bool> PN_NAME(reset);

        sc_in<bool> PN_NAME(stb_in);

        sc_in<sc_uint<32>> PN_NAME(a_in);
        sc_in<sc_uint<32>> PN_NAME(b_in);
        sc_in<sc_uint<3>> PN_NAME(cvt_mode_in);
        sc_in<sc_uint<3>> PN_NAME(rounding_mode_in);

        sc_out<sc_uint<32>> PN_NAME(result_out);

        sc_out<bool> PN_NAME(ready_out);
        sc_out<bool> PN_NAME(done_out);

        SC_CTOR(m_converter_float) {
            SC_CTHREAD(proc_clk_converter_float, clk.pos());
            reset_signal_is(reset, true);
        }

        sc_uint<1> sign(sc_uint<32> float_in) { return float_in.range(31, 31); }
        sc_uint<8> exponent(sc_uint<32> float_in) { return float_in.range(30, 23); }
        sc_uint<23> mantissa(sc_uint<32> float_in) { return float_in.range(22, 0); }

        void pn_trace(sc_trace_file *tf, int level = 1);
        void proc_clk_converter_float();

    protected:
        sc_signal<sc_uint<3>> PN_NAME(state);
        
        // --- Input capture registers ---
        sc_signal<sc_uint<32>> PN_NAME(reg_a);
        sc_signal<sc_uint<32>> PN_NAME(reg_b);
        sc_signal<sc_uint<4>>  PN_NAME(reg_cvt_mode);
        sc_signal<sc_uint<3>>  PN_NAME(reg_rm);

        // --- Extracted floating-point attributes ---
        sc_signal<sc_uint<1>>  PN_NAME(reg_sign);
        sc_signal<sc_uint<8>>  PN_NAME(reg_exp);
        sc_signal<sc_uint<23>> PN_NAME(reg_mant);
        sc_signal<sc_int<10>>  PN_NAME(reg_shift);

        // --- Intermediate datapath registers ---
        sc_signal<bool>        PN_NAME(reg_invalid);
        sc_signal<sc_uint<32>> PN_NAME(reg_mag);
        
        // --- Rounding state registers ---
        sc_signal<sc_uint<1>>  PN_NAME(reg_g);
        sc_signal<sc_uint<1>>  PN_NAME(reg_r);
        sc_signal<sc_uint<1>>  PN_NAME(reg_s);
};

#endif // __CONVERTER_FLOAT_H__