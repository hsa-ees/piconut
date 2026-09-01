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

#include "compare_float.h"

typedef enum {
    LESS_EQUAL = 0b000,
    LESS_THAN  = 0b001,
    EQUAL      = 0b010,
} e_compare_mode;

void m_compare_float::pn_trace(sc_trace_file* tf, int level) {
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, a_in);
    PN_TRACE(tf, b_in);
    PN_TRACE(tf, mode_in);
    PN_TRACE(tf, data_out);
}

void m_compare_float::proc_clk_compare_float() {
    data_out.write(0);

    while(true) {
        wait();

        data_out.write(0);

        sc_uint<32> a = a_in.read();
        sc_uint<32> b = b_in.read();
        sc_uint<3> mode = mode_in.read();
        sc_uint<1> sign_a = sign(a);
        sc_uint<1> sign_b = sign(b);
        sc_uint<8> exp_a = exponent(a);
        sc_uint<8> exp_b = exponent(b);
        sc_uint<23> mant_a = mantissa(a);
        sc_uint<23> mant_b = mantissa(b);
        bool a_is_nan = (exp_a == 0xFF) && (mant_a != 0);
        bool b_is_nan = (exp_b == 0xFF) && (mant_b != 0);
        bool a_is_zero = (exp_a == 0) && (mant_a == 0);
        bool b_is_zero = (exp_b == 0) && (mant_b == 0);
        bool both_zero = a_is_zero && b_is_zero;

        bool result = false;

        if (a_is_nan || b_is_nan) {
            result = false;
        } else {
            bool eq = (a == b) || both_zero;

            bool lt;
            if (both_zero) {
                lt = false;
            } else if (sign_a != sign_b) {
                lt = (sign_a == 1);
            } else {
                if (sign_a == 0) {
                    lt = (a.range(30, 0) < b.range(30, 0));
                } else {
                    lt = (a.range(30, 0) > b.range(30, 0));
                }
            }

            switch(mode) {
                case EQUAL:
                    result = eq;
                    break;
                case LESS_THAN:
                    result = lt;
                    break;
                case LESS_EQUAL:
                    result = lt || eq;
                    break;
                default:
                    result = false;
                    break;
            }
        }

        data_out.write(result ? 1 : 0);
    }
}