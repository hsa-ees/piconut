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

#include "sqrt_float.h"

typedef enum {
    SF_STATE_WAIT,
    SF_STATE_COMP,
    SF_STATE_CALC,
    SF_STATE_ROUND,
    SF_STATE_PACK
} e_sqrt_float_states;

void m_sqrt_float::pn_trace(sc_trace_file* tf, int level) {
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, stb_in);
    PN_TRACE(tf, a_in);
    PN_TRACE(tf, rounding_mode_in);
    PN_TRACE(tf, result_out);
    PN_TRACE(tf, ready_out);
    PN_TRACE(tf, done_out);
    PN_TRACE(tf, state);
    PN_TRACE(tf, reg_radicand);
    PN_TRACE(tf, reg_rem);
    PN_TRACE(tf, reg_q);
    PN_TRACE(tf, reg_iter);
    PN_TRACE(tf, reg_result_mant);
    PN_TRACE(tf, reg_result_exp);
}

void m_sqrt_float::proc_clk_sqrt_float() {
    // ---- Reset values ----
    state.write(SF_STATE_WAIT);
    ready_out.write(1);
    done_out.write(0);
    result_out.write(0);

    reg_init_sign.write(0);
    reg_init_exp.write(0);
    reg_init_mant.write(0);
    reg_init_rm.write(0);

    reg_radicand.write(0);
    reg_rem.write(0);
    reg_q.write(0);
    reg_iter.write(0);
    reg_new_exp.write(0);

    reg_result_mant.write(0);
    reg_result_exp.write(0);
    reg_g.write(0);
    reg_r.write(0);
    reg_s.write(0);

    while (true) {
        wait();

        // Default outputs
        ready_out.write(state.read() == SF_STATE_WAIT);
        done_out.write(0);

        switch (state.read()) {

            // -----------------------------------------------------------
            case SF_STATE_WAIT: {
                if (stb_in.read()) {
                    sc_uint<32> a = a_in.read();
                    reg_init_sign.write(a.range(31, 31));
                    reg_init_exp .write(a.range(30, 23));
                    reg_init_mant.write(a.range(22,  0));
                    reg_init_rm  .write(rounding_mode_in.read());
                    state.write(SF_STATE_COMP);
                }
                break;
            }

            // -----------------------------------------------------------
            // Special-case handling + (sub)normal classification.
            // Sets up the 54-bit radicand and the biased result exponent.
            case SF_STATE_COMP: {
                sc_uint<1>  s_in = reg_init_sign.read();
                sc_uint<8>  e_in = reg_init_exp.read();
                sc_uint<23> m_in = reg_init_mant.read();

                bool is_zero = (e_in == 0)    && (m_in == 0);
                bool is_inf  = (e_in == 0xFF) && (m_in == 0);
                bool is_nan  = (e_in == 0xFF) && (m_in != 0);
                bool is_sub  = (e_in == 0)    && (m_in != 0);
                bool is_neg  = (s_in == 1);

                if (is_nan) {
                    // sqrt(NaN) -> propagate as quiet NaN
                    sc_uint<32> res = (s_in, e_in, m_in);
                    res[22] = 1;
                    result_out.write(res);
                    done_out.write(1);
                    state.write(SF_STATE_WAIT);

                } else if (is_zero) {
                    // sqrt(+/-0) = +/-0  (sign preserved per IEEE 754)
                    sc_uint<32> res = 0;
                    res.range(31, 31) = s_in;
                    result_out.write(res);
                    done_out.write(1);
                    state.write(SF_STATE_WAIT);

                } else if (is_neg) {
                    // sqrt of negative non-zero -> quiet NaN
                    result_out.write(0xFFC00000);
                    done_out.write(1);
                    state.write(SF_STATE_WAIT);

                } else if (is_inf) {
                    // sqrt(+inf) = +inf
                    result_out.write(0x7F800000);
                    done_out.write(1);
                    state.write(SF_STATE_WAIT);

                } else {
                    // -------------------------------------------------
                    // Normal / sub-normal: build 24-bit normalized mantissa
                    // and signed unbiased exponent (range -149 .. +128).
                    // -------------------------------------------------
                    sc_uint<24> m_ext;
                    sc_int<10>  exp_unb;

                    if (!is_sub) {
                        m_ext   = (sc_uint<1>(1), m_in);          // implicit 1
                        exp_unb = (sc_int<10>)e_in - 127;
                    } else {
                        sc_uint<5> lz = 22;
                        if (m_in.range(22, 12) != 0) {
                            if (m_in.range(22, 18) != 0) {
                                if (m_in.range(22, 20) != 0) {
                                    if (m_in.range(22, 21) != 0) lz = m_in[22] ? 0 : 1;
                                    else                          lz = 2;
                                } else {
                                    lz = m_in[19] ? 3 : 4;
                                }
                            } else {
                                if (m_in.range(17, 15) != 0) {
                                    if (m_in.range(17, 16) != 0) lz = m_in[17] ? 5 : 6;
                                    else                          lz = 7;
                                } else {
                                    if (m_in.range(14, 13) != 0) lz = m_in[14] ? 8 : 9;
                                    else                          lz = 10;
                                }
                            }
                        } else if (m_in.range(11, 4) != 0) {
                            if (m_in.range(11, 8) != 0) {
                                if (m_in.range(11, 10) != 0) lz = m_in[11] ? 11 : 12;
                                else                          lz = m_in[9]  ? 13 : 14;
                            } else {
                                if (m_in.range(7, 6) != 0) lz = m_in[7] ? 15 : 16;
                                else                        lz = m_in[5] ? 17 : 18;
                            }
                        } else {
                            if (m_in.range(3, 2) != 0) lz = m_in[3] ? 19 : 20;
                            else                        lz = m_in[1] ? 21 : 22;
                        }

                        // shift leading 1 up to bit 23 of m_ext
                        sc_uint<24> tmp = (sc_uint<1>(0), m_in);
                        m_ext   = tmp << (lz + 1);
                        exp_unb = (sc_int<10>)(-127) - (sc_int<10>)lz;
                    }

                    bool        odd = (exp_unb[0] == 1);
                    sc_uint<54> rad;
                    sc_int<10>  half_exp;

                    if (!odd) {
                        rad      = ((sc_uint<54>)m_ext) << 29;
                        half_exp = exp_unb >> 1;                  // arithmetic
                    } else {
                        rad      = ((sc_uint<54>)m_ext) << 30;
                        half_exp = (exp_unb - 1) >> 1;            // floor for odd
                    }

                    reg_radicand.write(rad);
                    reg_rem.write(0);
                    reg_q.write(0);
                    reg_iter.write(0);
                    reg_new_exp.write(half_exp + 127);            // re-bias

                    state.write(SF_STATE_CALC);
                }
                break;
            }

            case SF_STATE_CALC: {
                sc_uint<5>  i   = reg_iter.read();
                sc_uint<54> rad = reg_radicand.read();
                sc_uint<29> rem = reg_rem.read();
                sc_uint<27> q   = reg_q.read();

                sc_uint<2>   pair = rad.range(53, 52);

                sc_uint<31> shifted_rem = (((sc_uint<31>)rem) << 2) | (sc_uint<31>)pair;
                sc_uint<29> trial       = (((sc_uint<29>)q)   << 2) | sc_uint<29>(1);

                sc_uint<27> next_q;
                sc_uint<29> next_rem;
                if (shifted_rem >= (sc_uint<31>)trial) {
                    next_rem = (sc_uint<29>)(shifted_rem - (sc_uint<31>)trial);
                    next_q   = (q << 1) | sc_uint<27>(1);
                } else {
                    next_rem = (sc_uint<29>)shifted_rem;
                    next_q   = (q << 1);
                }

                reg_rem.write(next_rem);
                reg_q.write(next_q);
                reg_iter.write(i + 1);

                reg_radicand.write(rad << 2);

                if (i == 26) {
                    state.write(SF_STATE_ROUND);
                }
                break;
            }

            case SF_STATE_ROUND: {
                sc_uint<27> q   = reg_q.read();
                sc_uint<29> rem = reg_rem.read();
                sc_int<10>  exp = reg_new_exp.read();
                sc_uint<3>  rm  = reg_init_rm.read();

                sc_uint<24> mant = q.range(26, 3);
                sc_uint<1>  g    = q.range(2, 2);
                sc_uint<1>  r    = q.range(1, 1);
                sc_uint<1>  s_lo = q.range(0, 0);
                sc_uint<1>  s    = (s_lo == 1) || (rem != 0) ? sc_uint<1>(1)
                                                              : sc_uint<1>(0);
                sc_uint<1>  lsb  = mant.range(0, 0);

                bool res_sign = false;
                bool any_frac = (g == 1) || (r == 1) || (s == 1);
                bool round_up = false;

                switch (rm) {
                    case 0: round_up = (g == 1) && ((lsb == 1) || (r == 1) || (s == 1)); break; // RNE
                    case 1: round_up = false;                                             break; // RTZ
                    case 2: round_up = res_sign && any_frac;                              break; // RDN
                    case 3: round_up = !res_sign && any_frac;                             break; // RUP
                    case 4: round_up = (g == 1);                                          break; // RMM
                    default: round_up = false;                                            break;
                }

                if (round_up) {
                    if (mant == 0xFFFFFF) {
                        mant = 0x800000;   // 1.000...0
                        exp++;
                    } else {
                        mant++;
                    }
                }

                reg_result_mant.write(mant);
                reg_result_exp.write(exp);
                reg_g.write(g);
                reg_r.write(r);
                reg_s.write(s);

                state.write(SF_STATE_PACK);
                break;
            }

            case SF_STATE_PACK: {
                sc_uint<24> mant = reg_result_mant.read();
                sc_int<10>  exp  = reg_result_exp.read();

                sc_uint<32> res = 0;
                res.range(31, 31) = 0;
                res.range(30, 23) = (sc_uint<8>)exp;
                res.range(22,  0) = mant.range(22, 0);

                result_out.write(res);
                done_out.write(1);
                state.write(SF_STATE_WAIT);
                break;
            }

            default: {
                state.write(SF_STATE_WAIT);
                break;
            }
        }
    }
}