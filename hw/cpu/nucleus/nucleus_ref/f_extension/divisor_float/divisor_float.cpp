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

#include "divisor_float.h"

typedef enum {
    DF_STATE_WAIT,
    DF_STATE_COMP,
    DF_STATE_NORM_SUBNORM,
    DF_STATE_CALC,
    DF_STATE_NORMAL,
    DF_STATE_DENORM,
    DF_STATE_ROUND
} e_divisor_float_states;

void m_divisor_float::pn_trace(sc_trace_file* tf, int level) {
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, stb_in);
    PN_TRACE(tf, a_in);
    PN_TRACE(tf, b_in);
    PN_TRACE(tf, ready_out);
    PN_TRACE(tf, done_out);
    PN_TRACE(tf, result_out);
    PN_TRACE(tf, rounding_mode_in);
}

void m_divisor_float::proc_clk_divisor_float() {
    state.write(DF_STATE_WAIT);
    ready_out.write(1);
    done_out.write(0);
    result_out.write(0);

    reg_init_a_sign.write(0);
    reg_init_b_sign.write(0);
    reg_init_a_exp.write(0);
    reg_init_b_exp.write(0);
    reg_init_a_mant.write(0);
    reg_init_b_mant.write(0);
    reg_init_rm.write(0);

    reg_a_exp.write(0);
    reg_b_exp.write(0);
    reg_a_mant.write(0);
    reg_b_mant.write(0);
    reg_div_res.write(0);
    reg_remainder.write(0);
    reg_count.write(0);

    reg_norm_mant.write(0);
    reg_grs.write(0);
    reg_res_exp.write(0);

    while(true) {
        wait();

        ready_out.write(state.read() == DF_STATE_WAIT);
        done_out.write(0);

        switch(state.read()) {
            case DF_STATE_WAIT: {
                if (stb_in.read()) {
                    reg_init_rm.write(rounding_mode_in.read());
                    
                    sc_uint<32> a = a_in.read();
                    sc_uint<32> b = b_in.read();

                    reg_init_a_sign.write(a.range(31, 31));
                    reg_init_b_sign.write(b.range(31, 31));
                    reg_init_a_exp.write(a.range(30, 23));
                    reg_init_b_exp.write(b.range(30, 23));
                    reg_init_a_mant.write(a.range(22, 0));
                    reg_init_b_mant.write(b.range(22, 0));

                    state.write(DF_STATE_COMP);
                }
                break;
            }

            case DF_STATE_COMP: {
                sc_uint<1>  sign_a = reg_init_a_sign.read();
                sc_uint<1>  sign_b = reg_init_b_sign.read();
                sc_uint<8>  exp_a  = reg_init_a_exp.read();
                sc_uint<8>  exp_b  = reg_init_b_exp.read();
                sc_uint<23> mant_a = reg_init_a_mant.read();
                sc_uint<23> mant_b = reg_init_b_mant.read();
                
                bool a_is_inf  = (exp_a == 0xFF) && (mant_a == 0);
                bool b_is_inf  = (exp_b == 0xFF) && (mant_b == 0);
                bool a_is_nan  = (exp_a == 0xFF) && (mant_a != 0);
                bool b_is_nan  = (exp_b == 0xFF) && (mant_b != 0);
                bool a_is_zero = (exp_a == 0)    && (mant_a == 0);
                bool b_is_zero = (exp_b == 0)    && (mant_b == 0);
                bool a_is_sub  = (exp_a == 0);
                bool b_is_sub  = (exp_b == 0);

                if (a_is_nan || b_is_nan) {
                    sc_uint<32> val_a = (sign_a, exp_a, mant_a);
                    sc_uint<32> val_b = (sign_b, exp_b, mant_b);
                    sc_uint<32> res_nan = a_is_nan ? val_a : val_b;

                    res_nan.range(22, 22) = 1; // Force Quiet NaN
                    result_out.write(res_nan);
                    done_out.write(1);
                    state.write(DF_STATE_WAIT);
                    
                } else if (a_is_zero && b_is_zero) {
                    result_out.write(0xffc00000); 
                    done_out.write(1);
                    state.write(DF_STATE_WAIT);
                    
                } else if (a_is_zero) {
                    sc_uint<32> res = 0;
                    res.range(31, 31) = sign_a ^ sign_b;
                    result_out.write(res);
                    done_out.write(1);
                    state.write(DF_STATE_WAIT);
                    
                } else if (b_is_zero) {
                    sc_uint<32> res = 0x7f800000; 
                    res.range(31, 31) = sign_a ^ sign_b;
                    result_out.write(res);
                    done_out.write(1);
                    state.write(DF_STATE_WAIT);
                    
                } else if (a_is_inf && b_is_inf) {
                    result_out.write(0xffc00000); 
                    done_out.write(1);
                    state.write(DF_STATE_WAIT);
                    
                } else if (a_is_inf) {
                    sc_uint<32> res = 0x7f800000; 
                    res.range(31, 31) = sign_a ^ sign_b;
                    result_out.write(res);
                    done_out.write(1);
                    state.write(DF_STATE_WAIT);
                    
                } else if (b_is_inf) {
                    sc_uint<32> res = 0;
                    res.range(31, 31) = sign_a ^ sign_b;
                    result_out.write(res);
                    done_out.write(1);
                    state.write(DF_STATE_WAIT);
                    
                } else {
                    sc_int<10> calc_exp_a = a_is_sub ? sc_int<10>(1) : sc_int<10>(exp_a);
                    sc_int<10> calc_exp_b = b_is_sub ? sc_int<10>(1) : sc_int<10>(exp_b);
                    sc_uint<24> calc_mant_a = (sc_uint<1>(!a_is_sub), mant_a);
                    sc_uint<24> calc_mant_b = (sc_uint<1>(!b_is_sub), mant_b);

                    reg_a_mant.write(calc_mant_a);
                    reg_b_mant.write(calc_mant_b);
                    reg_a_exp.write(calc_exp_a);
                    reg_b_exp.write(calc_exp_b);

                    state.write(DF_STATE_NORM_SUBNORM);
                }
                break;
            }

            case DF_STATE_NORM_SUBNORM: {
                sc_uint<24> mant_a = reg_a_mant.read();
                sc_uint<24> mant_b = reg_b_mant.read();
                sc_int<10>  exp_a  = reg_a_exp.read();
                sc_int<10>  exp_b  = reg_b_exp.read();

                bool a_needs_norm = (mant_a.range(23, 23) == 0) && (mant_a != 0);
                bool b_needs_norm = (mant_b.range(23, 23) == 0) && (mant_b != 0);

                if (a_needs_norm) {
                    mant_a = mant_a << 1;
                    exp_a  = exp_a - 1;
                }
                if (b_needs_norm) {
                    mant_b = mant_b << 1;
                    exp_b  = exp_b - 1;
                }

                reg_a_mant.write(mant_a);
                reg_b_mant.write(mant_b);
                reg_a_exp.write(exp_a);
                reg_b_exp.write(exp_b);

                if (!a_needs_norm && !b_needs_norm) {
                    reg_remainder.write((sc_uint<25>)mant_a);
                    reg_div_res.write(0);
                    reg_count.write(26);
                    reg_res_exp.write(exp_a - exp_b + 127); 
                    state.write(DF_STATE_CALC);
                } 
                break;
            }

            case DF_STATE_CALC: {
                sc_uint<25> rem     = reg_remainder.read(); 
                sc_uint<27> res     = reg_div_res.read();
                sc_uint<24> divisor = reg_b_mant.read();
                sc_uint<5>  count   = reg_count.read();

                if (count > 0) {
                    if (rem >= divisor) {
                        rem = rem - divisor;
                        res = (res << 1) | 1; 
                    } else {
                        res = (res << 1) | 0; 
                    }

                    if (count > 1) {
                        rem = (rem << 1);
                    }

                    reg_remainder.write(rem);
                    reg_div_res.write(res);
                    reg_count.write(count - 1);
                } else {
                    bool sticky = (rem != 0);
                    res = (res << 1) | sticky;
                    
                    reg_div_res.write(res);
                    state.write(DF_STATE_NORMAL);
                }
                break;
            }

            case DF_STATE_NORMAL: {
                sc_uint<27> res = reg_div_res.read();
                sc_int<10> exp_diff = reg_res_exp.read();

                sc_uint<24> norm_mantissa;
                sc_uint<3>  grs;

                if (res.range(26, 26) == 1) {
                    norm_mantissa = res.range(26, 3);
                    grs = res.range(2, 0);
                } else {
                    norm_mantissa = res.range(25, 2);
                    exp_diff = exp_diff - 1;
                    grs.range(2, 2) = res.range(1, 1);
                    grs.range(1, 1) = 0;
                    grs.range(0, 0) = res.range(0, 0);
                }

                reg_norm_mant.write(norm_mantissa);
                reg_grs.write(grs);
                reg_res_exp.write(exp_diff);

                // We calculate the shift amount HERE and pass it across the clock boundary.
                // This breaks the Subtractor -> Comparator -> Barrel Shifter dependency!
                if (exp_diff <= 0) {
                    sc_uint<10> shift_amt = 1 - exp_diff;
                    reg_count.write((shift_amt >= 26) ? (sc_uint<5>)26 : (sc_uint<5>)shift_amt);
                }

                state.write(DF_STATE_DENORM);
                break;
            }

            case DF_STATE_DENORM: {
                sc_int<10> exp = reg_res_exp.read();
                
                if (exp > 0) {
                    state.write(DF_STATE_ROUND);
                } else {
                    sc_uint<24> mant = reg_norm_mant.read();
                    sc_uint<3>  grs  = reg_grs.read();
                    sc_uint<5>  safe_shift = reg_count.read(); // Read the pre-calculated shift!
        
                    sc_uint<27> combined = (mant, grs); 
                    sc_uint<27> mask = (((sc_uint<27>)1) << safe_shift) - 1;
                    bool sticky_out = (combined & mask) != 0;
                    combined >>= safe_shift;
        
                    mant = combined.range(26, 3);
                    grs  = combined.range(2, 0);
                    grs.range(0, 0) = grs.range(0, 0) | sticky_out;
        
                    reg_norm_mant.write(mant);
                    reg_grs.write(grs);
                    reg_res_exp.write(0);
                    
                    state.write(DF_STATE_ROUND);
                }
                break;
            }

            case DF_STATE_ROUND: {
                sc_uint<1>  sign_a = reg_init_a_sign.read();
                sc_uint<1>  sign_b = reg_init_b_sign.read();
                sc_uint<24> mant   = reg_norm_mant.read();
                sc_int<10>  exp    = reg_res_exp.read();
                sc_uint<3>  grs    = reg_grs.read();
                
                bool g = grs.range(2, 2);
                bool r = grs.range(1, 1);
                bool s = grs.range(0, 0);
                bool lsb = mant.range(0, 0);

                bool round_up = false;
                sc_uint<3> rm = reg_init_rm.read();
                bool res_sign = sign_a ^ sign_b;
                bool any_fraction = (g || r || s);

                switch(rm) {
                    case 0: round_up = g && (lsb || r || s); break;
                    case 1: round_up = false; break;
                    case 2: round_up = res_sign && any_fraction; break;
                    case 3: round_up = !res_sign && any_fraction; break;
                    case 4: round_up = g; break;
                    default: round_up = false; break;
                }

                if (round_up) {
                    if (mant == 0xFFFFFF) {
                        mant = 0x800000; 
                        exp++;
                    } else {
                        mant++;
                        if (exp == 0 && mant.range(23, 23) == 1) {
                            exp = 1;
                        }
                    }
                }

                sc_uint<32> res = 0;
                res.range(31, 31) = res_sign;

                if (exp >= 255) {
                    bool overflow_to_inf = false;
                    switch(rm) {
                        case 0: case 4: overflow_to_inf = true; break;
                        case 1: overflow_to_inf = false; break;
                        case 2: overflow_to_inf = (res_sign == 1); break;
                        case 3: overflow_to_inf = (res_sign == 0); break;
                        default: overflow_to_inf = true; break;
                    }

                    if (overflow_to_inf) {
                        res.range(30, 23) = 0xFF;
                        res.range(22, 0) = 0;
                    } else {
                        res.range(30, 23) = 0xFE; 
                        res.range(22, 0) = 0x7FFFFF;
                    }
                } else {
                    res.range(30, 23) = (sc_uint<8>)exp;
                    res.range(22, 0) = mant;
                }

                result_out.write(res);
                done_out.write(1);
                state.write(DF_STATE_WAIT);
                break;
            }

            default:
                state.write(DF_STATE_WAIT);
                break;
        }
    }
}