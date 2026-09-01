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

#include "mul_add_float.h"

typedef enum {
    MAF_STATE_WAIT,
    MAF_STATE_COMP,
    MAF_STATE_MUL,
    MAF_STATE_ALIGN_EXP,
    MAF_STATE_ALIGN_SHIFT,
    MAF_STATE_ADD,
    MAF_STATE_NORM_MSB,
    MAF_STATE_NORM_SHIFT,
    MAF_STATE_ROUND
} e_mul_add_float_states;

void m_mul_add_float::pn_trace(sc_trace_file* tf, int level) {
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, stb_in);
    PN_TRACE(tf, a_in);
    PN_TRACE(tf, b_in);
    PN_TRACE(tf, c_in);
    PN_TRACE(tf, negate_in);
    PN_TRACE(tf, ready_out);
    PN_TRACE(tf, done_out);
    PN_TRACE(tf, result_out);
    PN_TRACE(tf, rounding_mode_in);
    PN_TRACE(tf, state);
}

void m_mul_add_float::proc_clk_mul_add_float() {
    state.write(MAF_STATE_WAIT);
    ready_out.write(1);
    done_out.write(0);
    result_out.write(0);

    reg_init_a_sign.write(0);
    reg_init_b_sign.write(0);
    reg_init_c_sign.write(0);
    reg_init_a_exp.write(0);
    reg_init_b_exp.write(0);
    reg_init_c_exp.write(0);
    reg_init_a_mant.write(0);
    reg_init_b_mant.write(0);
    reg_init_c_mant.write(0);
    reg_init_rm.write(0);
    reg_init_negate.write(0);

    while(true) {
        wait();

        ready_out.write(state.read() == MAF_STATE_WAIT);
        done_out.write(0);

        switch(state.read()) {
            case MAF_STATE_WAIT: {
                if (stb_in.read()) {
                    sc_uint<32> a = a_in.read();
                    sc_uint<32> b = b_in.read();
                    sc_uint<32> c = c_in.read();

                    reg_init_a_sign.write(sign(a));
                    reg_init_b_sign.write(sign(b));
                    reg_init_c_sign.write(sign(c));
                    reg_init_a_exp.write(exponent(a));
                    reg_init_b_exp.write(exponent(b));
                    reg_init_c_exp.write(exponent(c));
                    reg_init_a_mant.write(mantissa(a));
                    reg_init_b_mant.write(mantissa(b));
                    reg_init_c_mant.write(mantissa(c));
                    reg_init_rm.write(rounding_mode_in.read());
                    reg_init_negate.write(negate_in.read());

                    state.write(MAF_STATE_COMP);
                }
                break;
            }

            case MAF_STATE_COMP: {
                sc_uint<1> sign_a = reg_init_a_sign.read();
                sc_uint<1> sign_b = reg_init_b_sign.read();
                sc_uint<1> sign_c = reg_init_c_sign.read();
                sc_uint<8> exp_a  = reg_init_a_exp.read();
                sc_uint<8> exp_b  = reg_init_b_exp.read();
                sc_uint<8> exp_c  = reg_init_c_exp.read();
                sc_uint<23> mant_a = reg_init_a_mant.read();
                sc_uint<23> mant_b = reg_init_b_mant.read();
                sc_uint<23> mant_c = reg_init_c_mant.read();

                bool a_is_inf  = (exp_a == 0xFF) && (mant_a == 0);
                bool b_is_inf  = (exp_b == 0xFF) && (mant_b == 0);
                bool c_is_inf  = (exp_c == 0xFF) && (mant_c == 0);
                bool a_is_nan  = (exp_a == 0xFF) && (mant_a != 0);
                bool b_is_nan  = (exp_b == 0xFF) && (mant_b != 0);
                bool c_is_nan  = (exp_c == 0xFF) && (mant_c != 0);
                bool a_is_zero = (exp_a == 0)    && (mant_a == 0);
                bool b_is_zero = (exp_b == 0)    && (mant_b == 0);
                bool c_is_zero = (exp_c == 0)    && (mant_c == 0);
                bool a_is_sub  = (exp_a == 0);
                bool b_is_sub  = (exp_b == 0);
                bool c_is_sub  = (exp_c == 0);

                // Apply negate_in to the product's sign
                bool mul_sign = sign_a ^ sign_b ^ reg_init_negate.read();
                
                // The product is only infinite if neither contributing operand is NaN.
                bool prod_is_inf = (a_is_inf && !b_is_zero && !b_is_nan) || 
                                   (b_is_inf && !a_is_zero && !a_is_nan);
                
                bool is_invalid_mul = (a_is_zero && b_is_inf) || (a_is_inf && b_is_zero);
                bool is_invalid_add = prod_is_inf && c_is_inf && (mul_sign != sign_c);
                bool is_invalid_op  = is_invalid_mul || is_invalid_add;

                if (a_is_nan || b_is_nan || c_is_nan || is_invalid_op) {
                    sc_uint<32> nan_res;
                    
                    if (is_invalid_op) {
                        // Canonical invalid-operation qNaN
                        nan_res = 0xFFC00000;
                    }
                    else if (a_is_nan) {
                        nan_res =
                            (sign_a << 31) |
                            (exp_a  << 23) |
                            mant_a |
                            0x00400000;
                    }
                    else if (b_is_nan) {
                        nan_res =
                            (sign_b << 31) |
                            (exp_b  << 23) |
                            mant_b |
                            0x00400000;
                    }
                    else if (c_is_nan) {
                        nan_res =
                            (sign_c << 31) |
                            (exp_c  << 23) |
                            mant_c |
                            0x00400000;
                    }
                    
                    result_out.write(nan_res);
                    done_out.write(1);
                    state.write(MAF_STATE_WAIT);
                    break; 
                } 
                else if (a_is_inf || b_is_inf || c_is_inf) {
                    sc_uint<32> inf_res;
                    
                    if (c_is_inf && !a_is_inf && !b_is_inf) {
                        inf_res = (sign_c << 31) | 0x7F800000;
                    } else if ((a_is_inf || b_is_inf) && !c_is_inf) {
                        inf_res = (mul_sign << 31) | 0x7F800000;
                    } else { 
                        inf_res = (sign_c << 31) | 0x7F800000;
                    }
                    
                    result_out.write(inf_res);
                    done_out.write(1);
                    state.write(MAF_STATE_WAIT);
                    break; 
                }

                reg_a_sign.write(sign_a);
                reg_b_sign.write(sign_b);
                reg_c_sign.write(sign_c);
                reg_a_exp.write(a_is_sub ? sc_uint<8>(1) : exp_a);
                reg_b_exp.write(b_is_sub ? sc_uint<8>(1) : exp_b);
                reg_c_exp.write(c_is_sub ? sc_uint<8>(1) : exp_c);
                reg_a_mant.write((sc_uint<1>(!a_is_sub), mant_a));
                reg_b_mant.write((sc_uint<1>(!b_is_sub), mant_b));
                reg_c_mant.write((sc_uint<1>(!c_is_sub), mant_c));
    
                reg_mul_res.write(0);
                digit.write(0);
                state.write(MAF_STATE_MUL);
                break;
            }

            case MAF_STATE_MUL: {
                sc_uint<48> current_res = reg_mul_res.read();
                sc_uint<48> multiplicand = reg_b_mant.read();
                sc_uint<24> a_mant_val = reg_a_mant.read();
                
                int i = digit.read(); 

                bool bit_m1 = (i == 0) ? 0 : a_mant_val.range(i - 1, i - 1);
                bool bit_0  = a_mant_val.range(i, i);
                bool bit_p1 = a_mant_val.range(i + 1, i + 1);

                sc_uint<3> booth_val = (bit_p1 << 2) | (bit_0 << 1) | bit_m1;
                sc_uint<48> shifted_b = multiplicand << i;
                
                switch (booth_val) {
                    case 1: 
                    case 2: current_res += shifted_b; break;
                    case 3: current_res += (shifted_b << 1); break;
                    case 4: current_res -= (shifted_b << 1); break;
                    case 5: 
                    case 6: current_res -= shifted_b; break;
                    case 0: 
                    case 7: break;
                }

                if (i == 22) { 
                    if (bit_p1 == 1) { 
                        current_res += (multiplicand << 24);
                    }
                    state.write(MAF_STATE_ALIGN_EXP);
                } else {
                    digit.write(i + 2); 
                }
                
                reg_mul_res.write(current_res);
                break;
            }

            case MAF_STATE_ALIGN_EXP: {
                sc_biguint<128> wide_mul = (sc_biguint<128>)reg_mul_res.read() << 46; 
                sc_biguint<128> wide_c   = (sc_biguint<128>)reg_c_mant.read() << 69; 

                sc_uint<8> raw_exp_a = reg_init_a_exp.read();
                sc_uint<8> raw_exp_b = reg_init_b_exp.read();
                sc_uint<8> raw_exp_c = reg_init_c_exp.read();
                sc_uint<23> raw_mant_a = reg_init_a_mant.read();
                sc_uint<23> raw_mant_b = reg_init_b_mant.read();
                sc_uint<23> raw_mant_c = reg_init_c_mant.read();

                bool prod_is_zero = ((raw_exp_a == 0) && (raw_mant_a == 0)) || 
                                    ((raw_exp_b == 0) && (raw_mant_b == 0));
                bool c_is_zero    = ((raw_exp_c == 0) && (raw_mant_c == 0));

                sc_int<10> mul_exp_sum = (sc_int<10>)reg_a_exp.read() + (sc_int<10>)reg_b_exp.read() - 127;
                sc_int<10> val_c_exp   = (sc_int<10>)reg_c_exp.read();

                sc_int<10> exp_diff = mul_exp_sum - val_c_exp;

                sc_int<10> final_exp = 0;
                int shift_amt = 0;
                int target = 0; // 0 = none, 1 = shift C, 2 = shift Mul
                bool pre_sticky = false;

                if (c_is_zero && prod_is_zero) {
                    final_exp = 0;
                    wide_mul = 0;
                    wide_c = 0;
                } else if (c_is_zero) {
                    final_exp = mul_exp_sum;
                    wide_c = 0;
                } else if (prod_is_zero) {
                    final_exp = val_c_exp;
                    wide_mul = 0;
                } else if (exp_diff >= 0) {
                    final_exp = mul_exp_sum;
                    if (exp_diff >= 128) {
                        pre_sticky = (wide_c != 0);
                        wide_c = 0; // effectively shifted out of existence
                    } else if (exp_diff > 0) {
                        shift_amt = exp_diff;
                        target = 1;
                    }
                } else {
                    sc_int<10> abs_diff = -exp_diff;
                    final_exp = val_c_exp;
                    if (abs_diff >= 128) {
                        pre_sticky = (wide_mul != 0);
                        wide_mul = 0; // effectively shifted out of existence
                    } else if (abs_diff > 0) {
                        shift_amt = abs_diff;
                        target = 2;
                    }
                }

                reg_pre_align_mul.write(wide_mul);
                reg_pre_align_c.write(wide_c);
                reg_align_shift_amt.write(shift_amt);
                reg_align_target.write(target);
                reg_align_sticky_pre.write(pre_sticky);
                reg_common_exp.write(final_exp); // Read later in MAF_STATE_ADD

                state.write(MAF_STATE_ALIGN_SHIFT);
                break;
            }

            case MAF_STATE_ALIGN_SHIFT: {
                sc_biguint<128> wide_mul = reg_pre_align_mul.read();
                sc_biguint<128> wide_c   = reg_pre_align_c.read();
                int shift_amt            = reg_align_shift_amt.read();
                int target               = reg_align_target.read();
                bool sticky              = reg_align_sticky_pre.read();

                if (target == 1) {
                    sc_biguint<128> mask = ((sc_biguint<128>)1 << shift_amt) - 1;
                    sticky |= ((wide_c & mask) != 0);
                    wide_c >>= shift_amt;
                } else if (target == 2) {
                    sc_biguint<128> mask = ((sc_biguint<128>)1 << shift_amt) - 1;
                    sticky |= ((wide_mul & mask) != 0);
                    wide_mul >>= shift_amt;
                }

                reg_aligned_mul.write(wide_mul);
                reg_aligned_c.write(wide_c);
                reg_sticky_bit.write(sticky);

                state.write(MAF_STATE_ADD);
                break;
            }

            case MAF_STATE_ADD: {
                sc_biguint<128> wide_mul = reg_aligned_mul.read();
                sc_biguint<128> wide_c   = reg_aligned_c.read();
                sc_int<10> common_exp    = reg_common_exp.read();
                bool sticky_align        = reg_sticky_bit.read();

                bool mul_sign = reg_a_sign.read() ^ reg_b_sign.read() ^ reg_init_negate.read(); 
                bool c_sign   = reg_c_sign.read();

                sc_biguint<129> sum_res = 0;
                bool final_sign = false;
                bool effective_subtraction = (mul_sign != c_sign);

                if (!effective_subtraction) {
                    sum_res = (sc_biguint<129>)wide_mul + (sc_biguint<129>)wide_c;
                    final_sign = mul_sign; 
                } else {
                    if (wide_mul >= wide_c) {
                        sum_res = (sc_biguint<129>)wide_mul - (sc_biguint<129>)wide_c;
                        if (sticky_align) sum_res -= 1;
                        final_sign = mul_sign;
                    } else {
                        sum_res = (sc_biguint<129>)wide_c - (sc_biguint<129>)wide_mul;
                        if (sticky_align) sum_res -= 1;
                        final_sign = c_sign;
                    }

                    if (sum_res == 0 && !sticky_align) {
                        final_sign = (reg_init_rm.read() == 2) ? 1 : 0; 
                    }
                }

                reg_sum_res.write(sum_res);
                reg_final_sign.write(final_sign);
                reg_final_exp.write(common_exp);
                reg_add_sticky.write(sticky_align);

                state.write(MAF_STATE_NORM_MSB);
                break;
            }

            case MAF_STATE_NORM_MSB: {
                sc_biguint<129> sum_res = reg_sum_res.read();
                sc_int<10> exp = reg_final_exp.read();
                bool sticky_align = reg_add_sticky.read();

                int msb = 0;
                if (sum_res != 0) {
                    sc_biguint<129> tmp = sum_res;
                    if (tmp.range(128, 64) != 0) { msb += 64; tmp = tmp >> 64; }
                    if (tmp.range(63, 32)  != 0) { msb += 32; tmp = tmp >> 32; }
                    if (tmp.range(31, 16)  != 0) { msb += 16; tmp = tmp >> 16; }
                    if (tmp.range(15, 8)   != 0) { msb += 8;  tmp = tmp >> 8;  }
                    if (tmp.range(7, 4)    != 0) { msb += 4;  tmp = tmp >> 4;  }
                    if (tmp.range(3, 2)    != 0) { msb += 2;  tmp = tmp >> 2;  }
                    if (tmp.range(1, 1)    != 0) { msb += 1; }
                }

                sc_int<10> final_exp = exp;
                int shift_val = 0;
                
                if (sum_res == 0) {
                    final_exp = 0; 
                } else {
                    shift_val = msb - 92; // Baseline binary point moved to 92
                    
                    if (exp + shift_val <= 0) {
                        shift_val = 1 - exp; 
                        final_exp = 0; 
                    } else {
                        final_exp += shift_val;
                    }
                }

                // Write to new pipeline registers
                reg_shift_val.write(shift_val);
                reg_final_exp_out.write(final_exp); 
                reg_add_sticky_pass.write(sticky_align);
                
                state.write(MAF_STATE_NORM_SHIFT);
                break;
            }

            case MAF_STATE_NORM_SHIFT: {
                // sum_res is still valid from MAF_STATE_ADD
                sc_biguint<129> sum_res = reg_sum_res.read(); 
                int shift_val = reg_shift_val.read();
                bool sticky_align = reg_add_sticky_pass.read();

                sc_biguint<129> norm_mantissa = 0;
                
                if (sum_res != 0) {
                    if (shift_val >= 129) {
                        norm_mantissa = 0;
                        sticky_align = sticky_align | (sum_res != 0);
                    } else if (shift_val > 0) {
                        norm_mantissa = sum_res >> shift_val;
                        sc_biguint<129> mask = ((sc_biguint<129>)1 << shift_val) - 1;
                        sticky_align = sticky_align | ((sum_res & mask) != 0);
                    } else if (shift_val < 0) {
                        norm_mantissa = sum_res << (-shift_val);
                    } else {
                        norm_mantissa = sum_res;
                    }
                }

                // G, R, and S extracted relative to new radix point position
                bool G = norm_mantissa[68];
                bool R = norm_mantissa[67];
                bool S = (norm_mantissa.range(66, 0) != 0) | sticky_align;

                reg_norm_mant.write(norm_mantissa.range(91, 69).to_uint()); 
                reg_round_bits.write((G << 2) | (R << 1) | (S << 0)); 
                
                state.write(MAF_STATE_ROUND);
                break;
            }

            case MAF_STATE_ROUND: {
                sc_uint<23> norm_frac = reg_norm_mant.read();
                sc_uint<3>  grs       = reg_round_bits.read();
                sc_int<10>  final_exp = reg_final_exp_out.read();
                bool        l_sign    = reg_final_sign.read();
                sc_uint<3>  rm        = reg_init_rm.read();

                sc_uint<27> M;
                bool is_zero = (norm_frac == 0 && final_exp <= 0); // Check if the result is a true zero
                
                M.range(26, 26) = is_zero ? 0 : 1; 
                M.range(25, 3)  = norm_frac;
                M.range(2, 0)   = grs;

                sc_uint<24> mant = M.range(26, 3);
                bool lsb = M.range(3, 3);
                bool g   = M.range(2, 2);
                bool r   = M.range(1, 1);
                bool s   = M.range(0, 0);
                
                bool round_up = false;
                bool any_fraction = (g || r || s);

                switch(rm) {
                    case 0: round_up = g && (lsb || r || s); break;    // Round to Nearest, Ties to Even
                    case 1: round_up = false; break;                   // Round toward Zero (Truncate)
                    case 2: round_up = l_sign && any_fraction; break;  // Round toward Minus Infinity
                    case 3: round_up = !l_sign && any_fraction; break; // Round toward Plus Infinity
                    case 4: round_up = g; break;                       // Round to Nearest, Ties to Away
                    default: round_up = false; break;
                }

                sc_uint<24> final_mant;

                if (round_up) {
                    sc_uint<25> mant_plus_1 = mant + 1;
                    if (mant_plus_1.range(24, 24)) { 
                        final_mant = mant_plus_1.range(24, 1);
                        final_exp++;
                    } else {
                        final_mant = mant_plus_1.range(23, 0);
                    }
                } else {
                    final_mant = mant;
                }

                sc_uint<32> result;
                result.range(31, 31) = l_sign;

                if (final_exp >= 255) {
                    bool to_inf;
                    switch(rm) {
                        case 0: case 4: to_inf = true; break;
                        case 1: to_inf = false; break;
                        case 2: to_inf = l_sign; break;
                        case 3: to_inf = !l_sign; break;
                        default: to_inf = true; break;
                    }

                    if (to_inf) {
                        result.range(30, 23) = 0xFF;        
                        result.range(22, 0)  = 0;           
                    } else {
                        result.range(30, 23) = 0xFE;        
                        result.range(22, 0)  = 0x7FFFFF;    
                    }
                } 
                // Underflow / Denormal Handling
                else if (final_exp <= 0) {
                    result.range(30, 23) = 0;
                    result.range(22, 0)  = final_mant.range(22, 0);
                } 
                // Normal Number Packing
                else {
                    result.range(30, 23) = (sc_uint<8>)final_exp;
                    result.range(22, 0)  = final_mant.range(22, 0);
                }

                result_out.write(result);
                done_out.write(1);
                state.write(MAF_STATE_WAIT); // Pointing to your FMA's wait state
                break;
            }

            default: {
                state.write(MAF_STATE_WAIT);
                break;
            }
        }
    }
}