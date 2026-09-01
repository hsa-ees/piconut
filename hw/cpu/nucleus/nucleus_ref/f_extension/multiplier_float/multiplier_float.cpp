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

#include "multiplier_float.h"

typedef enum {
    MF_STATE_WAIT,
    MF_STATE_COMP,
    MF_STATE_CALC,
    MF_STATE_NORM_LZC,
    MF_STATE_NORM_SHIFT,
    MF_STATE_ROUND,
    MF_STATE_PACK
} e_multiplier_float_states;

void m_multiplier_float::pn_trace(sc_trace_file* tf, int level) {
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, stb_in);
    PN_TRACE(tf, a_in);
    PN_TRACE(tf, b_in);
    PN_TRACE(tf, ready_out);
    PN_TRACE(tf, done_out);
    PN_TRACE(tf, result_out);
    PN_TRACE(tf, rounding_mode_in);
    PN_TRACE(tf, state);
    PN_TRACE(tf, reg_mul_res);
    PN_TRACE(tf, reg_result_mant);
    PN_TRACE(tf, reg_result_exp);
}

void m_multiplier_float::proc_clk_multiplier_float() {
    state.write(MF_STATE_WAIT);
    ready_out.write(1);
    done_out.write(0);
    result_out.write(0);

    reg_a_exp.write(0);
    reg_b_exp.write(0);
    reg_a_mant.write(0);
    reg_b_mant.write(0);
    reg_mul_res.write(0);
    digit.write(0);
    reg_g.write(0);
    reg_r.write(0);
    reg_s.write(0);
    reg_result_mant.write(0);
    reg_result_exp.write(0);

    reg_init_a_sign.write(0);
    reg_init_b_sign.write(0);
    reg_init_a_exp.write(0);
    reg_init_b_exp.write(0);
    reg_init_a_mant.write(0);
    reg_init_b_mant.write(0);
    reg_init_rm.write(0);

    while(true) {
        wait();

        // Default outputs
        ready_out.write(state.read() == MF_STATE_WAIT);
        done_out.write(0);

        switch(state.read()) {
            
            case MF_STATE_WAIT: {
                if (stb_in.read()) {
                    sc_uint<32> a = a_in.read();
                    sc_uint<32> b = b_in.read();
                    
                    reg_init_a_sign.write(a.range(31, 31));
                    reg_init_b_sign.write(b.range(31, 31));
                    reg_init_a_exp.write(a.range(30, 23));
                    reg_init_b_exp.write(b.range(30, 23));
                    reg_init_a_mant.write(a.range(22, 0));
                    reg_init_b_mant.write(b.range(22, 0));
                    reg_init_rm.write(rounding_mode_in.read());

                    state.write(MF_STATE_COMP);
                }
                break;
            }

            case MF_STATE_COMP: {
                sc_uint<1> sign_a = reg_init_a_sign.read();
                sc_uint<1> sign_b = reg_init_b_sign.read();
                sc_uint<8> exp_a  = reg_init_a_exp.read();
                sc_uint<8> exp_b  = reg_init_b_exp.read();
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
                    
                    res_nan[22] = 1; // Force Quiet NaN
                    result_out.write(res_nan);
                    done_out.write(1);
                    state.write(MF_STATE_WAIT);
                    
                } else if (a_is_inf || b_is_inf) {
                    sc_uint<32> res;
                    if (a_is_zero || b_is_zero) {
                        res = 0xffc00000; // -qnan (Inf * 0)
                    } else {
                        res = 0x7f800000; // inf
                        res.range(31, 31) = sign_a ^ sign_b;
                    }
                    result_out.write(res);
                    done_out.write(1);
                    state.write(MF_STATE_WAIT);
                    
                } else if (a_is_zero || b_is_zero) {
                    sc_uint<32> res = 0;
                    res.range(31, 31) = sign_a ^ sign_b;
                    result_out.write(res);
                    done_out.write(1);
                    state.write(MF_STATE_WAIT);
                    
                } else {
                    reg_a_exp.write(a_is_sub ? sc_uint<8>(1) : exp_a);
                    reg_b_exp.write(b_is_sub ? sc_uint<8>(1) : exp_b);
                    reg_a_mant.write((sc_uint<1>(!a_is_sub), mant_a));
                    reg_b_mant.write((sc_uint<1>(!b_is_sub), mant_b));
        
                    reg_mul_res.write(0);
                    digit.write(0);
                    state.write(MF_STATE_CALC);
                }
                break;
            }

            case MF_STATE_CALC: {
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
                    state.write(MF_STATE_NORM_LZC);
                } else {
                    digit.write(i + 2); 
                }
                
                reg_mul_res.write(current_res);
                break;
            }

            case MF_STATE_NORM_LZC: {
                sc_uint<48> mul_res = reg_mul_res.read();
                sc_int<10> exp_sum = (sc_int<10>)reg_a_exp.read() + (sc_int<10>)reg_b_exp.read() - 127;

                if (mul_res == 0) {
                    reg_result_mant.write(0);
                    reg_result_exp.write(0);
                    reg_g.write(0);
                    reg_r.write(0);
                    reg_s.write(0);
                    state.write(MF_STATE_ROUND);
                } else {
                    int msb = 0;
                    
                    // 48-bit Binary Search Tree
                    if (mul_res.range(47, 32) != 0) {
                        if (mul_res.range(47, 40) != 0) {
                            if (mul_res.range(47, 44) != 0) {
                                if (mul_res.range(47, 46) != 0) msb = mul_res.range(47, 47) ? 47 : 46;
                                else msb = mul_res.range(45, 45) ? 45 : 44;
                            } else {
                                if (mul_res.range(43, 42) != 0) msb = mul_res.range(43, 43) ? 43 : 42;
                                else msb = mul_res.range(41, 41) ? 41 : 40;
                            }
                        } else {
                            if (mul_res.range(39, 36) != 0) {
                                if (mul_res.range(39, 38) != 0) msb = mul_res.range(39, 39) ? 39 : 38;
                                else msb = mul_res.range(37, 37) ? 37 : 36;
                            } else {
                                if (mul_res.range(35, 34) != 0) msb = mul_res.range(35, 35) ? 35 : 34;
                                else msb = mul_res.range(33, 33) ? 33 : 32;
                            }
                        }
                    } else if (mul_res.range(31, 16) != 0) {
                        if (mul_res.range(31, 24) != 0) {
                            if (mul_res.range(31, 28) != 0) {
                                if (mul_res.range(31, 30) != 0) msb = mul_res.range(31, 31) ? 31 : 30;
                                else msb = mul_res.range(29, 29) ? 29 : 28;
                            } else {
                                if (mul_res.range(27, 26) != 0) msb = mul_res.range(27, 27) ? 27 : 26;
                                else msb = mul_res.range(25, 25) ? 25 : 24;
                            }
                        } else {
                            if (mul_res.range(23, 20) != 0) {
                                if (mul_res.range(23, 22) != 0) msb = mul_res.range(23, 23) ? 23 : 22;
                                else msb = mul_res.range(21, 21) ? 21 : 20;
                            } else {
                                if (mul_res.range(19, 18) != 0) msb = mul_res.range(19, 19) ? 19 : 18;
                                else msb = mul_res.range(17, 17) ? 17 : 16;
                            }
                        }
                    } else {
                        if (mul_res.range(15, 8) != 0) {
                            if (mul_res.range(15, 12) != 0) {
                                if (mul_res.range(15, 14) != 0) msb = mul_res.range(15, 15) ? 15 : 14;
                                else msb = mul_res.range(13, 13) ? 13 : 12;
                            } else {
                                if (mul_res.range(11, 10) != 0) msb = mul_res.range(11, 11) ? 11 : 10;
                                else msb = mul_res.range(9, 9) ? 9 : 8;
                            }
                        } else {
                            if (mul_res.range(7, 4) != 0) {
                                if (mul_res.range(7, 6) != 0) msb = mul_res.range(7, 7) ? 7 : 6;
                                else msb = mul_res.range(5, 5) ? 5 : 4;
                            } else {
                                if (mul_res.range(3, 2) != 0) msb = mul_res.range(3, 3) ? 3 : 2;
                                else msb = mul_res.range(1, 1) ? 1 : 0;
                            }
                        }
                    }
                    
                    // Pipeline the results
                    reg_lzc.write(47 - msb);
                    reg_exp_sum.write(exp_sum);
                    
                    state.write(MF_STATE_NORM_SHIFT);
                }

                break;
            }

            case MF_STATE_NORM_SHIFT: {
                sc_uint<48> mul_res = reg_mul_res.read();
                sc_uint<6>  lzc     = reg_lzc.read();
                sc_int<10>  exp_sum = reg_exp_sum.read();

                sc_int<10> ideal_exp = exp_sum + 1 - (int)lzc;
                sc_uint<48> shifted_res = 0;
                sc_int<10> final_exp = 0;
                bool lost_bits = false;

                if (ideal_exp >= 1) {
                    final_exp = ideal_exp;
                    if (lzc == 0) {
                        shifted_res = mul_res >> 1;
                        lost_bits = (mul_res.range(0, 0) != 0);
                    } else {
                        shifted_res = mul_res << ((int)lzc - 1);
                        lost_bits = false;
                    }
                } else {
                    final_exp = 0; 
                    int right_shift = 1 - exp_sum; 
                    
                    if (right_shift > 0) {
                        if (right_shift > 47) {
                            shifted_res = 0;
                            lost_bits = (mul_res != 0);
                        } else {
                            shifted_res = mul_res >> right_shift;
                            sc_uint<48> mask = (sc_uint<48>(1) << right_shift) - 1;
                            lost_bits = ((mul_res & mask) != 0);
                        }
                    } else if (right_shift < 0) {
                        int left_shift = -right_shift;
                        shifted_res = mul_res << left_shift;
                        lost_bits = false;
                    } else {
                        shifted_res = mul_res;
                        lost_bits = false;
                    }
                }

                reg_result_mant.write(shifted_res.range(46, 23));
                reg_g.write(shifted_res.range(22, 22));
                reg_r.write(shifted_res.range(21, 21));
                reg_s.write((shifted_res.range(20, 0) != 0) || lost_bits);
                reg_result_exp.write(final_exp);
                
                state.write(MF_STATE_ROUND);
                break;
            }

            case MF_STATE_ROUND: {
                sc_uint<1> sign_a = reg_init_a_sign.read();
                sc_uint<1> sign_b = reg_init_b_sign.read();
                sc_uint<24> mant = reg_result_mant.read();
                sc_int<10> exp = reg_result_exp.read();
                
                sc_uint<1> g = reg_g.read();
                sc_uint<1> r = reg_r.read();
                sc_uint<1> s = reg_s.read();
                sc_uint<1> lsb = mant.range(0, 0);
                sc_uint<3> rm = reg_init_rm.read();

                bool res_sign = sign_a ^ sign_b;
                bool any_fraction = (g || r || s);
                bool round_up = false;

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

                reg_result_mant.write(mant);
                reg_result_exp.write(exp);
                
                state.write(MF_STATE_PACK); // Step forward
                break;
            }

            case MF_STATE_PACK: {
                sc_uint<1> sign_a = reg_init_a_sign.read();
                sc_uint<1> sign_b = reg_init_b_sign.read();
                sc_uint<24> mant = reg_result_mant.read();  // Read the rounded mantissa
                sc_int<10> exp = reg_result_exp.read();     // Read the updated exponent
                sc_uint<3> rm = reg_init_rm.read();

                bool res_sign = sign_a ^ sign_b;
                sc_uint<32> res = 0;
                res.range(31, 31) = res_sign;

                if (exp >= 255) {
                    bool overflow_to_inf = false;
                    switch(rm) {
                        case 0: overflow_to_inf = true; break;
                        case 1: overflow_to_inf = false; break;
                        case 2: overflow_to_inf = (res_sign == 1); break;
                        case 3: overflow_to_inf = (res_sign == 0); break;
                        case 4: overflow_to_inf = true; break;
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
                    // Standard packing
                    res.range(30, 23) = (sc_uint<8>)exp;
                    res.range(22, 0)  = mant.range(22, 0);
                }

                result_out.write(res);
                done_out.write(1);
                state.write(MF_STATE_WAIT);
                break;
            }

            default: {
                state.write(MF_STATE_WAIT);
                break;
            }
        }
    }
}