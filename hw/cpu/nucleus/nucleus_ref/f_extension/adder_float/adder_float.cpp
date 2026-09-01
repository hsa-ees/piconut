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

#include "adder_float.h"

typedef enum {
    AF_STATE_WAIT,
    AF_STATE_COMP,
    AF_STATE_ALIGN,
    AF_STATE_ADD_AND_NORM,
    AF_STATE_ROUND_AND_PACK,
} e_adder_float_states;

void m_adder_float::pn_trace(sc_trace_file* tf, int level) {
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, stb_in);
    PN_TRACE(tf, a_in);
    PN_TRACE(tf, b_in);
    PN_TRACE(tf, ready_out);
    PN_TRACE(tf, done_out);
    PN_TRACE(tf, result_out);
    PN_TRACE(tf, rounding_mode_in);
    PN_TRACE(tf, reg_a_exp);
    PN_TRACE(tf, reg_a_mant);
    PN_TRACE(tf, reg_b_mant);
    PN_TRACE(tf, shift_amount);
    PN_TRACE(tf, res_sign);
    PN_TRACE(tf, aligned_a_with_grs);
    PN_TRACE(tf, aligned_b_with_grs);
    PN_TRACE(tf, state);
    PN_TRACE(tf, sum);
    PN_TRACE(tf, normalized_mant);
    PN_TRACE(tf, reg_init_a_sign);
    PN_TRACE(tf, reg_init_b_sign);
    PN_TRACE(tf, reg_init_a_exp);
    PN_TRACE(tf, reg_init_b_exp);
    PN_TRACE(tf, reg_init_a_mant);
    PN_TRACE(tf, reg_init_b_mant);
}

void m_adder_float::proc_clk_adder_float() {
    state.write(AF_STATE_WAIT);
    ready_out.write(1);
    done_out.write(0);
    result_out.write(0);

    reg_a_exp.write(0);
    reg_a_mant.write(0);
    reg_b_mant.write(0);
    shift_amount.write(0);
    res_sign.write(0);
    aligned_a_with_grs.write(0);
    aligned_b_with_grs.write(0);
    sum.write(0);
    normalized_mant.write(0);

    reg_init_a_sign.write(0);
    reg_init_b_sign.write(0);
    reg_init_a_exp.write(0);
    reg_init_b_exp.write(0);
    reg_init_a_mant.write(0);
    reg_init_b_mant.write(0);
    reg_init_rm.write(0);

    while(true) {
        wait();

        ready_out.write(state.read() == AF_STATE_WAIT);
        done_out.write(0);

        switch(state.read()) {
            case AF_STATE_WAIT: {
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

                    state.write(AF_STATE_COMP);
                }
                break;
            }

            case AF_STATE_COMP: {
                sc_uint<1> sign_a = reg_init_a_sign.read();
                sc_uint<1> sign_b = reg_init_b_sign.read();
                sc_uint<8> exp_a = reg_init_a_exp.read();
                sc_uint<8> exp_b = reg_init_b_exp.read();
                sc_uint<23> mant_a = reg_init_a_mant.read();
                sc_uint<23> mant_b = reg_init_b_mant.read();
                sc_uint<3> current_rm = reg_init_rm.read();
                
                bool a_is_inf = (exp_a == 0xFF) && (mant_a == 0);
                bool b_is_inf = (exp_b == 0xFF) && (mant_b == 0);
                bool a_is_nan = (exp_a == 0xFF) && (mant_a != 0);
                bool b_is_nan = (exp_b == 0xFF) && (mant_b != 0);
                bool a_is_zero = (exp_a == 0) && (mant_a == 0);
                bool b_is_zero = (exp_b == 0) && (mant_b == 0);
                bool a_is_sub = (exp_a == 0);
                bool b_is_sub = (exp_b == 0);

                if (a_is_nan || b_is_nan) {
                    sc_uint<32> val_a = (sign_a, exp_a, mant_a);
                    sc_uint<32> val_b = (sign_b, exp_b, mant_b);
                    sc_uint<32> res_nan = a_is_nan ? val_a : val_b;
                    res_nan[22] = 1; 
                    result_out.write(res_nan);
                    done_out.write(1);
                    state.write(AF_STATE_WAIT);
                    
                } else if (a_is_inf && b_is_inf) {
                    if (sign_a != sign_b) {
                        result_out.write(0xFFC00000); // Quiet NaN
                    } else {
                        result_out.write((sign_a, exp_a, mant_a));
                    }
                    done_out.write(1);
                    state.write(AF_STATE_WAIT);
                    
                } else if (a_is_inf || b_is_inf) {
                    sc_uint<32> inf_res;
                    inf_res[31] = a_is_inf ? sign_a : sign_b;
                    inf_res.range(30, 23) = 0xFF;
                    inf_res.range(22, 0) = 0;
                    result_out.write(inf_res);
                    done_out.write(1);
                    state.write(AF_STATE_WAIT);
                    
                } else if (a_is_zero && b_is_zero) {
                    sc_uint<32> zero_res = 0;
                    if (sign_a == sign_b) {
                        zero_res[31] = sign_a; 
                    } else {
                        zero_res[31] = (current_rm == 2) ? 1 : 0; 
                    }
                    result_out.write(zero_res);
                    done_out.write(1);
                    state.write(AF_STATE_WAIT);
                    
                } else if (a_is_zero) {
                    result_out.write((sign_b, exp_b, mant_b));
                    done_out.write(1);
                    state.write(AF_STATE_WAIT);
                    
                } else if (b_is_zero) {
                    result_out.write((sign_a, exp_a, mant_a));
                    done_out.write(1);
                    state.write(AF_STATE_WAIT);
                    
                } else {
                    sc_uint<8> eff_a = a_is_sub ? sc_uint<8>(1) : exp_a;
                    sc_uint<8> eff_b = b_is_sub ? sc_uint<8>(1) : exp_b;
        
                    bool a_larger;
                    if (eff_a != eff_b) {
                        a_larger = (eff_a > eff_b);
                    } else {
                        sc_uint<24> full_a = (sc_uint<1>(!a_is_sub), mant_a);
                        sc_uint<24> full_b = (sc_uint<1>(!b_is_sub), mant_b);
                        a_larger = (full_a >= full_b);
                    }
        
                    if (a_larger) {
                        reg_a_exp.write(eff_a);
                        reg_a_mant.write((sc_uint<1>(!a_is_sub), mant_a));
                        reg_b_mant.write((sc_uint<1>(!b_is_sub), mant_b));
                        shift_amount.write(eff_a - eff_b); 
                        res_sign.write(sign_a);
                    } else {
                        reg_a_exp.write(eff_b);
                        reg_a_mant.write((sc_uint<1>(!b_is_sub), mant_b));
                        reg_b_mant.write((sc_uint<1>(!a_is_sub), mant_a));
                        shift_amount.write(eff_b - eff_a);
                        res_sign.write(sign_b);
                    }
                    state.write(AF_STATE_ALIGN);
                }
                break;
            }

            case AF_STATE_ALIGN: {
                sc_uint<24> val_b_mant = reg_b_mant.read();
                sc_uint<8>  val_shift  = shift_amount.read();

                sc_uint<27> ext = (val_b_mant, sc_uint<3>(0));
                sc_uint<27> shifted;

                if (val_shift >= 27) {
                    shifted = 0;
                    if (val_b_mant != 0) shifted.range(0, 0) = 1;
                } else {
                    sc_uint<5> safe_shift = (sc_uint<5>)val_shift;
                    shifted = ext >> safe_shift;

                    sc_uint<27> mask = (sc_uint<27>(1) << safe_shift) - 1;
                    if ((ext & mask) != 0) shifted.range(0, 0) = 1;
                }

                aligned_b_with_grs.write(shifted);
                aligned_a_with_grs.write((reg_a_mant.read(), sc_uint<3>(0)));

                state.write(AF_STATE_ADD_AND_NORM); // Strict linear step
                break;
            }

            case AF_STATE_ADD_AND_NORM: {
                sc_uint<27> a_mant = aligned_a_with_grs.read();
                sc_uint<27> b_mant = aligned_b_with_grs.read();
                sc_uint<1>  sign_a = reg_init_a_sign.read();
                sc_uint<1>  sign_b = reg_init_b_sign.read();
                sc_int<10>  l_exp  = (sc_int<10>)reg_a_exp.read();

                sc_uint<28> l_sum;
                if (sign_a == sign_b) {
                    l_sum = a_mant + b_mant;
                } else {
                    l_sum = a_mant - b_mant;
                }

                if (l_sum == 0) {
                    // Fast zero out, handled safely
                    sc_uint<32> zero_res = 0;
                    zero_res.range(31, 31) = (reg_init_rm.read() == 2) ? 1 : 0;
                    result_out.write(zero_res);
                    done_out.write(1);
                    state.write(AF_STATE_WAIT);
                } else {
                    if (l_sum.range(27, 27)) {
                        sc_uint<27> temp = l_sum.range(27, 1);
                        temp.range(0, 0) = temp.range(0, 0) | l_sum.range(0, 0); 
                        normalized_mant.write(temp); 
                        l_exp = l_exp + 1;
                    } else {
                        // Static Binary Tree LZC
                        sc_uint<27> val = l_sum.range(26, 0);
                        int msb = 0;
                        if (val.range(26, 16) != 0) {
                            if (val.range(26, 24) != 0) {
                                if (val.range(26, 25) != 0) msb = val.range(26, 26) ? 26 : 25;
                                else msb = 24;
                            } else { 
                                if (val.range(23, 20) != 0) {
                                    if (val.range(23, 22) != 0) msb = val.range(23, 23) ? 23 : 22;
                                    else msb = val.range(21, 21) ? 21 : 20;
                                } else { 
                                    if (val.range(19, 18) != 0) msb = val.range(19, 19) ? 19 : 18;
                                    else msb = val.range(17, 17) ? 17 : 16;
                                }
                            }
                        } else if (val.range(15, 8) != 0) {
                            if (val.range(15, 12) != 0) {
                                if (val.range(15, 14) != 0) msb = val.range(15, 15) ? 15 : 14;
                                else msb = val.range(13, 13) ? 13 : 12;
                            } else {
                                if (val.range(11, 10) != 0) msb = val.range(11, 11) ? 11 : 10;
                                else msb = val.range(9, 9) ? 9 : 8;
                            }
                        } else { 
                            if (val.range(7, 4) != 0) {
                                if (val.range(7, 6) != 0) msb = val.range(7, 7) ? 7 : 6;
                                else msb = val.range(5, 5) ? 5 : 4;
                            } else {
                                if (val.range(3, 2) != 0) msb = val.range(3, 3) ? 3 : 2;
                                else msb = val.range(1, 1) ? 1 : 0;
                            }
                        }

                        int lzc = 26 - msb;
                        sc_int<10> s_lzc = (sc_int<10>)lzc;
                        sc_uint<5> shift;

                        if ((l_exp - s_lzc) < 1) {
                            shift = (l_exp > 0) ? (sc_uint<5>)(l_exp - 1) : (sc_uint<5>)0;
                            l_exp = 0;
                        } else {
                            shift = lzc;
                            l_exp = l_exp - s_lzc;
                        }

                        sc_uint<28> shifted = l_sum << shift;
                        normalized_mant.write(shifted.range(26, 0));
                    }
                    
                    reg_a_exp.write((l_exp < 0) ? (sc_uint<8>)0 : (sc_uint<8>)l_exp);
                    state.write(AF_STATE_ROUND_AND_PACK); // Strict linear step
                }
                break;
            }

            case AF_STATE_ROUND_AND_PACK: {
                sc_uint<27> M = normalized_mant.read();
                sc_uint<24> mant = M.range(26, 3);
                sc_int<10>  final_exp = (sc_int<10>)reg_a_exp.read();
                
                bool lsb = M.range(3, 3);
                bool g   = M.range(2, 2);
                bool r   = M.range(1, 1);
                bool s   = M.range(0, 0);
                
                bool round_up = false;
                sc_uint<3> rm = reg_init_rm.read();
                bool l_sign = res_sign.read();
                bool any_fraction = (g || r || s);

                switch(rm) {
                    case 0: round_up = g && (lsb || r || s); break;
                    case 1: round_up = false; break;
                    case 2: round_up = l_sign && any_fraction; break;
                    case 3: round_up = !l_sign && any_fraction; break;
                    case 4: round_up = g; break;
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

                if (reg_a_exp.read() == 0 && final_mant.range(23, 23) == 1) {
                    final_exp = 1; 
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
                } else if (final_exp <= 0) {
                    result.range(30, 23) = 0;
                    result.range(22, 0)  = final_mant.range(22, 0);
                } else {
                    result.range(30, 23) = (sc_uint<8>)final_exp;
                    result.range(22, 0)  = final_mant.range(22, 0);
                }

                result_out.write(result);
                done_out.write(1);
                state.write(AF_STATE_WAIT);
                break;
            }

            default:
                state.write(AF_STATE_WAIT);
                break;
        }
    }
}