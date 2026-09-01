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

#include "converter_float.h"

typedef enum {
    CF_STATE_WAIT,
    CF_STATE_FP_TO_INT_ALIGN,
    CF_STATE_INT_TO_FP_NORM,
    CF_STATE_ROUND_AND_PACK,
    CF_STATE_PACK_AND_FINISH,
    CF_STATE_FSGNJ_EXEC
} e_converter_float_states;

void m_converter_float::pn_trace(sc_trace_file* tf, int level) {
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, stb_in);
    PN_TRACE(tf, a_in);
    PN_TRACE(tf, b_in);
    PN_TRACE(tf, ready_out);
    PN_TRACE(tf, done_out);
    PN_TRACE(tf, result_out);
    PN_TRACE(tf, cvt_mode_in);
    PN_TRACE(tf, rounding_mode_in);
    PN_TRACE(tf, state);
}

// Thread or Clocked Method handling the state transitions and register assignments
void m_converter_float::proc_clk_converter_float() {
    state.write(CF_STATE_WAIT);
    ready_out.write(1);
    done_out.write(0);
    result_out.write(0);
    
    reg_a.write(0);
    reg_b.write(0);
    reg_cvt_mode.write(0);
    reg_rm.write(0);
    reg_sign.write(0);
    reg_exp.write(0);
    reg_mant.write(0);
    reg_shift.write(0);
    reg_invalid.write(false);
    reg_mag.write(0);
    reg_g.write(0);
    reg_r.write(0);
    reg_s.write(0);

    while (true) {
        wait();

        // Default outputs
        ready_out.write(state.read() == CF_STATE_WAIT);
        done_out.write(0);

        switch (state.read()) {
            case CF_STATE_WAIT: {
                if (stb_in.read()) {
                    sc_uint<32> a = a_in.read();
                    sc_uint<32> b = b_in.read();
                    sc_uint<3>  rm = rounding_mode_in.read();
                    sc_uint<4>  mode = cvt_mode_in.read();

                    reg_a.write(a);
                    reg_b.write(b);
                    reg_rm.write(rm);
                    reg_cvt_mode.write(mode);

                    // Pre-unpack floating point fields to alleviate next states
                    sc_uint<1>  s_a = a.range(31, 31);
                    sc_uint<8>  e_a = a.range(30, 23);
                    sc_uint<23> m_a = a.range(22, 0);
                    
                    reg_sign.write(s_a);
                    reg_exp.write(e_a);
                    reg_mant.write(m_a);
                    reg_invalid.write(false);

                    if (mode == FSGNJ) {
                        state.write(CF_STATE_FSGNJ_EXEC);
                    } else if (mode == F32_TO_UI32 || mode == F32_TO_I32) {
                        state.write(CF_STATE_FP_TO_INT_ALIGN);
                    } else if (mode == UI32_TO_F32 || mode == I32_TO_F32) {
                        state.write(CF_STATE_INT_TO_FP_NORM);
                    } else {
                        state.write(CF_STATE_WAIT);
                    }
                }
                break;
            }

            case CF_STATE_FP_TO_INT_ALIGN: {
                sc_uint<8>  exp_a  = reg_exp.read();
                sc_uint<23> mant_a = reg_mant.read();
                sc_uint<1>  sign_a = reg_sign.read();
                sc_uint<4>  mode   = reg_cvt_mode.read();
                sc_int<10>  shift  = exp_a - 127;

                bool a_is_inf  = (exp_a == 0xFF) && (mant_a == 0);
                bool a_is_nan  = (exp_a == 0xFF) && (mant_a != 0);
                bool a_is_zero = (exp_a == 0)    && (mant_a == 0);
                bool a_is_sub  = (exp_a == 0)    && (mant_a != 0);

                sc_uint<32> local_mag = 0;
                sc_uint<1> local_g = 0, local_r = 0, local_s = 0;
                bool is_invalid = false;

                // Max shift bounds verification
                sc_int<10> max_shift = (mode == F32_TO_UI32) ? 31 : 30;

                if (a_is_nan || a_is_inf || shift > max_shift) {
                    if (mode == F32_TO_I32 && shift == 31 && mant_a == 0 && sign_a == 1 && !a_is_nan && !a_is_inf) {
                        local_mag = 0x80000000;
                    } else {
                        is_invalid = true;
                    }
                } else if (a_is_zero || a_is_sub) {
                    local_mag = 0;
                    local_s = a_is_sub ? 1 : 0;
                } else {
                    sc_uint<24> implicit_mant = (sc_uint<1>(1), mant_a);
                    
                    if (shift >= 23) {
                        // REUSE EXPLICIT BARREL SHIFT
                        local_mag = implicit_mant << (shift - 23);
                    } else if (shift >= 0) {
                        sc_uint<5> right_shift = 23 - shift;
                        // REUSE EXPLICIT BARREL SHIFT
                        local_mag = implicit_mant >> right_shift;

                        local_g = (implicit_mant >> (right_shift - 1)) & 1;
                        if (right_shift > 1) {
                            local_r = (implicit_mant >> (right_shift - 2)) & 1;
                        }
                        if (right_shift > 2) {
                            sc_uint<24> mask = (sc_uint<24>(1) << (right_shift - 2)) - 1;
                            local_s = ((implicit_mant & mask) != 0) ? 1 : 0;
                        }
                    } else {
                        local_mag = 0;
                        if (shift == -1) {
                            local_g = 1;
                            local_r = mant_a[22];
                            local_s = (mant_a.range(21, 0) != 0) ? 1 : 0;
                        } else if (shift == -2) {
                            local_g = 0;
                            local_r = 1;
                            local_s = (mant_a != 0) ? 1 : 0;
                        } else {
                            local_s = 1;
                        }
                    }
                }

                reg_mag.write(local_mag);
                reg_g.write(local_g);
                reg_r.write(local_r);
                reg_s.write(local_s);
                reg_invalid.write(is_invalid);
                
                state.write(CF_STATE_ROUND_AND_PACK);
                break;
            }

            case CF_STATE_INT_TO_FP_NORM: {
                sc_uint<32> a = reg_a.read();
                sc_uint<4> mode = reg_cvt_mode.read();
                
                sc_uint<1> sign_val = 0;
                sc_uint<32> abs_val = a;

                if (mode == I32_TO_F32) {
                    sign_val = a.range(31, 31);
                    abs_val = sign_val ? (sc_uint<32>)(0 - a) : a;
                }

                reg_sign.write(sign_val);

                if (abs_val == 0) {
                    reg_mag.write(0);
                    reg_exp.write(0);
                    reg_g.write(0);
                    reg_r.write(0);
                    reg_s.write(0);
                    state.write(CF_STATE_ROUND_AND_PACK);
                } else {
                    int msb = 0;
                    if (abs_val.range(31, 16) != 0) {
                        if (abs_val.range(31, 24) != 0) {
                            if (abs_val.range(31, 28) != 0) {
                                if (abs_val[31]) msb = 31; else if (abs_val[30]) msb = 30; else if (abs_val[29]) msb = 29; else msb = 28;
                            } else {
                                if (abs_val[27]) msb = 27; else if (abs_val[26]) msb = 26; else if (abs_val[25]) msb = 25; else msb = 24;
                            }
                        } else {
                            if (abs_val.range(23, 20) != 0) {
                                if (abs_val[23]) msb = 23; else if (abs_val[22]) msb = 22; else if (abs_val[21]) msb = 21; else msb = 20;
                            } else {
                                if (abs_val[19]) msb = 19; else if (abs_val[18]) msb = 18; else if (abs_val[17]) msb = 17; else msb = 16;
                            }
                        }
                    } else {
                        if (abs_val.range(15, 8) != 0) {
                            if (abs_val.range(15, 12) != 0) {
                                if (abs_val[15]) msb = 15; else if (abs_val[14]) msb = 14; else if (abs_val[13]) msb = 13; else msb = 12;
                            } else {
                                if (abs_val[11]) msb = 11; else if (abs_val[10]) msb = 10; else if (abs_val[9]) msb = 9; else msb = 8;
                            }
                        } else {
                            if (abs_val.range(7, 4) != 0) {
                                if (abs_val[7]) msb = 7; else if (abs_val[6]) msb = 6; else if (abs_val[5]) msb = 5; else msb = 4;
                            } else {
                                if (abs_val[3]) msb = 3; else if (abs_val[2]) msb = 2; else if (abs_val[1]) msb = 1; else msb = 0;
                            }
                        }
                    }

                    sc_uint<8> local_exp = 127 + msb;
                    sc_uint<32> local_mant = 0;
                    sc_uint<1> local_g = 0, local_r = 0, local_s = 0;

                    if (msb > 23) {
                        sc_uint<5> right_shift = msb - 23;
                        local_mant = (abs_val >> right_shift) & 0x7FFFFF;
                        
                        local_g = (abs_val >> (right_shift - 1)) & 1;
                        if (right_shift > 1) {
                            local_r = (abs_val >> (right_shift - 2)) & 1;
                        }
                        if (right_shift > 2) {
                            sc_uint<32> mask = (sc_uint<32>(1) << (right_shift - 2)) - 1;
                            local_s = ((abs_val & mask) != 0) ? 1 : 0;
                        }
                    } else {
                        local_mant = (abs_val << (23 - msb)) & 0x7FFFFF;
                    }

                    reg_mag.write(local_mant);
                    reg_exp.write(local_exp);
                    reg_g.write(local_g);
                    reg_r.write(local_r);
                    reg_s.write(local_s);
                    
                    state.write(CF_STATE_ROUND_AND_PACK);
                }
                break;
            }

            case CF_STATE_ROUND_AND_PACK: {
                sc_uint<4>  mode       = reg_cvt_mode.read();
                sc_uint<3>  rm         = reg_rm.read();
                sc_uint<1>  sign_val   = reg_sign.read();
                sc_uint<32> out_mag    = reg_mag.read();
                sc_uint<8>  out_exp    = reg_exp.read();
                bool        is_invalid = reg_invalid.read();
                
                sc_uint<1> g = reg_g.read();
                sc_uint<1> r = reg_r.read();
                sc_uint<1> s = reg_s.read();

                bool round_up = false;
                bool any_fraction = (g || r || s);
                bool lsb = out_mag.range(0, 0); // Safe from bitref error

                if (!is_invalid) {
                    switch(rm) {
                        case 0: round_up = g && (lsb || r || s); break; 
                        case 1: round_up = false; break;                
                        case 2: round_up = sign_val ? any_fraction : false; break; 
                        case 3: round_up = sign_val ? false : any_fraction; break; 
                        case 4: round_up = g; break;                    
                        default: round_up = false; break;
                    }
                }

                // Isolate the First Addition (Rounding)
                if (round_up && !is_invalid) {
                    out_mag += 1;
                    
                    // Handle float-specific mantissa overflow from rounding
                    if ((mode == UI32_TO_F32 || mode == I32_TO_F32) && out_mag > 0x7FFFFF) {
                        out_mag = 0;
                        out_exp += 1; 
                    }
                }

                reg_mag.write(out_mag); // Save the incremented magnitude
                reg_exp.write(out_exp); // Save the incremented exponent
                
                state.write(CF_STATE_PACK_AND_FINISH); // Step forward
                break;
            }

            case CF_STATE_PACK_AND_FINISH: {
                sc_uint<4>  mode       = reg_cvt_mode.read();
                sc_uint<1>  sign_val   = reg_sign.read();
                sc_uint<32> out_mag    = reg_mag.read(); // Read the rounded magnitude
                sc_uint<8>  out_exp    = reg_exp.read(); // Read the updated exponent
                bool        is_invalid = reg_invalid.read();
                
                sc_uint<32> final_res = 0;

                if (mode == F32_TO_UI32) {
                    if (is_invalid || (sign_val && out_mag != 0)) {
                        final_res = 0xFFFFFFFF; // Bounded to max unsigned
                    } else {
                        final_res = out_mag;
                    }
                } 
                else if (mode == F32_TO_I32) {
                    if (is_invalid) {
                        final_res = 0x80000000;
                    } else if (sign_val == 0 && out_mag > 0x7FFFFFFF) {
                        final_res = 0x80000000; // Positive overflow
                    } else if (sign_val == 1 && out_mag > 0x80000000) {
                        final_res = 0x80000000; // Negative overflow
                    } else {
                        // Isolate the Second Addition (Two's Complement)
                        final_res = sign_val ? (sc_uint<32>)(0 - out_mag) : out_mag;
                    }
                } 
                else if (mode == UI32_TO_F32 || mode == I32_TO_F32) {
                    if (out_mag != 0 || out_exp != 0) {
                        final_res = (sign_val << 31) | (out_exp << 23) | (out_mag & 0x7FFFFF);
                    } else {
                        final_res = 0;
                    }
                }

                result_out.write(final_res);
                done_out.write(1);
                state.write(CF_STATE_WAIT);
                break;
            }

            case CF_STATE_FSGNJ_EXEC: {
                sc_uint<32> a    = reg_a.read();
                sc_uint<32> b    = reg_b.read();
                sc_uint<3>  rm   = reg_rm.read();
                sc_uint<32> res  = 0;

                if (rm == 0b000) {
                    res = (b.range(31, 31), a.range(30, 0));
                } else if (rm == 0b001) {
                    res = (sc_uint<1>(~b.range(31, 31)), a.range(30, 0)); 
                } else if (rm == 0b010) {
                    res = (sc_uint<1>(a.range(31, 31) ^ b.range(31, 31)), a.range(30, 0));
                }

                result_out.write(res);
                done_out.write(1);
                state.write(CF_STATE_WAIT);
                break;
            }

            default:
                state.write(CF_STATE_WAIT);
                break;
        }
    }
}