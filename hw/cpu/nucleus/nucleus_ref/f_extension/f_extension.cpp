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

#include "f_extension.h"

typedef enum {
    F_STATE_RESET,
    F_STATE_WAIT,
    F_STATE_DECODE,
    F_STATE_CLASS,
    F_STATE_COMP_REQUEST,
    F_STATE_COMP_BUSY,
    F_STATE_SUB_REQUEST,
    F_STATE_ADD_REQUEST,
    F_STATE_ADD_BUSY,
    F_STATE_MULT_REQUEST,
    F_STATE_MULT_BUSY,
    F_STATE_DIV_REQUEST,
    F_STATE_DIV_BUSY,
    F_STATE_MV_OUT,
    F_STATE_CVT_I_TO_F_REQUEST,
    F_STATE_CVT_I_TO_F_BUSY,
    F_STATE_CVT_F_TO_I_REQUEST,
    F_STATE_CVT_F_TO_I_BUSY,
    F_STATE_SGNJ_REQUEST,
    F_STATE_SGNJ_BUSY,
    F_STATE_SQRT_REQUEST,
    F_STATE_SQRT_BUSY,
    F_STATE_FMADD_REQUEST,
    F_STATE_FMSUB_REQUEST,
    F_STATE_FNMSUB_REQUEST,
    F_STATE_FNMADD_REQUEST,
    F_STATE_FMADD_BUSY
} e_fext_states;

typedef enum {
    FUNCT5F_ADD          = 0b00000,
    FUNCT5F_SUB          = 0b00001,
    FUNCT5F_MULT         = 0b00010,
    FUNCT5F_DIV          = 0b00011,
    FUNCT5F_SGNJ         = 0b00100,
    FUNCT5F_SQRT         = 0b01011,
    FUNCT5F_COMP         = 0b10100,
    FUNCT5F_CVT_F_TO_I   = 0b11000,
    FUNCT5F_CVT_I_TO_F   = 0b11010,
    FUNCT5F_CLASS_MV_X_W = 0b11100,
} e_fext_func5;

typedef enum {
    OPF_FMADD  = 0b1000011,
    OPF_FMSUB  = 0b1000111,
    OPF_FNMSUB = 0b1001011,
    OPF_FNMADD = 0b1001111
} e_fext_op;

#include "regfile_float/regfile_float.h"
#include "classifier_float/classifier_float.h"
#include "adder_float/adder_float.h"
#include "multiplier_float/multiplier_float.h"
#include "divisor_float/divisor_float.h"
#include "compare_float/compare_float.h"
#include "converter_float/converter_float.h"
#include "sqrt_float/sqrt_float.h"
#include "mul_add_float/mul_add_float.h"

void m_f_extension::init_submodules() {
    regfile = sc_new<m_regfile_float>("regfile");
    classifier = sc_new<m_classifier_float>("classifier");
    adder = sc_new<m_adder_float>("adder");
    multiplier = sc_new<m_multiplier_float>("multiplier");
    divisor = sc_new<m_divisor_float>("divisor");
    compare = sc_new<m_compare_float>("compare");
    converter = sc_new<m_converter_float>("converter");
    sqrt = sc_new<m_sqrt_float>("sqrt");
    mul_add = sc_new<m_mul_add_float>("mul_add");

    regfile->clk(clk);
    regfile->reset(reset);
    regfile->data_in(data_in);
    regfile->select_in(select_in);
    regfile->rs1_select_in(rs1_select_in);
    regfile->rs2_select_in(rs2_select_in);
    regfile->rs3_select_in(rs3_select_in);
    regfile->en_load_in(en_load_in);

    regfile->rs1_out(signal_regfile_rs1_out);
    regfile->rs2_out(signal_regfile_rs2_out);
    regfile->rs3_out(signal_regfile_rs3_out);

    classifier->clk(clk);
    classifier->reset(reset);
    classifier->data_in(signal_regfile_rs1_out);
    classifier->data_out(signal_classifier_data_out);

    adder->clk(clk);
    adder->reset(reset);
    adder->stb_in(signal_adder_stb_in);
    adder->a_in(signal_a);
    adder->b_in(signal_b);
    adder->rounding_mode_in(signal_rounding_mode);
    adder->result_out(signal_adder_result_out);
    adder->ready_out(signal_adder_ready);
    adder->done_out(signal_adder_done);

    multiplier->clk(clk);
    multiplier->reset(reset);
    multiplier->stb_in(signal_multiplier_stb_in);
    multiplier->a_in(signal_a);
    multiplier->b_in(signal_b);
    multiplier->rounding_mode_in(signal_rounding_mode);
    multiplier->result_out(signal_multiplier_result_out);
    multiplier->ready_out(signal_multiplier_ready);
    multiplier->done_out(signal_multiplier_done);

    divisor->clk(clk);
    divisor->reset(reset);
    divisor->stb_in(signal_divisor_stb_in);
    divisor->a_in(signal_a);
    divisor->b_in(signal_b);
    divisor->rounding_mode_in(signal_rounding_mode);
    divisor->result_out(signal_divisor_result_out);
    divisor->ready_out(signal_divisor_ready);
    divisor->done_out(signal_divisor_done);

    compare->clk(clk);
    compare->reset(reset);
    compare->a_in(signal_regfile_rs1_out);
    compare->b_in(signal_regfile_rs2_out);
    compare->mode_in(signal_rounding_mode);
    compare->data_out(signal_compare_data_out);

    converter->clk(clk);
    converter->reset(reset);
    converter->stb_in(signal_converter_stb_in);
    converter->a_in(signal_a);
    converter->b_in(signal_b);
    converter->rounding_mode_in(signal_rounding_mode);
    converter->cvt_mode_in(signal_convert_mode);
    converter->result_out(signal_converter_result_out);
    converter->ready_out(signal_converter_ready);
    converter->done_out(signal_converter_done);

    sqrt->clk(clk);
    sqrt->reset(reset);
    sqrt->stb_in(signal_sqrt_stb_in);
    sqrt->a_in(signal_a);
    sqrt->rounding_mode_in(signal_rounding_mode);
    sqrt->result_out(signal_sqrt_result_out);
    sqrt->ready_out(signal_sqrt_ready);
    sqrt->done_out(signal_sqrt_done);

    mul_add->clk(clk);
    mul_add->reset(reset);
    mul_add->stb_in(signal_mul_add_stb_in);
    mul_add->a_in(signal_a);
    mul_add->b_in(signal_b);
    mul_add->c_in(signal_c);
    mul_add->negate_in(signal_negate);
    mul_add->rounding_mode_in(signal_rounding_mode);
    mul_add->result_out(signal_mul_add_result_out);
    mul_add->ready_out(signal_mul_add_ready);
    mul_add->done_out(signal_mul_add_done);
}

void m_f_extension::pn_trace(sc_trace_file* tf, int level) {
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, data_in);
    PN_TRACE(tf, select_in);
    PN_TRACE(tf, rs1_select_in);
    PN_TRACE(tf, rs2_select_in);
    PN_TRACE(tf, rs3_select_in);
    PN_TRACE(tf, en_load_in);

    PN_TRACE(tf, stb_in);
    PN_TRACE(tf, instruction_in);
    PN_TRACE(tf, rounding_mode_in);
    PN_TRACE(tf, ready_out);
    PN_TRACE(tf, f_out);
    
    PN_TRACE(tf, state);

    PN_TRACE(tf, signal_regfile_rs1_out);
    PN_TRACE(tf, signal_regfile_rs2_out);
    PN_TRACE(tf, signal_regfile_rs3_out);

    PN_TRACE(tf, signal_rounding_mode);

    PN_TRACE(tf, signal_compare_data_out);

    PN_TRACE(tf, signal_adder_stb_in);
    PN_TRACE(tf, signal_adder_result_out);
    PN_TRACE(tf, signal_adder_ready);
    PN_TRACE(tf, signal_adder_done);

    PN_TRACE(tf, signal_multiplier_stb_in);
    PN_TRACE(tf, signal_multiplier_result_out);
    PN_TRACE(tf, signal_multiplier_ready);
    PN_TRACE(tf, signal_multiplier_done);

    PN_TRACE(tf, signal_divisor_stb_in);
    PN_TRACE(tf, signal_divisor_result_out);
    PN_TRACE(tf, signal_divisor_ready);
    PN_TRACE(tf, signal_divisor_done);

    PN_TRACE(tf, signal_converter_stb_in);
    PN_TRACE(tf, signal_converter_result_out);
    PN_TRACE(tf, signal_converter_ready);
    PN_TRACE(tf, signal_converter_done);

    PN_TRACE(tf, signal_sqrt_stb_in);
    PN_TRACE(tf, signal_sqrt_result_out);
    PN_TRACE(tf, signal_sqrt_ready);
    PN_TRACE(tf, signal_sqrt_done);

    PN_TRACE(tf, signal_mul_add_stb_in);
    PN_TRACE(tf, signal_mul_add_result_out);
    PN_TRACE(tf, signal_mul_add_ready);
    PN_TRACE(tf, signal_mul_add_done);
    PN_TRACE(tf, signal_c);
    PN_TRACE(tf, signal_negate);

    if(level >= 2)
    {
        regfile->pn_trace(tf, level);
        classifier->pn_trace(tf, level);
        adder->pn_trace(tf, level);
        multiplier->pn_trace(tf, level);
        divisor->pn_trace(tf, level);
        compare->pn_trace(tf, level);
        converter->pn_trace(tf, level);
        sqrt->pn_trace(tf, level);
        mul_add->pn_trace(tf, level);
    }
}

void m_f_extension::proc_clk_f_extension() {
    state.write(F_STATE_WAIT);
    f_out_reg.write(0);
    ready_out.write(1);
    f_out.write(0);
    rs2_out.write(0);

    signal_rounding_mode.write(0);
    signal_a.write(0);
    signal_b.write(0);
    signal_c.write(0);
    signal_convert_mode.write(0);

    signal_adder_stb_in.write(0);
    signal_multiplier_stb_in.write(0);
    signal_divisor_stb_in.write(0);
    signal_converter_stb_in.write(0);
    signal_sqrt_stb_in.write(0);
    signal_mul_add_stb_in.write(0);

    signal_negate.write(0);

    while(true) {
        wait();

        // Continuous Output Drivers
        f_out.write(f_out_reg.read());
        rs2_out.write(signal_regfile_rs2_out.read());
        ready_out.write(state.read() == F_STATE_WAIT);

        // Strobe Drop-offs: Guarantee 1-cycle pulses for submodules
        signal_adder_stb_in.write(0);
        signal_multiplier_stb_in.write(0);
        signal_divisor_stb_in.write(0);
        signal_converter_stb_in.write(0);
        signal_sqrt_stb_in.write(0);
        signal_mul_add_stb_in.write(0);

        signal_negate.write(0);

        sc_uint<3> rm_val = (rounding_mode_in.read() == 0b111) ? rm(instruction_in.read()) : rounding_mode_in.read();

        sc_uint<32> instruction = instruction_in.read();

        switch(state.read()) {
            case F_STATE_WAIT: {
                if (stb_in.read()) {
                    state.write(F_STATE_DECODE);
                }
                break;
            }

            case F_STATE_DECODE: {
                if (opcode(instruction) == OPF_FMADD) {
                    state.write(F_STATE_FMADD_REQUEST);
                } else if (opcode(instruction) == OPF_FMSUB) {
                    state.write(F_STATE_FMSUB_REQUEST);
                } else if (opcode(instruction) == OPF_FNMSUB) {
                    state.write(F_STATE_FNMSUB_REQUEST);
                } else if (opcode(instruction) == OPF_FNMADD) {
                    state.write(F_STATE_FNMADD_REQUEST);
                } else {
                    switch (funct5(instruction)) {
                        case FUNCT5F_CLASS_MV_X_W:
                            state.write(rm(instruction_in.read()) ? F_STATE_CLASS : F_STATE_MV_OUT);
                            break;
                        case FUNCT5F_COMP:
                            signal_rounding_mode.write(rm(instruction_in.read()));
                            state.write(F_STATE_COMP_REQUEST);
                            break;
                        case FUNCT5F_ADD:
                            state.write(F_STATE_ADD_REQUEST);
                            break;
                        case FUNCT5F_SUB:
                            state.write(F_STATE_SUB_REQUEST);
                            break;
                        case FUNCT5F_MULT:
                            state.write(F_STATE_MULT_REQUEST);
                            break;
                        case FUNCT5F_DIV:
                            state.write(F_STATE_DIV_REQUEST);
                            break;
                        case FUNCT5F_SQRT:
                            state.write(F_STATE_SQRT_REQUEST);
                            break;
                        case FUNCT5F_CVT_I_TO_F:
                            state.write(F_STATE_CVT_I_TO_F_REQUEST);
                            break;
                        case FUNCT5F_CVT_F_TO_I:
                            state.write(F_STATE_CVT_F_TO_I_REQUEST);
                            break;
                        case FUNCT5F_SGNJ:
                            state.write(F_STATE_SGNJ_REQUEST);
                            break;
                        default:
                            state.write(F_STATE_WAIT);
                            break;
                    }
                }
                break;
            }

            case F_STATE_CLASS: {
                f_out_reg.write(signal_classifier_data_out.read());
                state.write(F_STATE_WAIT);
                break;
            }

            case F_STATE_COMP_REQUEST: {
                state.write(F_STATE_COMP_BUSY);
                break;
            }

            case F_STATE_COMP_BUSY: {
                f_out_reg.write(signal_compare_data_out.read());
                state.write(F_STATE_WAIT);
                break;
            }

            case F_STATE_MV_OUT: {
                f_out_reg.write(signal_regfile_rs1_out.read());
                state.write(F_STATE_WAIT);
                break;
            }

            // --- ADDER / SUBTRACTOR ---
            case F_STATE_SUB_REQUEST: {
                signal_adder_stb_in.write(1);
                signal_a.write(signal_regfile_rs1_out.read());
                
                // Safe bitref workaround for Subtraction sign flip
                sc_uint<32> rs2_val = signal_regfile_rs2_out.read();
                rs2_val.range(31, 31) = rs2_val.range(31, 31) ^ 1; 
                signal_b.write(rs2_val);
                
                signal_rounding_mode.write(rm_val);
                state.write(F_STATE_ADD_BUSY);
                break;
            }

            case F_STATE_ADD_REQUEST: {
                signal_adder_stb_in.write(1);
                signal_a.write(signal_regfile_rs1_out.read());
                signal_b.write(signal_regfile_rs2_out.read());
                signal_rounding_mode.write(rm_val);
                state.write(F_STATE_ADD_BUSY);
                break;
            }

            case F_STATE_ADD_BUSY: {
                if (signal_adder_done.read() == 1) {
                    f_out_reg.write(signal_adder_result_out.read());
                    state.write(F_STATE_WAIT);
                }
                break;
            }

            // --- MULTIPLIER ---
            case F_STATE_MULT_REQUEST: {
                signal_multiplier_stb_in.write(1);
                signal_a.write(signal_regfile_rs1_out.read());
                signal_b.write(signal_regfile_rs2_out.read());
                signal_rounding_mode.write(rm_val);
                state.write(F_STATE_MULT_BUSY);
                break;
            }

            case F_STATE_MULT_BUSY: {
                if (signal_multiplier_done.read() == 1) {
                    f_out_reg.write(signal_multiplier_result_out.read());
                    state.write(F_STATE_WAIT);
                }
                break;
            }

            // --- DIVISOR ---
            case F_STATE_DIV_REQUEST: {
                signal_divisor_stb_in.write(1);
                signal_a.write(signal_regfile_rs1_out.read());
                signal_b.write(signal_regfile_rs2_out.read());
                signal_rounding_mode.write(rm_val);
                state.write(F_STATE_DIV_BUSY);
                break;
            }

            case F_STATE_DIV_BUSY: {
                if (signal_divisor_done.read() == 1) {
                    f_out_reg.write(signal_divisor_result_out.read());
                    state.write(F_STATE_WAIT);
                }
                break;
            }

            // --- SQRT ---
            case F_STATE_SQRT_REQUEST: {
                signal_sqrt_stb_in.write(1);
                signal_a.write(signal_regfile_rs1_out.read());
                signal_rounding_mode.write(rm_val);
                state.write(F_STATE_SQRT_BUSY);
                break;
            }

            case F_STATE_SQRT_BUSY: {
                if (signal_sqrt_done.read() == 1) {
                    f_out_reg.write(signal_sqrt_result_out.read());
                    state.write(F_STATE_WAIT);
                }
                break;
            }

            // --- CONVERTER (F to I) ---
            case F_STATE_CVT_F_TO_I_REQUEST: {
                signal_converter_stb_in.write(1);
                signal_a.write(signal_regfile_rs1_out.read());
                signal_rounding_mode.write(rm_val);
                signal_convert_mode.write(rs2(instruction_in.read()) == 0 ? F32_TO_I32 : F32_TO_UI32);
                state.write(F_STATE_CVT_F_TO_I_BUSY);
                break;
            }

            case F_STATE_CVT_F_TO_I_BUSY: {
                if (signal_converter_done.read() == 1) {
                    f_out_reg.write(signal_converter_result_out.read());
                    state.write(F_STATE_WAIT);
                }
                break;
            }

            // --- CONVERTER (I to F) ---
            case F_STATE_CVT_I_TO_F_REQUEST: {
                signal_converter_stb_in.write(1);
                signal_a.write(data_in.read());
                signal_rounding_mode.write(rm_val);
                signal_convert_mode.write(rs2(instruction_in.read()) == 0 ? I32_TO_F32 : UI32_TO_F32);
                state.write(F_STATE_CVT_I_TO_F_BUSY);
                break;
            }

            case F_STATE_CVT_I_TO_F_BUSY: {
                if (signal_converter_done.read() == 1) {
                    f_out_reg.write(signal_converter_result_out.read());
                    state.write(F_STATE_WAIT);
                }
                break;
            }

            // --- SIGN INJECTION ---
            case F_STATE_SGNJ_REQUEST: {
                signal_converter_stb_in.write(1);
                signal_a.write(signal_regfile_rs1_out.read());
                signal_b.write(signal_regfile_rs2_out.read());
                signal_rounding_mode.write(rm(instruction_in.read()));
                signal_convert_mode.write(FSGNJ);
                state.write(F_STATE_SGNJ_BUSY);
                break;
            }

            case F_STATE_SGNJ_BUSY: {
                if (signal_converter_done.read() == 1) {
                    f_out_reg.write(signal_converter_result_out.read());
                    state.write(F_STATE_WAIT);
                }
                break;
            }

            // --- FUSED MULTIPLY ADD ---
            case F_STATE_FMADD_REQUEST: {
                signal_mul_add_stb_in.write(1);
                signal_a.write(signal_regfile_rs1_out.read());
                signal_b.write(signal_regfile_rs2_out.read());
                signal_c.write(signal_regfile_rs3_out.read());
                signal_negate.write(0);
                signal_rounding_mode.write(rm_val);
                state.write(F_STATE_FMADD_BUSY);
                break;
            }

            case F_STATE_FMSUB_REQUEST: {
                signal_mul_add_stb_in.write(1);
                signal_a.write(signal_regfile_rs1_out.read());
                signal_b.write(signal_regfile_rs2_out.read());
                sc_uint<32> rs3_val = signal_regfile_rs3_out.read();
                rs3_val.range(31, 31) = rs3_val.range(31, 31) ^ 1; 
                signal_c.write(rs3_val);
                signal_negate.write(0);
                signal_rounding_mode.write(rm_val);
                state.write(F_STATE_FMADD_BUSY);
                break;
            }

            case F_STATE_FNMADD_REQUEST: {
                signal_mul_add_stb_in.write(1);
                signal_a.write(signal_regfile_rs1_out.read());
                signal_b.write(signal_regfile_rs2_out.read());
                sc_uint<32> rs3_val = signal_regfile_rs3_out.read();
                rs3_val.range(31, 31) = rs3_val.range(31, 31) ^ 1; 
                signal_c.write(rs3_val);
                signal_negate.write(1);
                signal_rounding_mode.write(rm_val);
                state.write(F_STATE_FMADD_BUSY);
                break;
            }

            case F_STATE_FNMSUB_REQUEST: {
                signal_mul_add_stb_in.write(1);
                signal_a.write(signal_regfile_rs1_out.read());
                signal_b.write(signal_regfile_rs2_out.read());
                signal_c.write(signal_regfile_rs3_out.read());
                signal_negate.write(1);
                signal_rounding_mode.write(rm_val);
                state.write(F_STATE_FMADD_BUSY);
                break;
            }

            case F_STATE_FMADD_BUSY: {
                if (signal_mul_add_done.read() == 1) {
                    f_out_reg.write(signal_mul_add_result_out.read());
                    state.write(F_STATE_WAIT);
                }
                break;
            }

            default:
                state.write(F_STATE_WAIT);
                break;
        }
    }
}