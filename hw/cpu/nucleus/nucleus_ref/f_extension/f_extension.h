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

#ifndef __F_EXTENSION_H__
#define __F_EXTENSION_H__

#include <piconut.h>

/**
 * @fn SC_MODULE(m_f_extension)
 * @brief Floating-Point Number Module interfacing all operations.
 *
 * At the moment, this module supports single precision Floating-Point Numbers
 *  according to the RISC-V F-Extension. This module can be modified and extended to support
 *  different precisions. (D, Q, Zfh)
 *
 * This module implements a simple state machine to interact with the different floating-point modules.
 *
 * @par Ports:
 * @param[in] clk Clock of the module.
 * @param[in] reset Reset of the module.
 * @param[in] data_in Data input for the internal floating-point register.
 * @param[in] select_in Data input selection.
 * @param[in] en_load_in Load enable for data input.
 * @param[in] rs1_select_in Data output selection for rs1.
 * @param[in] rs2_select_in Data output selection for rs2.
 * @param[in] rs3_select_in Data output selection for rs3.
 * @param[in] stb_in Strobe input for starting operation.
 * @param[in] instruction_in Instruction input for decoding.
 * @param[in] rounding_mode_in Rounding mode select from CSR. 
 * 
 * @param[out] rs2_out Route out for rs2.
 * @param[out] ready_out Ready signaling.
 * @param[out] f_out Operation output. 
 * 
 */
SC_MODULE(m_f_extension) {
    public:
        sc_in_clk PN_NAME(clk);
        sc_in<bool> PN_NAME(reset);

        // regfile input
        sc_in<sc_uint<32>> PN_NAME(data_in);
        sc_in<sc_uint<5>> PN_NAME(select_in);
        sc_in<sc_uint<5>> PN_NAME(rs1_select_in);
        sc_in<sc_uint<5>> PN_NAME(rs2_select_in);
        sc_in<sc_uint<5>> PN_NAME(rs3_select_in);
        sc_in<bool> PN_NAME(en_load_in);

        // regfile output
        sc_out<sc_uint<32>> PN_NAME(rs2_out);
        
        sc_in<bool> PN_NAME(stb_in);
        sc_in<sc_uint<32>> PN_NAME(instruction_in);
        sc_in<sc_uint<3>> PN_NAME(rounding_mode_in);

        sc_out<bool> PN_NAME(ready_out);
        sc_out<sc_uint<32>> PN_NAME(f_out);

        SC_CTOR(m_f_extension) {
#if (PN_CFG_ENABLE_F_EXTENSION == 1)
            init_submodules();
#endif
            SC_CTHREAD(proc_clk_f_extension, clk.pos());
            reset_signal_is(reset, true);
        }

#if (PN_CFG_ENABLE_F_EXTENSION == 1)
        sc_uint<5> funct5(sc_uint<32> i) { return i.range(31, 27); }
        sc_uint<5> rs2(sc_uint<32> i) { return i.range(24, 20); }
        sc_uint<3> rm(sc_uint<32> i) { return i.range(14, 12); }
        sc_uint<7> opcode(sc_uint<32> i) { return i.range(6, 0); }
#endif
        void pn_trace(sc_trace_file *tf, int level = 1);
        void proc_clk_f_extension();

#if (PN_CFG_ENABLE_F_EXTENSION == 1)
        class m_regfile_float* regfile;
        class m_classifier_float* classifier;
        class m_adder_float* adder;
        class m_multiplier_float* multiplier;
        class m_divisor_float* divisor;
        class m_compare_float* compare;
        class m_converter_float* converter;
        class m_sqrt_float* sqrt;
        class m_mul_add_float* mul_add;
#endif

    protected:
#if (PN_CFG_ENABLE_F_EXTENSION == 1)
        void init_submodules();
        
        sc_signal<sc_uint<32>> PN_NAME(signal_regfile_rs1_out);
        sc_signal<sc_uint<32>> PN_NAME(signal_regfile_rs2_out);
        sc_signal<sc_uint<32>> PN_NAME(signal_regfile_rs3_out);

        sc_signal<sc_uint<32>> PN_NAME(signal_a);
        sc_signal<sc_uint<32>> PN_NAME(signal_b);
        sc_signal<sc_uint<32>> PN_NAME(signal_c);

        sc_signal<sc_uint<32>> PN_NAME(signal_classifier_data_out);

        sc_signal<sc_uint<1>> PN_NAME(signal_compare_data_out);

        sc_signal<sc_uint<3>> PN_NAME(signal_rounding_mode);

        sc_signal<sc_uint<3>> PN_NAME(signal_convert_mode);

        sc_signal<bool> PN_NAME(signal_adder_stb_in);
        sc_signal<sc_uint<32>> PN_NAME(signal_adder_result_out);
        sc_signal<bool> PN_NAME(signal_adder_ready);
        sc_signal<bool> PN_NAME(signal_adder_done);

        sc_signal<bool> PN_NAME(signal_multiplier_stb_in);
        sc_signal<sc_uint<32>> PN_NAME(signal_multiplier_result_out);
        sc_signal<bool> PN_NAME(signal_multiplier_ready);
        sc_signal<bool> PN_NAME(signal_multiplier_done);

        sc_signal<bool> PN_NAME(signal_divisor_stb_in);
        sc_signal<sc_uint<32>> PN_NAME(signal_divisor_result_out);
        sc_signal<bool> PN_NAME(signal_divisor_ready);
        sc_signal<bool> PN_NAME(signal_divisor_done);

        sc_signal<bool> PN_NAME(signal_converter_stb_in);
        sc_signal<sc_uint<32>> PN_NAME(signal_converter_result_out);
        sc_signal<bool> PN_NAME(signal_converter_ready);
        sc_signal<bool> PN_NAME(signal_converter_done);

        sc_signal<bool> PN_NAME(signal_sqrt_stb_in);
        sc_signal<sc_uint<32>> PN_NAME(signal_sqrt_result_out);
        sc_signal<bool> PN_NAME(signal_sqrt_ready);
        sc_signal<bool> PN_NAME(signal_sqrt_done);

        sc_signal<bool> PN_NAME(signal_negate);
        sc_signal<bool> PN_NAME(signal_mul_add_stb_in);
        sc_signal<sc_uint<32>> PN_NAME(signal_mul_add_result_out);
        sc_signal<bool> PN_NAME(signal_mul_add_ready);
        sc_signal<bool> PN_NAME(signal_mul_add_done);

        sc_signal<sc_uint<6>> PN_NAME(state);

        sc_signal<sc_uint<32>> PN_NAME(f_out_reg);
#endif
};

#endif //__F_EXTENSION_H__