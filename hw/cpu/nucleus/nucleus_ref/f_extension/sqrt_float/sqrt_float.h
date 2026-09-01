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

#ifndef __SQRT_FLOAT_H__
#define __SQRT_FLOAT_H__

#include <systemc.h>
#include <piconut.h>

/**
 * @fn SC_MODULE(m_sqrt_float)
 * @brief Floating-point square root module.
 * 
 * This module performs floating-point square root operations on 
 * a 32-bit floating-point number, complying with IEEE 754 standards and 
 * configurable rounding modes.
 * @par Ports:
 * @param[in] clk Clock of the module.
 * @param[in] reset Reset of the module.
 * @param[in] stb_in Strobe signal; when high, it initiates the floating-point operation.
 * @param[in] a_in 32-bit floating-point operand A.
 * @param[in] rounding_mode_in 3-bit code specifying the IEEE 754 rounding mode (e.g., Round to Nearest).
 * @param[out] result_out 32-bit floating-point result of the square root.
 * @param[out] ready_out Status signal indicating the module is free to accept a new operation.
 * @param[out] done_out Status signal indicating the current operation is complete and the result is valid.
 */
SC_MODULE(m_sqrt_float) {
    public:
        sc_in_clk           PN_NAME(clk);
        sc_in<bool>         PN_NAME(reset);

        sc_in<bool>         PN_NAME(stb_in);

        sc_in<sc_uint<32> > PN_NAME(a_in);
        sc_in<sc_uint<3> >  PN_NAME(rounding_mode_in);

        sc_out<sc_uint<32> > PN_NAME(result_out);

        sc_out<bool>        PN_NAME(ready_out);
        sc_out<bool>        PN_NAME(done_out);

        SC_CTOR(m_sqrt_float) {
            // Single clocked thread — HLS friendly
            SC_CTHREAD(proc_clk_sqrt_float, clk.pos());
            reset_signal_is(reset, true);
        }

        sc_uint<1> sign(sc_uint<32> float_in) { return float_in.range(31, 31); }
        sc_uint<8> exponent(sc_uint<32> float_in) { return float_in.range(30, 23); }
        sc_uint<23> mantissa(sc_uint<32> float_in) { return float_in.range(22, 0); }

        void pn_trace(sc_trace_file *tf, int level = 1);
        void proc_clk_sqrt_float();

    protected:
        sc_signal<sc_uint<3> > PN_NAME(state);

        // --- Initial input capture ---
        sc_signal<sc_uint<1> >  PN_NAME(reg_init_sign);
        sc_signal<sc_uint<8> >  PN_NAME(reg_init_exp);
        sc_signal<sc_uint<23> > PN_NAME(reg_init_mant);
        sc_signal<sc_uint<3> >  PN_NAME(reg_init_rm);

        // --- Iterative square-root working set ---
        // 54-bit radicand, 27 iterations -> 27 result bits
        sc_signal<sc_uint<54> > PN_NAME(reg_radicand);
        sc_signal<sc_uint<29> > PN_NAME(reg_rem);      // running partial remainder
        sc_signal<sc_uint<27> > PN_NAME(reg_q);        // running quotient (result)
        sc_signal<sc_uint<5> >  PN_NAME(reg_iter);     // 0..27 iteration counter
        sc_signal<sc_int<10> >  PN_NAME(reg_new_exp);  // biased result exponent

        // --- Rounding-stage pipeline registers ---
        sc_signal<sc_uint<24> > PN_NAME(reg_result_mant);
        sc_signal<sc_int<10> >  PN_NAME(reg_result_exp);
        sc_signal<sc_uint<1> >  PN_NAME(reg_g);
        sc_signal<sc_uint<1> >  PN_NAME(reg_r);
        sc_signal<sc_uint<1> >  PN_NAME(reg_s);
};

#endif // __SQRT_FLOAT_H__