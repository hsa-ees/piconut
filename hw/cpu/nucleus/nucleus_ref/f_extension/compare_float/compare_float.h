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

#ifndef __COMPARE_FLOAT_H__
#define __COMPARE_FLOAT_H__

#include <systemc.h>
#include <piconut.h>

/**
 * @fn SC_MODULE(m_compare_float)
 * @brief Floating-point comparison module.
 * This module compares two 32-bit floating-point numbers (IEEE 754 Single Precision)
 * according to a specified comparison condition (Less than or Equal, Less than, or Equal).
 * @par Ports:
 * @param[in] clk Clock of the module.
 * @param[in] reset Reset of the module.
 * @param[in] a_in 32-bit floating-point operand A.
 * @param[in] b_in 32-bit floating-point operand B.
 * @param[in] mode_in 3-bit selection flag to dictate the comparison operation.
 * @param[out] data_out 1-bit boolean evaluation result (1 if the condition is met, 0 otherwise).
 */
SC_MODULE(m_compare_float) {
    public:
        sc_in_clk PN_NAME(clk);
        sc_in<bool> PN_NAME(reset);

        sc_in<sc_uint<32>> PN_NAME(a_in);
        sc_in<sc_uint<32>> PN_NAME(b_in);
        sc_in<sc_uint<3>> PN_NAME(mode_in);

        sc_out<sc_uint<1>> PN_NAME(data_out);

        SC_CTOR(m_compare_float) {
            SC_CTHREAD(proc_clk_compare_float, clk.pos());
            reset_signal_is(reset, true);
        }

        sc_uint<1> sign(sc_uint<32> float_in) { return float_in.range(31, 31); }
        sc_uint<8> exponent(sc_uint<32> float_in) { return float_in.range(30, 23); }
        sc_uint<23> mantissa(sc_uint<32> float_in) { return float_in.range(22, 0); }

        void pn_trace(sc_trace_file * tf, int level = 1);

        void proc_clk_compare_float();
};

#endif // __COMPARE_FLOAT_H__