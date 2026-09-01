/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2026 Tristan Kundrat <tristan.kundrat@tha.de>
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

#ifndef __SHIFT_AND_ADD_H__
#define __SHIFT_AND_ADD_H__

#include <systemc.h>
#include <piconut.h>

#include "../../../nucleus_ref_defs.h"

// Disables skipping multiplication, if a and b are the same as last time, if 1
#ifndef MUL_SECURITY
#define MUL_SECURITY 0
#endif // MUL_SECURITY

SC_MODULE(m_shift_and_add)
{
public:
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    sc_in<bool> PN_NAME(start_in);
    sc_in<sc_uint<NUCLEUS_DATA_WIDTH>> PN_NAME(a_in);
    sc_in<sc_uint<NUCLEUS_DATA_WIDTH>> PN_NAME(b_in);
    sc_in<sc_uint<3>> PN_NAME(funct3_in);

    sc_out<bool> PN_NAME(valid_out);
    sc_out<sc_uint<NUCLEUS_DATA_WIDTH>> PN_NAME(y_out);

    /* Constructor... */
    SC_CTOR(m_shift_and_add)
    {
        SC_CTHREAD(proc_clk_shift_and_add, clk.pos());
        reset_signal_is(reset, true);

        SC_METHOD(proc_cmb_sign);
        sensitive << a_cache << b_cache << funct3_in;

        SC_METHOD(proc_cmb_output);
        sensitive << state << result << negative_result << funct3_in;
    }

    void pn_trace(sc_trace_file * tf, int level = 1);

    // Clocked processes
    void proc_clk_shift_and_add();
    // Combinatorial processes
    void proc_cmb_sign();
    void proc_cmb_output();

protected:
    sc_signal<sc_uint<3>> PN_NAME(funct3_cache);
    sc_signal<sc_uint<NUCLEUS_DATA_WIDTH>> PN_NAME(a_cache);
    sc_signal<sc_uint<NUCLEUS_DATA_WIDTH>> PN_NAME(b_cache);
    sc_signal<sc_uint<NUCLEUS_DATA_WIDTH>> PN_NAME(abs_a);
    sc_signal<sc_uint<NUCLEUS_DATA_WIDTH>> PN_NAME(abs_b);
    sc_signal<bool> PN_NAME(negative_result);
    sc_signal<sc_uint<NUCLEUS_DATA_WIDTH * 2>> PN_NAME(result);
    sc_signal<sc_uint<2>> PN_NAME(state);
    sc_signal<sc_uint<NUCLEUS_DATA_WIDTH_LOG2>> PN_NAME(digit);

private:
    // helper functions
    sc_uint<NUCLEUS_DATA_WIDTH> abs(sc_int<NUCLEUS_DATA_WIDTH>);

    // enums
    enum shift_and_add_state_t
    {
        S_MUL_IDLE = 0b00,
        S_MUL_WORKING = 0b01,
        S_MUL_DONE = 0b10,
    };
};

#endif //__SHIFT_AND_ADD_H__
