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

#include "shift_and_add.h"
#include "pn_base.h"
#include <unistd.h>

void m_shift_and_add::pn_trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, a_in);
    PN_TRACE(tf, b_in);
    PN_TRACE(tf, start_in);
    PN_TRACE(tf, funct3_in);
    PN_TRACE(tf, valid_out);
    PN_TRACE(tf, y_out);

    PN_TRACE(tf, a_cache);
    PN_TRACE(tf, b_cache);
    PN_TRACE(tf, abs_a);
    PN_TRACE(tf, abs_b);
    PN_TRACE(tf, negative_result);
    PN_TRACE(tf, result);
    PN_TRACE(tf, state);
    PN_TRACE(tf, digit);
}

sc_uint<NUCLEUS_DATA_WIDTH> m_shift_and_add::abs(sc_int<NUCLEUS_DATA_WIDTH> x)
{
    if(x[NUCLEUS_DATA_WIDTH - 1] == 1)
    { // negative
        return (sc_uint<NUCLEUS_DATA_WIDTH>)(0 - x);
    }
    else
    { // positive
        return (sc_uint<NUCLEUS_DATA_WIDTH>)x;
    }
}

void m_shift_and_add::proc_cmb_sign()
{
    bool a_neg = (bool)a_cache.read()[NUCLEUS_DATA_WIDTH - 1];
    bool b_neg = (bool)b_cache.read()[NUCLEUS_DATA_WIDTH - 1];

    negative_result = false;
    abs_a = 0x0;
    abs_b = 0x0;

    switch(funct3_in.read())
    {
        case FUNCT3_ADD_SUB_MUL:
            // lower half of bits: treat as signed*signed so we can reuse upper half easiely
        case FUNCT3_SLL_MULH:
            // signed * signed: negative, when numbers have different sign
            negative_result = (a_neg != b_neg);
            abs_a = abs((sc_int<NUCLEUS_DATA_WIDTH>)a_cache.read());
            abs_b = abs((sc_int<NUCLEUS_DATA_WIDTH>)b_cache.read());
            break;

        case FUNCT3_SLTU_MULHU:
            // unsigned * unsigned: always positive
            abs_a = a_cache.read();
            abs_b = b_cache.read();
            break;

        case FUNCT3_SLT_MULHSU:
            // signed * unsigned: negative, when signed (a) is negative
            negative_result = a_neg;
            abs_a = abs((sc_int<NUCLEUS_DATA_WIDTH>)a_cache.read());
            abs_b = b_cache.read();
            break;
    }
}

void m_shift_and_add::proc_clk_shift_and_add()
{
    // Reset values
    result = 0x0;
    digit = 0x0;
    state = S_MUL_IDLE;
    a_cache = 0x0;
    b_cache = 0x0;
    funct3_cache = 0x0;

    while(true)
    {
        wait();

        // Default values
        sc_uint<2> new_state = state.read();
        sc_uint<NUCLEUS_DATA_WIDTH * 2> new_result = result.read();
        sc_uint<NUCLEUS_DATA_WIDTH_LOG2> new_digit = digit.read();
        sc_uint<NUCLEUS_DATA_WIDTH> new_a_cache = a_cache.read();
        sc_uint<NUCLEUS_DATA_WIDTH> new_b_cache = b_cache.read();
        sc_uint<NUCLEUS_DATA_WIDTH> new_funct3_cache = funct3_cache.read();

        switch(state.read())
        {
            case S_MUL_IDLE:
                new_digit = 0x0;
                if(start_in.read() == 0x1)
                {
#if MUL_SECURITY <= 0
                    if(a_cache.read() == a_in.read() && b_cache.read() == b_in.read() &&
                        (funct3_cache.read() == funct3_in.read() ||
                            (funct3_cache.read() == FUNCT3_ADD_SUB_MUL && funct3_in.read() == FUNCT3_SLL_MULH) ||
                            (funct3_in.read() == FUNCT3_ADD_SUB_MUL && funct3_cache.read() == FUNCT3_SLL_MULH)))
                    {
                        // skip actual multiplication, if input is the same.
                        new_state = S_MUL_DONE;
                    }
                    else
#endif // MUL_SECURITY <= 0
                    {
                        new_state = S_MUL_WORKING;
                        new_result = 0x0;
                    }
                    new_a_cache = a_in.read();
                    new_b_cache = b_in.read();
                    new_funct3_cache = funct3_in.read();
                }
                break;

            case S_MUL_WORKING:
                if(abs_a.read()[digit.read()] == 0b1)
                {
                    new_result += abs_b.read() << digit.read();
                }

                new_digit = digit.read() + 1;
                if(digit.read() == (NUCLEUS_DATA_WIDTH - 1))
                {
                    new_state = S_MUL_DONE;
                }
                break;

            case S_MUL_DONE:
                new_state = S_MUL_IDLE;
                break;
        }
        result = new_result;
        state = new_state;
        digit = new_digit;
        a_cache = new_a_cache;
        b_cache = new_b_cache;
        funct3_cache = new_funct3_cache;
    }
}

void m_shift_and_add::proc_cmb_output()
{
    valid_out = 0x0;
    y_out = 0x0;

    if(state.read() == S_MUL_DONE)
    {
        switch(funct3_in.read())
        {
            case FUNCT3_ADD_SUB_MUL:
                // lower half of bits
                if(negative_result.read())
                {
                    y_out = (sc_uint<NUCLEUS_DATA_WIDTH>)(0 - result.read());
                }
                else
                {
                    y_out = (sc_uint<NUCLEUS_DATA_WIDTH>)result.read();
                }
                break;

            case FUNCT3_SLL_MULH:
            case FUNCT3_SLT_MULHSU:
                // upper half of bits
                if(negative_result.read())
                {
                    y_out = (sc_uint<NUCLEUS_DATA_WIDTH>)((0 - result.read()) >> NUCLEUS_DATA_WIDTH);
                }
                else
                {
                    y_out = (sc_uint<NUCLEUS_DATA_WIDTH>)(result.read() >> NUCLEUS_DATA_WIDTH);
                }
                break;

            case FUNCT3_SLTU_MULHU:
                y_out = (sc_uint<NUCLEUS_DATA_WIDTH>)(result.read() >> NUCLEUS_DATA_WIDTH);
                break;

            default:
                PN_ERRORF(("Invalid Funct3 for multiplication: 0x%x", funct3_in.read()));
                break;
        }

        valid_out = 0x1;
    }
}
