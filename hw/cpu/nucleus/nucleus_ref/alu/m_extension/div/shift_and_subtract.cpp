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

#include "shift_and_subtract.h"
#include "pn_base.h"
#include <sysc/datatypes/int/sc_uint.h>

void m_shift_and_subtract::pn_trace(sc_trace_file* tf, int level)
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
    PN_TRACE(tf, quotient);
    PN_TRACE(tf, remainder);
    PN_TRACE(tf, state);
    PN_TRACE(tf, digit);
}

sc_uint<NUCLEUS_DATA_WIDTH> m_shift_and_subtract::abs(sc_int<NUCLEUS_DATA_WIDTH> x)
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

void m_shift_and_subtract::proc_cmb_sign()
{
    bool a_neg = (bool)a_cache.read()[NUCLEUS_DATA_WIDTH - 1];
    bool b_neg = (bool)b_cache.read()[NUCLEUS_DATA_WIDTH - 1];

    negative_result = false;
    abs_a = 0x0;
    abs_b = 0x0;

    switch(funct3_in.read())
    {
        case FUNCT3_XOR_DIV:
            // signed / signed: negative, when numbers have different sign
            negative_result = (a_neg != b_neg);
            abs_a = abs((sc_int<NUCLEUS_DATA_WIDTH>)a_cache.read());
            abs_b = abs((sc_int<NUCLEUS_DATA_WIDTH>)b_cache.read());
            break;
        case FUNCT3_SRL_SRA_DIVU:
            // unsigned / unsigned: always positive
        case FUNCT3_AND_REMU:
            // unsigned % unsigned: always positive
            abs_a = a_cache.read();
            abs_b = b_cache.read();
            break;
        case FUNCT3_OR_REM:
            // sign of divisor ( divisor % dividend == a % b )
            // always take absolute value of dividend (b)
            negative_result = a_neg;
            abs_a = abs((sc_int<NUCLEUS_DATA_WIDTH>)a_cache.read());
            abs_b = abs((sc_int<NUCLEUS_DATA_WIDTH>)b_cache.read());
            break;
    }
}

void m_shift_and_subtract::proc_clk_shift_and_subtract()
{
    // Reset values
    quotient = 0x0;
    remainder = 0x0;
    digit = 0x0;
    state = S_DIV_IDLE;
    a_cache = 0x0;
    b_cache = 0x0;

    while(true)
    {
        wait();

        // Default values
        sc_uint<2> new_state = state.read();
        sc_uint<NUCLEUS_DATA_WIDTH> new_quotient = quotient.read();
        sc_uint<NUCLEUS_DATA_WIDTH> new_remainder = remainder.read();
        sc_uint<NUCLEUS_DATA_WIDTH_LOG2> new_digit = digit.read();
        sc_uint<NUCLEUS_DATA_WIDTH> new_a_cache = a_cache.read();
        sc_uint<NUCLEUS_DATA_WIDTH> new_b_cache = b_cache.read();

        switch(state.read())
        {
            case S_DIV_IDLE:
                new_digit = (sc_uint<NUCLEUS_DATA_WIDTH>)(NUCLEUS_DATA_WIDTH - 1);
                if(start_in.read() == 0x1)
                {
                    if(b_in.read() == 0)
                    {
                        new_state = S_DIV_DONE_EDGE_CASE;
                        // If signed division by zero, the quotient is -1.
                        // If unsigned division by zero, the quotient is
                        // 2^NUCLEUS_DATA_WIDTH - 1.
                        // Written in bits, these are the same numbers, so we
                        // just cast -1 into the unsigned type quotient variable
                        new_quotient = (sc_uint<NUCLEUS_DATA_WIDTH>)-1;
                        // If division by zero, the remainder is a_in.
                        new_remainder = (sc_uint<NUCLEUS_DATA_WIDTH>)a_in.read();
                    }
                    else if(
                        (funct3_in.read() == FUNCT3_XOR_DIV || funct3_in.read() == FUNCT3_OR_REM)
                        && a_in.read() == (1U << (NUCLEUS_DATA_WIDTH - 1)) && b_in.read() == (sc_uint<NUCLEUS_DATA_WIDTH>)(-1))
                    {
                        new_state = S_DIV_DONE_EDGE_CASE;
                        // If signed division overflow (i.e. only a_in is
                        // -2^(NUCLEUS_DATA_WIDTH - 1) and b_in is -1), the quotient
                        // is -2^(NUCLEUS_DATA_WIDTH - 1).
                        // In C code this number is equivalent to:
                        // 1 << (NUCLEUS_DATA_WIDTH - 1)
                        new_quotient = (sc_uint<NUCLEUS_DATA_WIDTH>)(1 << (NUCLEUS_DATA_WIDTH - 1));
                    }
                    else
                    {
                        new_state = S_DIV_WORKING;
                        new_quotient = 0x0;
                        new_remainder = 0x0;
                    }
                }
                new_a_cache = a_in.read();
                new_b_cache = b_in.read();
                break;

            case S_DIV_WORKING:
                // actual division
                new_digit = digit.read() - 1;
                new_remainder = (remainder.read() << 1) | abs_a.read()[digit.read()];
                new_quotient = quotient.read() << 1; // prepare for next quotient bit
                if(new_remainder >= abs_b.read()) // if remainder >= divisor
                {
                    new_remainder = new_remainder - abs_b.read();
                    new_quotient = new_quotient | 1;
                }
                if(digit.read() == 0)
                {
                    new_state = S_DIV_DONE_REGULAR;
                }
                break;

            case S_DIV_DONE_REGULAR:
            case S_DIV_DONE_EDGE_CASE:
                new_state = S_DIV_IDLE;
                break;
        }
        quotient = new_quotient;
        remainder = new_remainder;
        state = new_state;
        digit = new_digit;
        a_cache = new_a_cache;
        b_cache = new_b_cache;
    }
}

void m_shift_and_subtract::proc_cmb_output()
{
    valid_out = 0x0;
    y_out = 0x0;

    switch(state.read())
    {
        case S_DIV_DONE_REGULAR:
            switch(funct3_in.read())
            {
                case FUNCT3_XOR_DIV:
                    if(negative_result.read())
                    {
                        y_out = 0 - quotient.read();
                    } else {
                        y_out = quotient.read();
                    }
                    break;

                case FUNCT3_SRL_SRA_DIVU:
                    y_out = quotient.read();
                    break;

                case FUNCT3_OR_REM:
                    if(negative_result.read())
                    {
                        y_out = 0 - remainder.read();
                    } else {
                        y_out = remainder.read();
                    }
                    break;

                case FUNCT3_AND_REMU:
                    y_out = remainder.read();
                    break;

                default:
                    PN_ERRORF(("Invalid Funct3 for division: 0x%x", funct3_in.read()));
                    break;
            }
            valid_out = 0x1;
            break;

        case S_DIV_DONE_EDGE_CASE:
            switch(funct3_in.read())
            {
                case FUNCT3_XOR_DIV:
                case FUNCT3_SRL_SRA_DIVU:
                    y_out = quotient.read();
                    break;

                case FUNCT3_OR_REM:
                case FUNCT3_AND_REMU:
                    y_out = remainder.read();
                    break;

                default:
                    PN_ERRORF(("Invalid Funct3 for division: 0x%x", funct3_in.read()));
                    break;
            }
            valid_out = 0x1;
            break;

    }
}
