/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Beaurel Ngaleu <beaurel.ingride.ngaleu@tha.de>
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

/**
 * @fn SC_MODULE(m_line_column_counter)
 * @authors Beaurel I. Ngaleu
 * @brief
 * This module implements two synchronized counters (column and line)
 * used for generating a video signal. The counters increment according to
 * a clock signal and output the current image position as well as an
 * activation signal (`vid_enable_out`).
 *
 * When `enable` is deactivated, the counters pause.
 * On reset, both counters are set to 0.
 * When `column >= column_end`, `column` resets to 0 and `line` increments.
 * Similarly, when `line >= line_end`, it wraps back to 0.
 *
 * The `vid_enable_out` signal is active (1) when the current position is
 * within the visible video area:
 * `(column <= column_active) && (line <= line_active)`
 *
 * @par Ports:
 * @param[in] clk Clock signal
 * @param[in] reset Asynchronous reset
 * @param[in] enable_in Enables the counters
 *
 * @param[in] column_end_in Maximum column count (inclusive)
 * @param[in] line_end_in Maximum line count (inclusive)
 * @param[in] column_active_in Number of visible columns
 * @param[in] line_active_in Number of visible lines
 *
 * @param[out] vid_column_out Current column position
 * @param[out] vid_line_out Current line position
 * @param[out] blanking_enable_out Active signal, asserted when within the visible area
 */

#ifndef LINE_COLUMN_COUNTER_H
#define LINE_COLUMN_COUNTER_H

#include <systemc.h>
#include <piconut.h>
#include "video_defs.h"
#include "video_defs_hw.h"

SC_MODULE(m_line_column_counter)
{
public:
    // Inputs
    sc_in<bool> clk;
    sc_in<bool> reset;
    sc_in<bool> enable_in;

    sc_in<sc_uint<H_COUNTER_WIDTH>> column_end_in;
    sc_in<sc_uint<V_COUNTER_WIDTH>> line_end_in;
    sc_in<sc_uint<H_COUNTER_WIDTH>> column_active_in;
    sc_in<sc_uint<V_COUNTER_WIDTH>> line_active_in;

    // Outputs
    sc_out<sc_uint<H_COUNTER_WIDTH>> vid_column_out;
    sc_out<sc_uint<V_COUNTER_WIDTH>> vid_line_out;
    sc_out<bool> blanking_enable_out;

    SC_CTOR(m_line_column_counter)
    {
        SC_CTHREAD(proc_clk_counter, clk.pos());
        reset_signal_is(reset, true);

        SC_METHOD(proc_comb_outputs);
        sensitive << reg_column << reg_line << column_active_in
                  << line_active_in << column_end_in << line_end_in;
    }

    void pn_trace(sc_trace_file * tf, int level);

protected:
    void proc_clk_counter();
    void proc_comb_outputs();

    // Internal registers
    sc_signal<sc_uint<H_COUNTER_WIDTH>> reg_column;
    sc_signal<sc_uint<V_COUNTER_WIDTH>> reg_line;
};

#endif // LINE_COLUMN_COUNTER_H
