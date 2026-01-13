/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Beaurel Ngaleu <beaurel.ingride.ngaleu@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    This file contains a SystemC implementation of a line and column counter
    used in a VGA or video timing pipeline. It generates the current pixel 
    coordinates (vid_column, vid_line) based on configurable end values and 
    signals when the coordinates are within the visible display area.

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
 * activation signal (`vid_enable`).
 *
 * When `enable` is deactivated, the counters pause.
 * On reset, both counters are set to 0.
 * When `column >= vid_column_end`, `column` resets to 0 and `line` increments.
 * Similarly, when `line >= vid_line_end`, it wraps back to 0.
 *
 * The `vid_enable` signal is active (1) when the current position is
 * within the visible video area:
 * `(column <= vid_column_active) && (line <= vid_line_active)`
 *
 * @par Ports:
 * @param[in] clk Clock signal
 * @param[in] reset Asynchronous reset
 * @param[in] enable Enables the counters
 *
 * @param[in] vid_column_end Maximum column count (inclusive)
 * @param[in] vid_line_end Maximum line count (inclusive)
 * @param[in] vid_column_active Number of visible columns
 * @param[in] vid_line_active Number of visible lines
 *
 * @param[out] vid_column Current column position
 * @param[out] vid_line Current line position
 * @param[out] vid_enable Active signal, asserted when within the visible area
 */



#ifndef LINE_COLUMN_COUNTER_H
#define LINE_COLUMN_COUNTER_H

#include <systemc.h>
#include <piconut.h>

SC_MODULE(m_line_column_counter) {
public:
    // Inputs
    sc_in<bool> clk;
    sc_in<bool> reset;
    sc_in<bool> enable;

    sc_in<sc_uint<16>> vid_column_end;
    sc_in<sc_uint<16>> vid_line_end;
    sc_in<sc_uint<16>> vid_column_active;
    sc_in<sc_uint<16>> vid_line_active;

    // Outputs
    sc_out<sc_uint<16>> vid_column;
    sc_out<sc_uint<16>> vid_line;
    sc_out<bool> vid_enable;

    void counter_process();

    SC_CTOR(m_line_column_counter) {
        SC_METHOD(counter_process);
        sensitive << clk.pos();
        column = 0;
        line = 0;
    }
protected:
    // Internal registers
    sc_uint<16> column;
    sc_uint<16> line;
};

#endif // LINE_COLUMN_COUNTER_H
