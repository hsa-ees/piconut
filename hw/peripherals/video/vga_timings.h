/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Martin Erichsen <martin.erichsen@tha.de>
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
 * @fn SC_MODULE(m_vga_timings)
 * @author Martin Erichsen
 * @brief Video timing configuration lookup table
 *
 * Outputs a timing configuration selected by `resolution_mode`. Currently, only
 * a resolution of 640x480 at pixel clock of 25 MHz is supported.
 *
 * This module has no clock, lookup is done combinatorically.
 *
 * @par Ports:
 * @param[in] resolution_mode    active resolution mode
 * @param[out] vid_column_active number of active pixels on a video line
 * @param[out] vid_column_end    total number of pixels on a video line
 * @param[out] vid_line_active   number of active lines in a video frame
 * @param[out] vid_line_end      total number of pixels in a video frame
 * @param[out] vga_hsync_begin   horizontal address to assert hsync after
 * @param[out] vga_hsync_end     horizontal address to deassert hsync after
 * @param[out] vga_vsync_begin   vertical address to assert vsync after
 * @param[out] vga_vsync_end     vertical address to deassert vsync after
 *
 */

#ifndef __VGA_TIMINGS_H__
#define __VGA_TIMINGS_H__

#include "wb_graphics_config.h"

#include <systemc.h>
#include <piconut.h>

SC_MODULE(m_vga_timings)
{
public:
    sc_in<sc_uint<5>> PN_NAME(resolution_mode);

    // Video timing signals for the counter
    sc_out<sc_uint<H_COUNTER_WIDTH>> PN_NAME(vid_column_active);
    sc_out<sc_uint<H_COUNTER_WIDTH>> PN_NAME(vid_column_end);
    sc_out<sc_uint<V_COUNTER_WIDTH>> PN_NAME(vid_line_active);
    sc_out<sc_uint<V_COUNTER_WIDTH>> PN_NAME(vid_line_end);

    // Horizontal and vertical synchronization pulse timings
    sc_out<sc_uint<H_COUNTER_WIDTH>> PN_NAME(vga_hsync_begin);
    sc_out<sc_uint<H_COUNTER_WIDTH>> PN_NAME(vga_hsync_end);
    sc_out<sc_uint<V_COUNTER_WIDTH>> PN_NAME(vga_vsync_begin);
    sc_out<sc_uint<V_COUNTER_WIDTH>> PN_NAME(vga_vsync_end);

    SC_CTOR(m_vga_timings)
    {
        SC_METHOD(proc_comb_timings);
        sensitive << resolution_mode;
    }

    // Functions
    void pn_trace(sc_trace_file * tf, int level = 1);

    // Processes
    void proc_comb_timings();
};

#endif //__VGA_TIMINGS_H__
