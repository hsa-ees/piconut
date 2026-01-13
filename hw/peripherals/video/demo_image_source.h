/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Martin Erichsen <martin.erichsen@tha.de>
                2025 Beaurel Ngaleu <beaurel.ingride.ngaleu@tha.de>
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

#ifndef __DEMO_IMAGE_SOURCE_H__
#define __DEMO_IMAGE_SOURCE_H__

#include "wb_graphics_config.h"
#include <systemc.h>
#include <piconut.h>

/**
 * @fn SC_MODULE(m_demo_image_source)
 * @author Beaurel I Ngaleu
 * @brief Generates a demo image
 *
 * Requires one clock cycle to output.
 *
 * @par Ports:
 * @param[in] clk         clock of the module
 * @param[in] reset       reset of the module
 * @param[in] vid_enable  video output enable
 * @param[in] vid_column  video horizontal address
 * @param[in] vid_line    video vertical address
 * @param[out] vid_output video output
 */

SC_MODULE(m_demo_image_source)
{
public:
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    // Video Output Interface
    sc_in<bool> PN_NAME(vid_enable);
    sc_in<sc_uint<H_COUNTER_WIDTH>> PN_NAME(vid_column);
    sc_in<sc_uint<V_COUNTER_WIDTH>> PN_NAME(vid_line);
    sc_out<sc_uint<32>> PN_NAME(vid_output);

    SC_CTOR(m_demo_image_source)
    {
        SC_CTHREAD(proc_clk_pixel, clk.pos());
        reset_signal_is(reset, true);

        SC_CTHREAD(proc_clk_frame_update, clk.pos());
        reset_signal_is(reset, true);
    }

    // Functions
    void pn_trace(sc_trace_file * tf, int level = 1);

    // Processes
    void proc_clk_frame_update();
    void proc_clk_pixel();

protected:
    sc_signal<sc_uint<16>> PN_NAME(icon_x);
    sc_signal<sc_uint<16>> PN_NAME(icon_y);
    sc_signal<bool> PN_NAME(icon_x_dir);
    sc_signal<bool> PN_NAME(icon_y_dir);

    sc_signal<sc_uint<16>> PN_NAME(color_offset);

    // clang-format off
    const sc_uint<3> PICONUT_SPRITE[32][32] = {
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 1, 4, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 1, 1, 0, 1, 4, 1, 4, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 4, 1, 4, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 1, 4, 1, 1, 1, 1, 1, 4, 4, 4, 1, 1, 1, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 4, 2, 4, 2, 4, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 4, 4, 2, 4, 2, 4, 4, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 1, 1, 1, 1, 4, 2, 4, 2, 4, 2, 4, 2, 4, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 1, 1, 1, 1, 4, 2, 4, 2, 4, 2, 4, 2, 4, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 1, 1, 0, 1, 1, 1, 1, 4, 2, 4, 4, 2, 4, 2, 4, 4, 2, 4, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 1, 1, 1, 4, 2, 4, 2, 2, 4, 2, 2, 4, 2, 4, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 1, 1, 1, 4, 2, 4, 2, 2, 4, 2, 2, 4, 2, 4, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 1, 1, 0, 1, 1, 1, 1, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 1, 1, 1, 4, 3, 4, 3, 3, 4, 3, 3, 4, 3, 4, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 1, 1, 1, 4, 3, 4, 3, 3, 4, 3, 3, 4, 3, 4, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 1, 1, 0, 1, 1, 1, 1, 1, 4, 3, 4, 3, 4, 3, 4, 3, 4, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 4, 4, 3, 4, 3, 4, 4, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 1, 4, 1, 1, 1, 1, 4, 4, 4, 4, 4, 1, 1, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 1, 1, 0, 1, 4, 1, 4, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 4, 1, 4, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 1, 4, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 4, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };

    // Colors for background pattern
    const sc_uint<32> COLORS[8] = {
        0xFFFFFF,
        0xE50000, // Red
        0xFF8D00, // Orange
        0xFFEE00, // Yellow
        0x008121, // Green
        0x004CFF, // Blue
        0x760188, // Purple
        0xFF0350,
    };
    // clang-format on
};

#endif //__DEMO_IMAGE_SOURCE_H__
