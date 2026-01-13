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

#include "color_translator.h"

void m_color_translator::pn_trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, color_mode);
    PN_TRACE(tf, color_in);
    PN_TRACE(tf, color_out);
}

void m_color_translator::proc_comb_translate()
{
    sc_uint<32> in = color_in.read();
    sc_uint<8> temp = 0;

    color_out_next = 0;

    switch(color_mode.read())
    {
        case COLOR_MODE_1_MONO:
            color_out_next = (in.bit(0) == 1) ? 0xFFFFFF : 0x000000;
            break;

        case COLOR_MODE_2_GRAY:
            color_out_next = ((in.range(1, 0) << 22) |
                              (in.range(1, 0) << 20) |
                              (in.range(1, 0) << 18) |
                              (in.range(1, 0) << 16) |
                              (in.range(1, 0) << 14) |
                              (in.range(1, 0) << 12) |
                              (in.range(1, 0) << 10) |
                              (in.range(1, 0) << 8) |
                              (in.range(1, 0) << 6) |
                              (in.range(1, 0) << 4) |
                              (in.range(1, 0) << 2) |
                              (in.range(1, 0) << 0));
            break;

        case COLOR_MODE_3_GRAY:
            temp = ((in.range(2, 0) << 5) |
                    (in.range(2, 0) << 2) |
                    (in.range(2, 0) >> 1));
            color_out_next = ((temp << 16) |
                              (temp << 8) |
                              (temp << 0));
            break;

        case COLOR_MODE_4_GRAY:
            temp = ((in.range(3, 0) << 4) |
                    (in.range(3, 0) << 0));
            color_out_next = ((temp << 16) |
                              (temp << 8) |
                              (temp << 0));
            break;

        case COLOR_MODE_4_RGBI:
            color_out_next = ((in.bit(3) == 1 ? 0x555555 : 0x000000) |
                              (in.bit(2) == 1 ? 0xAA0000 : 0x000000) |
                              (in.bit(1) == 1 ? 0x00AA00 : 0x000000) |
                              (in.bit(0) == 1 ? 0x0000AA : 0x000000));
            break;

        case COLOR_MODE_8_GRAY:
            temp = in.range(7, 0);
            color_out_next = ((temp << 16) |
                              (temp << 8) |
                              (temp << 0));
            break;

        case COLOR_MODE_3_RGB:
            color_out_next = ((in.bit(2) == 1 ? 0xFF0000 : 0x000000) |
                              (in.bit(1) == 1 ? 0x00FF00 : 0x000000) |
                              (in.bit(0) == 1 ? 0x0000FF : 0x000000));
            break;

        case COLOR_MODE_8_RGB332:
            color_out_next = (
                // Red
                (in.range(7, 5) << 21) |
                (in.range(7, 5) << 18) |
                (in.range(7, 6) << 16) |
                // Green
                (in.range(4, 2) << 13) |
                (in.range(4, 2) << 10) |
                (in.range(4, 3) << 8) |
                // Blue
                (in.range(1, 0) << 6) |
                (in.range(1, 0) << 4) |
                (in.range(1, 0) << 2) |
                (in.range(1, 0) << 0));
            break;

        case COLOR_MODE_16_RGB565:
            color_out_next = ((in.range(15, 11) << 19) |
                              (in.range(15, 13) << 16) |
                              (in.range(10, 5) << 10) |
                              (in.range(10, 9) << 8) |
                              (in.range(4, 0) << 3) |
                              (in.range(4, 2) << 0));
            break;

        case COLOR_MODE_32_RGB:
            color_out_next = in.range(23, 0);
            break;
    }
}

void m_color_translator::proc_clk_update()
{
    while(true)
    {
        wait();
        color_out = color_out_next.read();
    }
}
