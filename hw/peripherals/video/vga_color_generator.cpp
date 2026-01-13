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

#include "vga_color_generator.h"

void m_vga_color_generator::pn_trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, video_in);
    PN_TRACE(tf, blank_enable);
    PN_TRACE(tf, vga_red);
    PN_TRACE(tf, vga_green);
    PN_TRACE(tf, vga_blue);
}

void m_vga_color_generator::proc_comb_convert()
{
    if(blank_enable.read())
    {
        vga_red_next = 0;
        vga_green_next = 0;
        vga_blue_next = 0;
    }
    else
    {
        vga_red_next = video_in.read().range(23, 20);
        vga_green_next = video_in.read().range(15, 12);
        vga_blue_next = video_in.read().range(7, 4);
    }
}

void m_vga_color_generator::proc_clk_update()
{
    vga_red = 0;
    vga_green = 0;
    vga_blue = 0;

    while(true)
    {
        wait();
        vga_red = vga_red_next.read();
        vga_green = vga_green_next.read();
        vga_blue = vga_blue_next.read();
    }
}
