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

#include "vga_sync_generator.h"

void m_vga_sync_generator::pn_trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, vga_hsync);
    PN_TRACE(tf, vga_vsync);
}

void m_vga_sync_generator::proc_comb_sync()
{
    if(enable.read())
    {
        vga_hsync_next = vga_hsync.read();
        vga_vsync_next = vga_vsync.read();

        if(column.read() == vga_hsync_begin.read())
        {
            vga_hsync_next = 0;
        }
        else if(column.read() == vga_hsync_end.read())
        {
            vga_hsync_next = 1;
        }

        if(line.read() == vga_vsync_begin.read())
        {
            vga_vsync_next = 0;
        }
        else if(line.read() == vga_vsync_end.read())
        {
            vga_vsync_next = 1;
        }
    }
    else
    {
        vga_hsync_next = 1;
        vga_vsync_next = 1;
    }
}

void m_vga_sync_generator::proc_clk()
{
    vga_hsync = 1;
    vga_vsync = 1;

    while(true)
    {
        wait();
        vga_hsync = vga_hsync_next.read();
        vga_vsync = vga_vsync_next.read();
    }
}
