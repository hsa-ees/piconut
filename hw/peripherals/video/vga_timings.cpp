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

#include "vga_timings.h"

void m_vga_timings::pn_trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, resolution_mode);

    PN_TRACE(tf, vid_column_active);
    PN_TRACE(tf, vid_column_end);
    PN_TRACE(tf, vid_line_active);
    PN_TRACE(tf, vid_line_end);

    PN_TRACE(tf, vga_hsync_begin);
    PN_TRACE(tf, vga_hsync_end);
    PN_TRACE(tf, vga_vsync_begin);
    PN_TRACE(tf, vga_vsync_end);
}

void m_vga_timings::proc_comb_timings()
{
    vid_column_active = 0;
    vid_column_end = 0;
    vid_line_active = 0;
    vid_line_end = 0;

    vga_hsync_begin = 0;
    vga_hsync_end = 0;
    vga_vsync_begin = 0;
    vga_vsync_end = 0;

    switch(resolution_mode.read())
    {
        case RESOLUTION_MODE_640x480:
            // 640x480 at 25 MHz pixel clock
            vid_column_active = 640;
            vid_column_end = 800;
            vid_line_active = 480;
            vid_line_end = 525;

            vga_hsync_begin = 640 + 23;
            vga_hsync_end = 640 + 23 + 96;
            vga_vsync_begin = 480 + 10;
            vga_vsync_end = 480 + 10 + 2;
            break;
    }
}
