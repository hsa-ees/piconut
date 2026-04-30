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
    PN_TRACE(tf, resolution_mode_in);

    PN_TRACE(tf, column_active_out);
    PN_TRACE(tf, column_end_out);
    PN_TRACE(tf, line_active_out);
    PN_TRACE(tf, line_end_out);

    PN_TRACE(tf, vga_hsync_begin_out);
    PN_TRACE(tf, vga_hsync_end_out);
    PN_TRACE(tf, vga_vsync_begin_out);
    PN_TRACE(tf, vga_vsync_end_out);
}

void m_vga_timings::proc_comb_timings()
{
    uint16_t horizontal_visible_area = 0;
    uint16_t horizontal_front_porch = 0;
    uint16_t horizontal_sync_pulse = 0;
    uint16_t horizontal_back_porch = 0;

    uint16_t vertical_visible_area = 0;
    uint16_t vertical_front_porch = 0;
    uint16_t vertical_sync_pulse = 0;
    uint16_t vertical_back_porch = 0;

    switch(resolution_mode_in.read())
    {
        case RESOLUTION_MODE_640x480:
            // 640x480 at 25 MHz pixel clock
            horizontal_visible_area = 640;
            horizontal_front_porch = 16;
            horizontal_sync_pulse = 96;
            horizontal_back_porch = 48;

            vertical_visible_area = 480;
            vertical_front_porch = 10;
            vertical_sync_pulse = 2;
            vertical_back_porch = 32;
            break;
    }

    column_active_out = horizontal_visible_area;
    column_end_out = horizontal_visible_area +
                     horizontal_front_porch +
                     horizontal_sync_pulse +
                     horizontal_back_porch;
    vga_hsync_begin_out = horizontal_visible_area + horizontal_front_porch;
    vga_hsync_end_out = horizontal_visible_area + horizontal_front_porch + horizontal_sync_pulse;

    line_active_out = vertical_visible_area;
    line_end_out = vertical_visible_area +
                   vertical_front_porch +
                   vertical_sync_pulse +
                   vertical_back_porch;
    vga_vsync_begin_out = vertical_visible_area + vertical_front_porch;
    vga_vsync_end_out = vertical_visible_area + vertical_front_porch + vertical_sync_pulse;
}
