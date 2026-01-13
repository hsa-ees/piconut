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

#include "demo_image_source.h"

void m_demo_image_source::pn_trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, icon_x);
    PN_TRACE(tf, icon_y);
    PN_TRACE(tf, icon_x_dir);
    PN_TRACE(tf, icon_y_dir);
    PN_TRACE(tf, color_offset);
}

void m_demo_image_source::proc_clk_pixel()
{
    vid_output = 0;

    while(1)
    {
        wait();
        if(vid_enable.read())
        {
            // Generate background
            if(vid_column.read().bit(6) ^ vid_line.read().bit(6))
            {
                vid_output = COLORS[(color_offset.read() + vid_line.read().range(8, 6)) & 7];
            }
            else
            {
                vid_output = 0x000000;
            }

            // Generate moving sprite
            // Position (x: 0, y: 0) is the top-left corner of the sprite
            sc_uint<16> x = vid_column.read() - icon_x.read();
            sc_uint<16> y = vid_line.read() - icon_y.read();

            // Check upper bits to only show the sprite once
            if(x.range(15, 7) == 0 && y.range(15, 7) == 0)
            {
                // Lookup color index in sprite
                // x and y are each shifted by 2 bits to upscale by a factor
                auto color_index = PICONUT_SPRITE[y.range(6, 2)][x.range(6, 2)];

                // Lookup color
                switch(color_index)
                {
                    case 0:
                        break;
                    case 1:
                        vid_output = 0x1B313A;
                        break;
                    case 2:
                        vid_output = 0xC8AB8B;
                        break;
                    case 3:
                        vid_output = 0x81442B;
                        break;
                    case 4:
                        vid_output = 0x000000;
                        break;
                }
            }
        }
        else
        {
            vid_output = 0;
        }
    }
}

void m_demo_image_source::proc_clk_frame_update()
{
    // Reset values
    icon_x = 266;
    icon_y = 184;
    icon_x_dir = 0;
    icon_y_dir = 0;
    color_offset = 0;

    while(1)
    {
        wait();
        // Update icon position
        if(vid_line.read() == 0 && vid_column.read() == 0)
        {
            // Update x direction when the icon hits an vertical edge
            if(icon_x.read() == 0 + 1 && icon_x_dir.read() == 1)
            {
                icon_x_dir = 0;
                color_offset = (color_offset.read() + 1) & 7;
            }
            else if(icon_x.read() == 531 - 1 && icon_x_dir.read() == 0)
            {
                icon_x_dir = 1;
                color_offset = (color_offset.read() + 1) & 7;
            }

            // Update y direction when the icon hits an horizontal edge
            if(icon_y.read() == 0 + 1 && icon_y_dir.read() == 1)
            {
                icon_y_dir = 0;
                color_offset = (color_offset.read() + 1) & 7;
            }
            else if(icon_y.read() == 367 - 1 && icon_y_dir.read() == 0)
            {
                icon_y_dir = 1;
                color_offset = (color_offset.read() + 1) & 7;
            }

            // Advance x position according to direction bit
            if(icon_x_dir.read() == 0)
            {
                icon_x = icon_x.read() + 1;
            }
            else
            {
                icon_x = icon_x.read() - 1;
            }

            // Advance y position according to direction bit
            if(icon_y_dir.read() == 0)
            {
                icon_y = icon_y.read() + 1;
            }
            else
            {
                icon_y = icon_y.read() - 1;
            }
        }
    }
}
