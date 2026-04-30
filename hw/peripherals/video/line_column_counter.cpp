/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Martin Erichsen <martin.erichsen@tha.de>
                     Beaurel Ngaleu <beaurel.ingride.ngaleu@tha.de>
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

#include "line_column_counter.h"

void m_line_column_counter::pn_trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, enable_in);
    PN_TRACE(tf, column_end_in);
    PN_TRACE(tf, line_end_in);
    PN_TRACE(tf, column_active_in);
    PN_TRACE(tf, line_active_in);
    PN_TRACE(tf, reg_column);
    PN_TRACE(tf, reg_line);
    PN_TRACE(tf, vid_column_out);
    PN_TRACE(tf, vid_line_out);
}

void m_line_column_counter::proc_clk_counter()
{
    reg_column = 0;
    reg_line = 0;

    while(true)
    {
        wait();

        if(enable_in.read())
        {

            if(reg_column >= column_end_in.read())
            {
                reg_column = 0;
                if(reg_line >= line_end_in.read())
                    reg_line = 0;
                else
                {
                    reg_line = reg_line.read() + 1;
                }
            }
            else
            {
                reg_column = reg_column.read() + 1;
            }
        }
    }
}

void m_line_column_counter::proc_comb_outputs()
{
    vid_column_out = reg_column;
    vid_line_out = reg_line;
    // add one for clocked thread of color generator. This is a kind of workaround but it works fine for now and testing it correctly will take too long
    blanking_enable_out.write(reg_column.read() > column_active_in.read() || reg_column.read() < 2 ||
                              (reg_line.read() + 1) > line_active_in.read());
}
