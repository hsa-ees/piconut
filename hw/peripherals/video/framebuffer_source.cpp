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

#include "framebuffer_source.h"

void m_framebuffer_source::pn_trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, vid_enable_in);
    PN_TRACE(tf, ram_ena);
    PN_TRACE(tf, ctl_data_out);
    PN_TRACE(tf, ram_web);
    PN_TRACE(tf, ram_addrb);
    PN_TRACE(tf, ram_dib);
    PN_TRACE(tf, ram_dob);
    if(level >= 2)
    {
        fb_ram->pn_trace(tf, level);
    }
}

void m_framebuffer_source::init_submodules()
{
    // RAM Port A for system control
    fb_ram = sc_new<m_framebuffer_ram>("framebuffer_ram");
    fb_ram->clka(sys_clk);
    fb_ram->wea(ctl_write_en_in);
    fb_ram->ena(ram_ena);
    fb_ram->addra(ctl_addr_in);
    fb_ram->dia(ctl_data_in);
    fb_ram->doa(ctl_data_out);

    // RAM Port B for video output
    fb_ram->clkb(vid_clk);
    fb_ram->web(ram_web);
    fb_ram->enb(vid_enable_in);
    fb_ram->addrb(ram_addrb);
    fb_ram->dib(ram_dib);
    fb_ram->dob(ram_dob);
}

void m_framebuffer_source::proc_comb_ctl()
{
    ram_ena = 1;
}

void m_framebuffer_source::proc_comb_vid()
{

    ram_web = 0;
    ram_dib = 0;
    ram_addrb = 0;

    if(vid_enable_in.read())
    {
        vid_data_out = ram_dob.read();

        sc_uint<16> column = vid_column_in.read() / ((uint32_t)PN_CFG_VIDEO_FB_SCALING_DIVISOR);
        sc_uint<16> line = vid_line_in.read() / ((uint32_t)PN_CFG_VIDEO_FB_SCALING_DIVISOR);
        ram_addrb = column + line * (640 / ((uint32_t)PN_CFG_VIDEO_FB_SCALING_DIVISOR));
        // with divide and multiply actually 10 LUTs less than with shift and add :)
    }
    else
    {
        // Disable output while video is disabled
        vid_data_out = 0;
    }
}
