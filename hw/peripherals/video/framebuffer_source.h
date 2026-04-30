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
 * @fn SC_MODULE(m_framebuffer_source)
 * @author Martin Erichsen
 * @brief Video source module with internal framebuffer
 *
 * The framebuffer source generates a video signal from an internal framebuffer.
 * It offers two separate interfaces for interaction with the system and video
 * output, each driven by their own clocks.
 *
 * The system-side port offers a read/write interface to the internal
 * framebuffer.
 *
 * From the video-side port, `vid_column_in` and `vid_line` are used to address
 * pixels inside the buffer. Calculation of the buffer address is given by
 * following equation:
 * ```
 * address = (column / PN_CFG_VIDEO_FB_SCALING_DIVISOR) + (line / PN_CFG_VIDEO_FB_SCALING_DIVISOR) * (RESOLUTION_X / PN_CFG_VIDEO_FB_SCALING_DIVISOR)
 * ```
 * This results in a fixed upscaling factor of 16 and a maximum column address
 * of 639. Column addresses greater than 639 wrap over to the next line.
 * While `vid_enable_in` is disabled, zero will be output over `vid_output`.
 *
 * Generation of the video signal requires one clock cycle.
 *
 * @par Ports:
 * @param[in] sys_clk       system-side clock input
 * @param[in] reset         system-side reset
 *
 * @param[in] ctl_addr_in      system-side buffer address
 * @param[in] ctl_data_in      system-side buffer write data
 * @param[in] ctl_write_en_in  system-side buffer write enable
 *
 * @param[in] vid_clk        video-side clock input
 * @param[in] vid_enable_in  video-side output enable
 * @param[in] vid_column_in  video-side horizontal address
 * @param[in] vid_line_in    video-side vertical address
 * @param[out] vid_data_out  video-side video output
 */

#ifndef __FRAMEBUFFER_SOURCE_H__
#define __FRAMEBUFFER_SOURCE_H__

#include <piconut.h>

#include "framebuffer_ram.h"


SC_MODULE(m_framebuffer_source)
{
public:
    sc_in<bool> PN_NAME(reset);

    // System Control Interface
    sc_in_clk PN_NAME(sys_clk);
    sc_in<sc_uint<FB_RAM_ADDR_WIDTH>> PN_NAME(ctl_addr_in);
    sc_in<sc_uint<FB_RAM_DATA_WIDTH>> PN_NAME(ctl_data_in);
    sc_in<bool> PN_NAME(ctl_write_en_in);

    // Ctl Output
    sc_out<sc_uint<FB_RAM_DATA_WIDTH>> PN_NAME(ctl_data_out);

    // Video Output Interface
    sc_in_clk PN_NAME(vid_clk);
    sc_in<bool> PN_NAME(vid_enable_in);
    sc_in<sc_uint<H_COUNTER_WIDTH>> PN_NAME(vid_column_in);
    sc_in<sc_uint<V_COUNTER_WIDTH>> PN_NAME(vid_line_in);
    sc_out<sc_uint<32>> PN_NAME(vid_data_out);

    // Constructor
    SC_CTOR(m_framebuffer_source)
    {
        init_submodules();

        SC_METHOD(proc_comb_ctl);
        sensitive << reset;

        SC_METHOD(proc_comb_vid);
        sensitive << vid_enable_in << ram_dob
                  << vid_column_in << vid_line_in;
    }

    // Functions
    void pn_trace(sc_trace_file * tf, int level = 1);

    // Processes
    void proc_comb_ctl();
    void proc_comb_vid();

    // Submodules
    m_framebuffer_ram* fb_ram;

protected:
    void init_submodules();
    // RAM Port A for system control
    sc_signal<bool> PN_NAME(ram_ena); // enable bram for port A

    // RAM Port B for video output
    sc_signal<bool> PN_NAME(ram_web);                         // write enable for video port
    sc_signal<sc_uint<FB_RAM_ADDR_WIDTH>> PN_NAME(ram_addrb); // address for video port
    sc_signal<sc_uint<FB_RAM_DATA_WIDTH>> PN_NAME(ram_dib);   // data in for video port
    sc_signal<sc_uint<FB_RAM_DATA_WIDTH>> PN_NAME(ram_dob);   // data out for video port
};

#endif // __FRAMEBUFFER_SOURCE_H__
