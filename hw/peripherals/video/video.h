/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Niklas Sirch <niklas.sirch1@tha.de>
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
 * @fn SC_MODULE(m_video)
 * @brief
 * This module connects several subsystems such as Wishbone control, framebuffer,
 * VGA sync generator, color generator, and timing logic into a complete video system.
 *
 * @par Ports:
 * @param[in]  clk               Main clock signal
 * @param[in]  reset                 Global reset signal
 * @param[out] vga_red_out               4-bit VGA red color output
 * @param[out] vga_green_out             4-bit VGA green color output
 * @param[out] vga_blue_out              4-bit VGA blue color output
 * @param[out] vga_hsync_out             VGA horizontal sync output
 * @param[out] vga_vsync_out             VGA vertical sync output
 */

#ifndef __WB_GRAPHICS_H__
#define __WB_GRAPHICS_H__

#include <systemc.h>
#include <piconut.h>

#include "video_defs.h"
#include "video_defs_hw.h"

SC_MODULE(m_video), pn_module_if
{
public:
    // Clock & Reset & Enable
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    pn_wishbone_slave_t wb_slave;

    sc_out<sc_uint<4>> PN_NAME(vga_red_out);
    sc_out<sc_uint<4>> PN_NAME(vga_green_out);
    sc_out<sc_uint<4>> PN_NAME(vga_blue_out);
    sc_out<bool> PN_NAME(vga_hsync_out);
    sc_out<bool> PN_NAME(vga_vsync_out);

    SC_HAS_PROCESS(m_video);
    m_video(
        sc_module_name name,
        pn_wb_adr_t base_address)
        : sc_module(name)
        , wb_slave{
              .alen = 32,
              .dlen = 32,
              .base_address = base_address,
              .size = PN_VIDEO_REG_SIZE_BYTES}
    {
        pn_add_wishbone_slave(&wb_slave);

        init_submodules();
    }

    void pn_trace(sc_trace_file * tf, int level);

    void proc_cmb_video();

protected:
    void init_submodules();
    // Wishbone
    class m_video_wb* video_wb_inst;
    // Submodules
    class m_framebuffer_source* framebuffer_source_inst;
    class m_color_translator* color_translator_inst;
    class m_vga_color_generator* color_generator_inst;
    class m_vga_sync_generator* sync_generator_inst;
    class m_vga_timings* timings_inst;
    class m_line_column_counter* line_column_counter_inst;

    /**
     * @brief Video enable signal to activate video output generation
     */
    sc_signal<bool> PN_NAME(s_enable_video);
    /**
     * @brief Interrupt enable signal to activate video interrupts
     * Unimplemented yet
     */
    sc_signal<bool> PN_NAME(s_enable_interrupts);

    /// Internal Signals
    // Registers
    sc_signal<sc_uint<5>> PN_NAME(s_color_mode);
    sc_signal<sc_uint<5>> PN_NAME(s_resolution_mode);

    // Framebuffer Control Signals
    sc_signal<sc_uint<FB_RAM_ADDR_WIDTH>> PN_NAME(s_ctl_fb_addr);
    sc_signal<sc_uint<FB_RAM_DATA_WIDTH>> PN_NAME(s_ctl_data_in);
    sc_signal<sc_uint<FB_RAM_DATA_WIDTH>> PN_NAME(s_ctl_data_out);
    sc_signal<bool> PN_NAME(s_ctl_fb_write_en);

    sc_signal<sc_uint<32>> PN_NAME(s_fb_output);
    sc_signal<sc_uint<H_COUNTER_WIDTH>> PN_NAME(vid_column);
    sc_signal<sc_uint<V_COUNTER_WIDTH>> PN_NAME(vid_line);

    // Timing Signals
    sc_signal<sc_uint<V_COUNTER_WIDTH>> PN_NAME(s_column_timing_active);
    sc_signal<sc_uint<V_COUNTER_WIDTH>> PN_NAME(s_column_timing_end);
    sc_signal<sc_uint<H_COUNTER_WIDTH>> PN_NAME(s_line_timing_active);
    sc_signal<sc_uint<H_COUNTER_WIDTH>> PN_NAME(s_line_timing_end);
    sc_signal<sc_uint<H_COUNTER_WIDTH>> PN_NAME(s_vga_hsync_timing_begin);
    sc_signal<sc_uint<H_COUNTER_WIDTH>> PN_NAME(s_vga_hsync_timing_end);
    sc_signal<sc_uint<V_COUNTER_WIDTH>> PN_NAME(s_vga_vsync_timing_begin);
    sc_signal<sc_uint<V_COUNTER_WIDTH>> PN_NAME(s_vga_vsync_timing_end);

    // Blanking Signal
    sc_signal<bool> PN_NAME(s_blanking_enable);

    // Intermediate Color Signal
    sc_signal<sc_uint<24>> PN_NAME(s_color_trans_output);
};

#endif // __WB_GRAPHICS_H__
