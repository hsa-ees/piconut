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

#include "video.h"
#include "framebuffer_source.h"
#include "color_translator.h"
#include "vga_color_generator.h"
#include "vga_sync_generator.h"
#include "vga_timings.h"
#include "video_wb.h"
#include "framebuffer_ram.h"
#include "line_column_counter.h"

void m_video::pn_trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, s_enable_video);
    PN_TRACE(tf, vga_blue_out);
    PN_TRACE(tf, vga_green_out);
    PN_TRACE(tf, vga_red_out);
    PN_TRACE(tf, vga_hsync_out);
    PN_TRACE(tf, vga_vsync_out);
    if(level > 2)
    {
        video_wb_inst->pn_trace(tf, level);
        framebuffer_source_inst->pn_trace(tf, level);
        color_translator_inst->pn_trace(tf, level);
        color_generator_inst->pn_trace(tf, level);
        sync_generator_inst->pn_trace(tf, level);
        timings_inst->pn_trace(tf, level);
        line_column_counter_inst->pn_trace(tf, level);
    }
}

void m_video::init_submodules()
{
    // Wishbone
    video_wb_inst = sc_new<m_video_wb>("video_wb_inst", wb_slave.base_address);
    video_wb_inst->clk(clk); // sys_clk
    video_wb_inst->reset(reset);
    video_wb_inst->wb_slave.stb_i(wb_slave.stb_i);
    video_wb_inst->wb_slave.cyc_i(wb_slave.cyc_i);
    video_wb_inst->wb_slave.we_i(wb_slave.we_i);
    video_wb_inst->wb_slave.adr_i(wb_slave.adr_i);
    video_wb_inst->wb_slave.dat_i(wb_slave.dat_i);
    video_wb_inst->wb_slave.dat_o(wb_slave.dat_o);
    video_wb_inst->wb_slave.sel_i(wb_slave.sel_i);
    video_wb_inst->wb_slave.ack_o(wb_slave.ack_o);
    video_wb_inst->wb_slave.err_o(wb_slave.err_o);
    video_wb_inst->wb_slave.rty_o(wb_slave.rty_o);

    video_wb_inst->fb_ctl_addr_out(s_ctl_fb_addr);
    video_wb_inst->fb_ctl_data_in_out(s_ctl_data_in);
    video_wb_inst->fb_ctl_write_en_out(s_ctl_fb_write_en);
    video_wb_inst->vid_line_in(vid_line);
    video_wb_inst->fb_ctl_data_out_in(s_ctl_data_out);
    video_wb_inst->enable_video_out(s_enable_video);
    video_wb_inst->enable_interrupt_out(s_enable_interrupts);
    video_wb_inst->resolution_mode_out(s_resolution_mode);
    video_wb_inst->color_mode_out(s_color_mode);

    // Framebuffer Source
    framebuffer_source_inst = sc_new<m_framebuffer_source>("framebuffer_source_inst");
    framebuffer_source_inst->sys_clk(clk); // sys_clk
    framebuffer_source_inst->vid_clk(clk); // vid_clk
    framebuffer_source_inst->reset(reset);
    framebuffer_source_inst->vid_enable_in(s_enable_video);

    framebuffer_source_inst->ctl_addr_in(s_ctl_fb_addr);
    framebuffer_source_inst->ctl_data_in(s_ctl_data_in);
    framebuffer_source_inst->ctl_data_out(s_ctl_data_out);
    framebuffer_source_inst->ctl_write_en_in(s_ctl_fb_write_en);
    framebuffer_source_inst->vid_column_in(vid_column);
    framebuffer_source_inst->vid_line_in(vid_line);
    framebuffer_source_inst->vid_data_out(s_fb_output);

    // Color Translator
    color_translator_inst = sc_new<m_color_translator>("color_translator_inst");
    color_translator_inst->clk(clk); // vid_clk
    color_translator_inst->reset(reset);

    color_translator_inst->color_mode_in(s_color_mode);
    color_translator_inst->color_in(s_fb_output);
    color_translator_inst->color_out(s_color_trans_output);

    // VGA Color Generator
    color_generator_inst = sc_new<m_vga_color_generator>("color_generator_inst");
    color_generator_inst->clk(clk); // vid_clk
    color_generator_inst->reset(reset);

    color_generator_inst->video_in(s_color_trans_output);
    color_generator_inst->blank_enable_in(s_blanking_enable);
    color_generator_inst->vga_red_out(vga_red_out);
    color_generator_inst->vga_green_out(vga_green_out);
    color_generator_inst->vga_blue_out(vga_blue_out);

    // Sync Generator
    sync_generator_inst = sc_new<m_vga_sync_generator>("sync_generator_inst");
    sync_generator_inst->clk(clk); // vid_clk
    sync_generator_inst->reset(reset);
    sync_generator_inst->enable(s_enable_video);

    sync_generator_inst->column(vid_column);
    sync_generator_inst->line(vid_line);
    sync_generator_inst->vga_hsync_out(vga_hsync_out);
    sync_generator_inst->vga_vsync_out(vga_vsync_out);
    sync_generator_inst->vga_hsync_begin_in(s_vga_hsync_timing_begin);
    sync_generator_inst->vga_hsync_end_in(s_vga_hsync_timing_end);
    sync_generator_inst->vga_vsync_begin_in(s_vga_vsync_timing_begin);
    sync_generator_inst->vga_vsync_end_in(s_vga_vsync_timing_end);

    // Timings
    timings_inst = sc_new<m_vga_timings>("timings_inst");
    timings_inst->resolution_mode_in(s_resolution_mode);
    timings_inst->column_active_out(s_column_timing_active);
    timings_inst->column_end_out(s_column_timing_end);
    timings_inst->line_active_out(s_line_timing_active);
    timings_inst->line_end_out(s_line_timing_end);
    timings_inst->vga_hsync_begin_out(s_vga_hsync_timing_begin);
    timings_inst->vga_hsync_end_out(s_vga_hsync_timing_end);
    timings_inst->vga_vsync_begin_out(s_vga_vsync_timing_begin);
    timings_inst->vga_vsync_end_out(s_vga_vsync_timing_end);

    // Line-Column-Counter
    line_column_counter_inst = sc_new<m_line_column_counter>("line_column_counter_inst");
    line_column_counter_inst->clk(clk); // vid_clk
    line_column_counter_inst->reset(reset);
    line_column_counter_inst->enable_in(s_enable_video);
    line_column_counter_inst->vid_column_out(vid_column);
    line_column_counter_inst->vid_line_out(vid_line);
    line_column_counter_inst->column_active_in(s_column_timing_active);
    line_column_counter_inst->column_end_in(s_column_timing_end);
    line_column_counter_inst->line_active_in(s_line_timing_active);
    line_column_counter_inst->line_end_in(s_line_timing_end);
    line_column_counter_inst->blanking_enable_out(s_blanking_enable);
}
