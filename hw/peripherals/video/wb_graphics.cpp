/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Beaurel Ngaleu <beaurel.ingride.ngaleu@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    This file contains [eine kurze, konkrete Beschreibung deiner Datei,
    z. B. “a SystemC implementation of a framebuffer component used in a VGA video pipeline.”]

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

#include "wb_graphics.h"
#include <systemc.h>
#include <piconut.h>

void wb_graphics_modul::connect_ports_to_signals()
{
    clk_sig.write(clk.read());
    reset_sig.write(reset.read());
    enable_sig.write(enable.read());

    wb_stb_sig.write(wb_stb_i.read());
    wb_cyc_sig.write(wb_cyc_i.read());
    wb_sel_sig.write(wb_sel_i.read());
    wb_adr_sig.write(wb_adr_i.read());
    wb_dat_i_sig.write(wb_dat_i.read());
    wb_we_i_sig.write(wb_we_i.read()); 

    // Register Ports aktualisieren (optional, je nach Bedarf für Simulation)
    reg_control_sig.write(reg_control.read());
    reg_status_sig.write(reg_status.read());
    reg_line_sig.write(reg_line.read());
    reg_resolution_mode_sig.write(reg_resolution_mode.read());
    reg_resolution_mode_support_sig.write(reg_resolution_mode_support.read());
    reg_color_mode_sig.write(reg_color_mode.read());
    reg_color_mode_support_sig.write(reg_color_mode_support.read());
}

void wb_graphics_modul::mux_fb_data_read()
{
    fb_data_read_mux_sig.write(fb_data_read_vid_sig.read());
}

void wb_graphics_modul::init_submodule()
{
    /*** Submodule erstellen ***/
    framebuffer_source_inst    = new m_framebuffer_source("framebuffer_source_inst");
    color_translator_inst      = new m_color_translator("color_translator_inst");
    color_generator_inst       = new m_vga_color_generator("color_generator_inst");
    sync_generator_inst        = new m_vga_sync_generator("sync_generator_inst");
    timings_inst               = new m_vga_timings("timings_inst");
    wishbone_inst              = new m_wishbone_t("wishbone_inst");
    line_column_counter_inst   = new m_line_column_counter("line_column_counter_inst");

    /*** Clock & Reset Binding über interne Signale ***/
    framebuffer_source_inst->ctl_clk(clk_wishbone); // bleibt extern
    framebuffer_source_inst->vid_clk(clk_color_translator);
    framebuffer_source_inst->ctl_reset(reset_sig);

    color_generator_inst->clk(clk_sig);
    color_generator_inst->reset(reset_sig);

    sync_generator_inst->clk(clk_sig);
    sync_generator_inst->reset(reset_sig);
    sync_generator_inst->enable(enable_sig);

    wishbone_inst->clk(clk_wishbone);
    wishbone_inst->reset(reset_sig);

    /*** Wishbone mit internen Signalen ***/
    wishbone_inst->wb_stb_i(wb_stb_sig);
    wishbone_inst->wb_cyc_i(wb_cyc_sig);
    wishbone_inst->wb_sel_i(wb_sel_sig);
    wishbone_inst->wb_adr_i(wb_adr_sig);
    wishbone_inst->wb_we_i(wb_we_i_sig);
    wishbone_inst->wb_dat_i(wb_dat_i_sig);

    wishbone_inst->wb_ack_o(wb_ack_o);
    wishbone_inst->wb_err_o(wb_err_o);
    wishbone_inst->wb_rty_o(wb_rty_o);
    wishbone_inst->wb_dat_o(wb_dat_o);

    wishbone_inst->fb_addr(fb_addr);
    wishbone_inst->fb_write_en(fb_write_en);
    wishbone_inst->fb_data_write(fb_data_write);
    wishbone_inst->fb_data_read(fb_data_read_wb_sig);

    // Wishbone-Register-Ports anbinden
    wishbone_inst->reg_control(reg_control);
    wishbone_inst->reg_status(reg_status);
    wishbone_inst->reg_line(reg_line);
    wishbone_inst->reg_resolution_mode(reg_resolution_mode);
    wishbone_inst->reg_resolution_mode_support(reg_resolution_mode_support);
    wishbone_inst->reg_color_mode(reg_color_mode);
    wishbone_inst->reg_color_mode_support(reg_color_mode_support);

    /*** Framebuffer Binding ***/
    framebuffer_source_inst->ctl_addr(fb_addr);
    framebuffer_source_inst->ctl_data_in(ctl_data_in);
    framebuffer_source_inst->ctl_data_out(fb_data_read_vid_sig);
    framebuffer_source_inst->ctl_write_en(fb_write_en);
    framebuffer_source_inst->vid_enable(vid_line_enable);
    framebuffer_source_inst->vid_output(vid_output);
    framebuffer_source_inst->vid_column(vid_line_column);
    framebuffer_source_inst->vid_line(vid_line_line);

    /*** Color Translator ***/
    color_translator_inst->color_mode(color_mode);
    color_translator_inst->color_in(vid_output);
    color_translator_inst->color_out(color_out);
    color_translator_inst->clk(clk_color_translator);
    color_translator_inst->reset(reset_sig);

    /*** VGA Color Generator ***/
    color_generator_inst->video_in(color_out);
    color_generator_inst->blank_enable(vid_line_enable);
    color_generator_inst->vga_red(vga_red);
    color_generator_inst->vga_green(vga_green);
    color_generator_inst->vga_blue(vga_blue);

    /*** Sync Generator ***/
    sync_generator_inst->column(vid_line_column);
    sync_generator_inst->line(vid_line_line);
    sync_generator_inst->vga_hsync(vga_hsync);
    sync_generator_inst->vga_vsync(vga_vsync);
    sync_generator_inst->vga_hsync_begin(vga_hsync_timing_begin);
    sync_generator_inst->vga_hsync_end(vga_hsync_timing_end);
    sync_generator_inst->vga_vsync_begin(vga_vsync_timing_begin);
    sync_generator_inst->vga_vsync_end(vga_vsync_timing_end);

    /*** Timings ***/
    timings_inst->vid_column_active(column_timing_active);
    timings_inst->vid_column_end(column_timing_end);
    timings_inst->vid_line_active(line_timing_active);
    timings_inst->vid_line_end(line_timing_end);
    timings_inst->vga_hsync_begin(vga_hsync_timing_begin);
    timings_inst->vga_hsync_end(vga_hsync_timing_end);
    timings_inst->vga_vsync_begin(vga_vsync_timing_begin);
    timings_inst->vga_vsync_end(vga_vsync_timing_end);
    timings_inst->resolution_mode(resolution_mode);

    /*** Line-Column-Counter ***/
    line_column_counter_inst->vid_column(vid_line_column);
    line_column_counter_inst->vid_line(vid_line_line);
    line_column_counter_inst->vid_column_active(column_timing_active);
    line_column_counter_inst->vid_column_end(column_timing_end);
    line_column_counter_inst->vid_line_active(line_timing_active);
    line_column_counter_inst->vid_line_end(line_timing_end);
    line_column_counter_inst->vid_enable(sig_vid_enable);
    line_column_counter_inst->enable(sig_enable);
    line_column_counter_inst->reset(sig_reset);
    line_column_counter_inst->clk(clk_sig);
}
