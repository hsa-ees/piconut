/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Beaurel Ngaleu <beaurel.ingride.ngaleu@tha.de>
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
 * @fn SC_MODULE(wb_graphics_modul)
 * @author Beaurel I. Ngaleu
 * @brief 
 * This module connects several subsystems such as Wishbone control, framebuffer,
 * VGA sync generator, color generator, and timing logic into a complete video system.
 *
 * @par Ports:
 * @param[in]  clk                   Main clock signal
 * @param[in]  reset                 Global reset signal
 * @param[in]  enable                Module enable signal
 * @param[in]  clk_wishbone          Clock for the Wishbone interface
 * @param[in]  wb_stb_i              Wishbone strobe input
 * @param[in]  wb_we_i               Wishbone write enable input
 * @param[in]  wb_cyc_i              Wishbone cycle signal
 * @param[in]  wb_sel_i              Wishbone byte select signal
 * @param[out] wb_ack_o              Wishbone acknowledge output
 * @param[out] wb_err_o              Wishbone error output
 * @param[out] wb_rty_o              Wishbone retry output
 * @param[in]  wb_adr_i              Wishbone address input
 * @param[in]  wb_dat_i              Wishbone data input
 * @param[out] wb_dat_o              Wishbone data output
 * @param[out] fb_addr               Framebuffer address bus
 * @param[out] fb_write_en           Write enable for framebuffer
 * @param[in]  fb_data_read          Data read from the framebuffer
 * @param[out] fb_data_write         Data written to the framebuffer
 * @param[out] reg_control           Control register
 * @param[in]  reg_status            Status register
 * @param[in]  reg_line              Current line from the status module
 * @param[out] reg_resolution_mode   Selected video resolution mode
 * @param[in]  reg_resolution_mode_support Supported resolution modes
 * @param[out] reg_color_mode        Selected color mode
 * @param[in]  reg_color_mode_support Supported color modes
 * @param[in]  ctl_reset             Reset signal for framebuffer access control
 * @param[in]  ctl_addr              Address for framebuffer control access
 * @param[in]  ctl_data_in           Input data for write operations
 * @param[out] ctl_data_out          Output data from read operations
 * @param[in]  ctl_write_en          Write enable signal for control access
 * @param[in]  vid_clk               Clock for video signal processing
 * @param[in]  vid_enable            Enable signal for active video area
 * @param[in]  vid_column            Current column (x position)
 * @param[in]  vid_line              Current line (y position)
 * @param[out] vid_output            Current pixel data (e.g., color) from framebuffer
 * @param[out] vga_red               Red color component output (4-bit)
 * @param[out] vga_green             Green color component output (4-bit)
 * @param[out] vga_blue              Blue color component output (4-bit)
 * @param[out] vga_hsync             Horizontal sync signal
 * @param[out] vga_vsync             Vertical sync signal
 * @param[in]  vga_hsync_begin       Start of horizontal sync phase
 * @param[in]  vga_hsync_end         End of horizontal sync phase
 * @param[in]  vga_vsync_begin       Start of vertical sync phase
 * @param[in]  vga_vsync_end         End of vertical sync phase
 * @param[in]  column                Current column position for sync generation
 * @param[in]  line                  Current line position for sync generation
 * @param[in]  resolution_mode            Video resolution mode
 * @param[out] column_timing_end         End of horizontal visible area
 * @param[out] column_timing_active      Active horizontal display area
 * @param[out] line_timing_end           End of vertical visible area
 * @param[out] line_timing_active        Active vertical display area
 * @param[out] vga_hsync_timing_begin    Timing of horizontal sync start
 * @param[out] vga_hsync_timing_end      Timing of horizontal sync end
 * @param[out] vga_vsync_timing_begin    Timing of vertical sync start
 * @param[out] vga_vsync_timing_end      Timing of vertical sync end
 * @param[in]  vid_column_end        Last column (maximum horizontal counter value)
 * @param[in]  vid_line_end          Last line (maximum vertical counter value)
 * @param[in]  vid_column_active     Visible horizontal area
 * @param[in]  vid_line_active       Visible vertical area
 * @param[out] vid_line_column       Current column counter output
 * @param[out] vid_line_line         Current line counter output
 * @param[out] vid_line_enable       Enable signal for visible video data
 * @param[in]  clk_color_translator  Clock for color translation module
 * @param[in]  color_mode            Color mode (e.g., RGB888, RGB444, etc.)
 * @param[in]  color_in              Input color value
 * @param[out] color_out             Translated output color value (e.g., RGB888)
 */


#ifndef __WB_GRAPHICS_H__
#define __WB_GRAPHICS_H__

#include <systemc.h>
#include <piconut.h>

#include "framebuffer_source.h"
#include "color_translator.h"
#include "vga_color_generator.h"
#include "vga_sync_generator.h"
#include "vga_timings.h"
#include "wb_slave.h"
#include "wb_slave_config.h"
#include "framebuffer_ram.h"
#include "line_column_counter.h"

SC_MODULE(wb_graphics_modul)
{
public:
    // ========== Externe Ports ==========
    // Clock & Reset & Enable
    sc_in<bool> PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);
    sc_in_clk PN_NAME(clk_clk);
    sc_in<bool> PN_NAME(enable);

    // WISHBONE
    sc_in_clk PN_NAME(clk_wishbone);
    sc_in<bool> PN_NAME(wb_stb_i), PN_NAME(wb_cyc_i), PN_NAME(wb_we_i);
    sc_in<sc_uint<WB_DAT_WIDTH / 8>> PN_NAME(wb_sel_i);
    sc_out<bool> PN_NAME(wb_ack_o), PN_NAME(wb_err_o), PN_NAME(wb_rty_o);
    sc_in<sc_uint<WB_ADR_WIDTH>> PN_NAME(wb_adr_i);
    sc_in<sc_uint<WB_DAT_WIDTH>> PN_NAME(wb_dat_i);
    sc_out<sc_uint<WB_DAT_WIDTH>> PN_NAME(wb_dat_o);
    sc_out<sc_uint<FB_RAM_ADDR_WIDTH>> PN_NAME(fb_addr);
    sc_out<bool> PN_NAME(fb_write_en);
    sc_in<sc_uint<FB_RAM_DATA_WIDTH>> PN_NAME(fb_data_read);
    sc_out<sc_uint<FB_RAM_DATA_WIDTH>> PN_NAME(fb_data_write);

    // Wishbone Register Ports (neu ergänzt)
    sc_out<sc_uint<32>> PN_NAME(reg_control);
    sc_in<sc_uint<32>> PN_NAME(reg_status);
    sc_in<sc_uint<32>> PN_NAME(reg_line);
    sc_out<sc_uint<5>> PN_NAME(reg_resolution_mode);
    sc_in<sc_uint<32>> PN_NAME(reg_resolution_mode_support);
    sc_out<sc_uint<5>> PN_NAME(reg_color_mode);
    sc_in<sc_uint<32>> PN_NAME(reg_color_mode_support);

    // VGA COLOR GENERATOR
    sc_in<sc_uint<24>> PN_NAME(video_in);
    sc_out<sc_uint<4>> PN_NAME(vga_red);
    sc_out<sc_uint<4>> PN_NAME(vga_green);
    sc_out<sc_uint<4>> PN_NAME(vga_blue);

    // FRAMEBUFFER SOURCE
    sc_in<bool> PN_NAME(ctl_reset);
    sc_in<sc_uint<FB_RAM_ADDR_WIDTH>> PN_NAME(ctl_addr);
    sc_in<sc_uint<FB_RAM_DATA_WIDTH>> PN_NAME(ctl_data_in);
    sc_out<sc_uint<FB_RAM_DATA_WIDTH>> PN_NAME(ctl_data_out);
    sc_in<bool> PN_NAME(ctl_write_en);
    sc_in_clk PN_NAME(vid_clk);
    sc_in<bool> PN_NAME(vid_enable);
    sc_in<sc_uint<H_COUNTER_WIDTH>> PN_NAME(vid_column);
    sc_in<sc_uint<V_COUNTER_WIDTH>> PN_NAME(vid_line);
    sc_out<sc_uint<32>> PN_NAME(vid_output);

    // VGA SYNC GENERATOR
    sc_in<sc_uint<H_COUNTER_WIDTH>> PN_NAME(vga_hsync_begin);
    sc_in<sc_uint<H_COUNTER_WIDTH>> PN_NAME(vga_hsync_end);
    sc_in<sc_uint<V_COUNTER_WIDTH>> PN_NAME(vga_vsync_begin);
    sc_in<sc_uint<V_COUNTER_WIDTH>> PN_NAME(vga_vsync_end);
    sc_in<sc_uint<H_COUNTER_WIDTH>> PN_NAME(column);
    sc_in<sc_uint<V_COUNTER_WIDTH>> PN_NAME(line);
    sc_out<bool> PN_NAME(vga_hsync);
    sc_out<bool> PN_NAME(vga_vsync);

    // LINE-COLUMN-COUNTER
    sc_in<sc_uint<16>> PN_NAME(vid_column_end);
    sc_in<sc_uint<16>> PN_NAME(vid_line_end);
    sc_in<sc_uint<16>> PN_NAME(vid_column_active);
    sc_in<sc_uint<16>> PN_NAME(vid_line_active);
    sc_out<sc_uint<16>> PN_NAME(vid_line_column);
    sc_out<sc_uint<16>> PN_NAME(vid_line_line);
    sc_out<bool> PN_NAME(vid_line_enable);

    // COLOR TRANSLATOR
    sc_in_clk PN_NAME(clk_color_translator);
    sc_in<sc_uint<5>> PN_NAME(color_mode);
    sc_in<sc_uint<32>> PN_NAME(color_in);
    sc_out<sc_uint<24>> PN_NAME(color_out);

    // VGA TIMINGS
    sc_in<sc_uint<5>> PN_NAME(resolution_mode);
    sc_out<sc_uint<H_COUNTER_WIDTH>> PN_NAME(column_timing_end); 
    sc_out<sc_uint<H_COUNTER_WIDTH>> PN_NAME(column_timing_active);
    sc_out<sc_uint<V_COUNTER_WIDTH>> PN_NAME(line_timing_end);
    sc_out<sc_uint<V_COUNTER_WIDTH>> PN_NAME(line_timing_active);
    sc_out<sc_uint<H_COUNTER_WIDTH>> PN_NAME(vga_hsync_timing_begin);
    sc_out<sc_uint<H_COUNTER_WIDTH>> PN_NAME(vga_hsync_timing_end);
    sc_out<sc_uint<V_COUNTER_WIDTH>> PN_NAME(vga_vsync_timing_begin);
    sc_out<sc_uint<V_COUNTER_WIDTH>> PN_NAME(vga_vsync_timing_end);

    // Submodule-Pointer
    m_framebuffer_source* framebuffer_source_inst;
    m_color_translator* color_translator_inst;
    m_vga_color_generator* color_generator_inst;
    m_vga_sync_generator* sync_generator_inst;
    m_vga_timings* timings_inst;
    m_wishbone_t* wishbone_inst;
    m_line_column_counter* line_column_counter_inst;

    //  Konstruktor 
    SC_CTOR(wb_graphics_modul)
    {
        SC_METHOD(connect_ports_to_signals);
        sensitive << clk
                  << reset
                  << enable
                  << wb_stb_i
                  << wb_cyc_i
                  << wb_sel_i
                  << wb_we_i
                  << wb_adr_i
                  << wb_dat_i;

                  
        SC_METHOD(mux_fb_data_read);
        sensitive << fb_data_read_wb_sig << fb_data_read_vid_sig;

        init_submodule();
    }

    void init_submodule();
    void connect_ports_to_signals(); // für interne signalverbindung

protected:
    // Interne Signale zur Submodul-Verbindung
    sc_signal<bool> clk_sig, reset_sig, enable_sig;
    sc_signal<bool> wb_stb_sig, wb_cyc_sig, wb_we_i_sig;
    sc_signal<sc_uint<WB_DAT_WIDTH / 8>> wb_sel_sig;
    sc_signal<sc_uint<WB_ADR_WIDTH>> wb_adr_sig;
    sc_signal<sc_uint<WB_DAT_WIDTH>> wb_dat_i_sig, wb_dat_o_sig;
    sc_signal<sc_uint<FB_RAM_ADDR_WIDTH>> fb_addr_sig;
    sc_signal<bool> fb_write_en_sig;
    sc_signal<bool> sig_enable;
    sc_signal<bool> sig_reset;
    sc_signal<bool> sig_vid_enable;
    sc_signal<sc_uint<FB_RAM_DATA_WIDTH>> fb_data_read_sig, fb_data_write_sig;

    sc_signal<sc_uint<32>> reg_control_sig;
    sc_signal<sc_uint<32>> reg_status_sig;
    sc_signal<sc_uint<32>> reg_line_sig;
    sc_signal<sc_uint<5>> reg_resolution_mode_sig;
    sc_signal<sc_uint<32>> reg_resolution_mode_support_sig;
    sc_signal<sc_uint<5>> reg_color_mode_sig;
    sc_signal<sc_uint<32>> reg_color_mode_support_sig;
    sc_signal<sc_uint<FB_RAM_DATA_WIDTH>> fb_data_read_wb_sig;
    sc_signal<sc_uint<FB_RAM_DATA_WIDTH>> fb_data_read_vid_sig;
    sc_signal<sc_uint<FB_RAM_DATA_WIDTH>> fb_data_read_mux_sig;

    void mux_fb_data_read();
};

#endif // __WB_GRAPHICS_H__
