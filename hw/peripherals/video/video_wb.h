/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Niklas Sirch <niklas.sirch1@tha.de>

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
 * @fn SC_MODULE(m_video_wb)
 *
 * @brief Video Module Wishbone Interface
 *
 * It translates Wishbone bus accesses (adr/dat/we/stb/cyc/sel) into:
 * - Updates of video configuration registers (Control, Resolution, Color),
 * - Direct access to the video Framebuffer RAM,
 * - Status readback for current video scanlines and module state.
 * - Offers data from register to other video modules
 *
 * @par
 * Wishbone Ports (adr/dat/we/stb/cyc/sel) are included in pn_wishbone_slave_t wb_slave
 * @par Connection to Video/Graphics submodules:
 * @param[in] clk                   : System clock.
 * @param[in] reset                 : Active-high reset.
 * @param[in] vid_line_in         : Current horizontal/vertical scanline being processed.
 * @param[in] fb_ctl_data_out_in    : Data read from the Framebuffer RAM.
 * @param[out] fb_ctl_addr_out    : Address bus for the Framebuffer RAM.
 * @param[out] fb_ctl_data_in_out   : Data bus to be written to Framebuffer RAM.
 * @param[out] fb_ctl_write_en_out  : Write enable signal for Framebuffer RAM.
 * @param[out] enable_video_out   : Global video output enable signal.
 * @param[out] enable_interrupt_out : Video interrupt enable signal.
 * @param[out] resolution_mode_out  : Selected resolution configuration.
 * @param[out] color_mode_out       : Selected color depth/format configuration.
 */

#ifndef __VIDEO_WB_H__
#define __VIDEO_WB_H__

#include <systemc.h>
#include <piconut.h>

#include "video_defs.h"
#include "video_defs_hw.h"

SC_MODULE(m_video_wb)
{

public:
    enum e_wb_state
    {
        WB_IDLE = 0,
        WB_READ,
        WB_WRITE1,
        WB_WRITE2
    };

    // Wishbone ports & signals
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    pn_wishbone_slave_t wb_slave;

    sc_in<sc_uint<H_COUNTER_WIDTH>> PN_NAME(vid_line_in);
    sc_in<sc_uint<FB_RAM_DATA_WIDTH>> PN_NAME(fb_ctl_data_out_in);

    sc_out<sc_uint<FB_RAM_ADDR_WIDTH>> PN_NAME(fb_ctl_addr_out);
    sc_out<sc_uint<FB_RAM_DATA_WIDTH>> PN_NAME(fb_ctl_data_in_out);
    sc_out<bool> PN_NAME(fb_ctl_write_en_out);

    sc_out<bool> PN_NAME(enable_video_out);
    sc_out<bool> PN_NAME(enable_interrupt_out);
    sc_out<sc_uint<5>> PN_NAME(resolution_mode_out);
    sc_out<sc_uint<5>> PN_NAME(color_mode_out);

    SC_HAS_PROCESS(m_video_wb);
    m_video_wb(
        sc_module_name name,
        pn_wb_adr_t base_address)
        : sc_module(name)
        , wb_slave{
              .alen = 32,
              .dlen = 32,
              .base_address = base_address,
              .size = PN_VIDEO_REG_SIZE_BYTES}
    {
        SC_METHOD(proc_comb_wb_slave);
        sensitive << wb_slave.stb_i
                  << wb_slave.cyc_i
                  << wb_slave.we_i
                  << wb_slave.adr_i
                  << wb_slave.dat_i
                  << wb_slave.sel_i;
        sensitive << wb_current_state;
        sensitive << reg_control
                  << reg_status
                  << reg_resolution_mode
                  << reg_color_mode;
        sensitive << vid_line_in << fb_ctl_data_out_in;

        SC_METHOD(proc_comb_outputs);
        sensitive << reg_control << reg_resolution_mode << reg_color_mode;

        SC_METHOD(proc_comb_wb_write);
        sensitive << c_wb_write_en
                  << wb_slave.adr_i << wb_slave.sel_i << wb_slave.dat_i
                  << reg_control << reg_resolution_mode << reg_color_mode;

        SC_CTHREAD(proc_clk_write_regs, clk.pos());
        reset_signal_is(reset, true);
    }

    void pn_trace(sc_trace_file * tf, int level = 1);
    void proc_clk_write_regs();
    void proc_comb_wb_slave();
    void proc_comb_wb_write();
    void proc_comb_outputs();

protected:
    sc_signal<bool> PN_NAME(c_wb_write_en);

    /** Graphics registers */
    sc_signal<sc_uint<32>> PN_NAME(reg_control);
    sc_signal<sc_uint<32>> PN_NAME(reg_control_next);
    sc_signal<sc_uint<32>> PN_NAME(reg_status);
    sc_signal<sc_uint<5>> PN_NAME(reg_resolution_mode);
    sc_signal<sc_uint<5>> PN_NAME(reg_resolution_mode_next);
    sc_signal<sc_uint<5>> PN_NAME(reg_color_mode);
    sc_signal<sc_uint<5>> PN_NAME(reg_color_mode_next);

    /** Register */
    sc_signal<sc_uint<3>> PN_NAME(wb_current_state);
    sc_signal<sc_uint<3>> PN_NAME(wb_next_state);

    sc_uint<PN_CFG_WB_DATA_WIDTH> write_with_byte_select(sc_uint<PN_CFG_WB_DATA_WIDTH> original_word);
    sc_uint<PN_CFG_WB_DATA_WIDTH> read_with_byte_select(sc_uint<PN_CFG_WB_DATA_WIDTH> original_word);
};

#endif //__VIDEO_WB_H__
