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

#include "video_wb.h"
#include "color_palettes.h"

void m_video_wb::pn_trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, wb_slave.ack_o);
    PN_TRACE(tf, wb_slave.adr_i);
    PN_TRACE(tf, wb_slave.cyc_i);
    PN_TRACE(tf, wb_slave.stb_i);
    PN_TRACE(tf, wb_slave.dat_i);
    PN_TRACE(tf, wb_slave.dat_o);
    PN_TRACE(tf, wb_slave.sel_i);
    PN_TRACE(tf, wb_slave.we_i);
    PN_TRACE(tf, wb_slave.err_o);

    PN_TRACE(tf, wb_current_state);

    PN_TRACE(tf, reg_control);
    PN_TRACE(tf, reg_status);
    PN_TRACE(tf, reg_resolution_mode);
    PN_TRACE(tf, reg_color_mode);

    PN_TRACE(tf, fb_ctl_addr_out);
    PN_TRACE(tf, fb_ctl_data_in_out);
    PN_TRACE(tf, fb_ctl_data_out_in);
    PN_TRACE(tf, fb_ctl_write_en_out);
}

void m_video_wb::proc_comb_outputs()
{
    enable_video_out = reg_control.read()[0];
    enable_interrupt_out = reg_control.read()[1];
    resolution_mode_out = reg_resolution_mode.read();
    color_mode_out = reg_color_mode.read();
}

void m_video_wb::proc_comb_wb_slave() // Wishbone Statetransition logic (combinatoric)
{
    // set idle Outputs to default
    wb_slave.ack_o = 0;
    wb_slave.dat_o = 0;
    wb_slave.err_o = 0;
    wb_slave.rty_o = 0;

    c_wb_write_en = 0;

    wb_next_state = wb_current_state.read(); // default next state is current state

    switch(wb_current_state.read())
    {
        case WB_IDLE: // Wishbone Idle State to check if we have a valid transaction

            if(wb_slave.stb_i.read() == 1 && wb_slave.cyc_i.read() == 1) // WB Strobe and cycle valid,
            {
                if((PN_CFG_VIDEO_BASE_ADDRESS <= wb_slave.adr_i.read()) &&
                    (wb_slave.adr_i.read() < (PN_CFG_VIDEO_BASE_ADDRESS + PN_VIDEO_REG_SIZE_BYTES)))
                {
                    if(wb_slave.we_i.read() == 1)
                    {
                        wb_next_state = WB_WRITE1; // Master writes to us (Slave)
                    }
                    else
                    {
                        wb_next_state = WB_READ; // Master reads from us (Slave)
                    }
                }
            }
            break;

        case WB_WRITE1: // Master writes to us -> read from input, write to our register

            c_wb_write_en = 1;

            // set the next wishbone state because writing is not finished yet
            wb_next_state = WB_WRITE2;
            break;

        case WB_WRITE2: // Write Acknowledge

            c_wb_write_en = 1;
            wb_slave.ack_o = 1;

            // after setting the wishbone ack the writing is finished and we can go back to the idle state
            if(wb_slave.stb_i.read() == 0 && wb_slave.cyc_i.read() == 0)
            {
                wb_next_state = WB_IDLE;
            }
            break;

        case WB_READ: // Master Reads from us -> Write data to the bus output

            // Address offset within the block
            sc_uint<PN_CFG_WB_ADDR_WIDTH> off = wb_slave.adr_i.read() - PN_CFG_VIDEO_BASE_ADDRESS;
            sc_uint<PN_CFG_WB_DATA_WIDTH> dout = 0;

            switch(off.to_uint())
            {
                case VIDEO_REG_CONTROL:
                    dout = reg_control.read();
                    break;
                case VIDEO_REG_STATUS:
                    dout = reg_status.read();
                    break;
                case VIDEO_REG_RESOLUTION_MODE:
                    dout = reg_resolution_mode.read();
                    break;
                case VIDEO_REG_RESOLUTION_MODE_SUPPORT:
                    dout = RESOLUTION_MODE_SUPPORTED_MASK_HW;
                    break;
                case VIDEO_REG_COLOR_MODE:
                    dout = reg_color_mode.read();
                    break;
                case VIDEO_REG_COLOR_MODE_SUPPORT:
                    dout = COLOR_MODE_SUPPORTED_MASK_HW;
                    break;
                case VIDEO_REG_SCANLINE:
                    dout = vid_line_in.read();
                    break;
                /**
                 * Hardcoded framebuffer size for now, as we only support 640x480 currently
                 */
                case VIDEO_REG_FRAMEBUFFER_WIDTH:
                    dout = 640 / ((uint32_t)PN_CFG_VIDEO_FB_SCALING_DIVISOR);
                    break;
                case VIDEO_REG_FRAMEBUFFER_HEIGHT:
                    dout = 480 / ((uint32_t)PN_CFG_VIDEO_FB_SCALING_DIVISOR);
                    break;
                default:
                    if(wb_slave.adr_i.read() - PN_CFG_VIDEO_BASE_ADDRESS >= VIDEO_REG_COLOR_MAP &&
                        wb_slave.adr_i.read() - PN_CFG_VIDEO_BASE_ADDRESS < VIDEO_REG_FRAMEBUFFER)
                    {
                        switch(reg_color_mode.read())
                        {
                            case COLOR_MODE_2_MAP:
                                dout = palette_2_bit[(wb_slave.adr_i.read() - PN_CFG_VIDEO_BASE_ADDRESS - VIDEO_REG_COLOR_MAP) >> 2];
                                break;
                            case COLOR_MODE_3_MAP:
                                dout = palette_3_bit[(wb_slave.adr_i.read() - PN_CFG_VIDEO_BASE_ADDRESS - VIDEO_REG_COLOR_MAP) >> 2];
                                break;
                            case COLOR_MODE_4_MAP:
                                dout = palette_4_bit[(wb_slave.adr_i.read() - PN_CFG_VIDEO_BASE_ADDRESS - VIDEO_REG_COLOR_MAP) >> 2];
                                break;
                            case COLOR_MODE_8_MAP:
                                dout = palette_8_bit[(wb_slave.adr_i.read() - PN_CFG_VIDEO_BASE_ADDRESS - VIDEO_REG_COLOR_MAP) >> 2];
                                break;
                            default:
                                dout = 0;
                                break;
                        }
                    }
                    else if((wb_slave.adr_i.read() - PN_CFG_VIDEO_BASE_ADDRESS) >= VIDEO_REG_FRAMEBUFFER)
                    {
                        dout = fb_ctl_data_out_in.read();
                    }
                    break;
            }

            // Optional byte-lane mask on reads
            wb_slave.dat_o = (sc_uint<32>(0), read_with_byte_select(dout));
            wb_slave.ack_o = 1;

            if(wb_slave.stb_i.read() == 0 && wb_slave.cyc_i.read() == 0)
            {
                wb_next_state = WB_IDLE;
            }
            break;
    }
}

void m_video_wb::proc_comb_wb_write()
{
    reg_control_next = reg_control;
    reg_resolution_mode_next = reg_resolution_mode;
    reg_color_mode_next = reg_color_mode;

    if(c_wb_write_en.read() == 1)
    {
        switch(wb_slave.adr_i.read() - PN_CFG_VIDEO_BASE_ADDRESS) // use Address offset to determine target register
        {
            case VIDEO_REG_CONTROL:
                reg_control_next = write_with_byte_select(reg_control.read());
                break;
            case VIDEO_REG_RESOLUTION_MODE:
                reg_resolution_mode_next = write_with_byte_select(reg_resolution_mode.read()).range(4, 0);
                break;
            case VIDEO_REG_COLOR_MODE:
                reg_color_mode_next = write_with_byte_select(reg_color_mode.read()).range(4, 0);
                break;
            default:
                break;
        }
    }
}

void m_video_wb::proc_clk_write_regs()
{
    wb_current_state = WB_IDLE;
    reg_control = 1; // set enable bit to 1
    reg_status = 0;
    reg_resolution_mode = RESOLUTION_MODE_640x480;
    reg_color_mode = COLOR_MODE_2_GRAY;

    while(true)
    {
        wait();

        wb_current_state = wb_next_state.read();
        reg_control = reg_control_next.read();
        reg_resolution_mode = reg_resolution_mode_next.read();
        reg_color_mode = reg_color_mode_next.read();

        /**
         * Framebuffer write control
         * In clk domain hold the data to write to the framebuffer before the address changes
         */
        fb_ctl_addr_out = 0;
        fb_ctl_write_en_out = 0;
        fb_ctl_data_in_out = 0;

        if(wb_slave.adr_i.read() - PN_CFG_VIDEO_BASE_ADDRESS >= VIDEO_REG_FRAMEBUFFER &&
            wb_slave.adr_i.read() - PN_CFG_VIDEO_BASE_ADDRESS - VIDEO_REG_FRAMEBUFFER < PN_VIDEO_REG_SIZE_BYTES)
        {
            // writing 32-bit words to pixel-size addressable framebuffer so divide by 4
            fb_ctl_addr_out = (wb_slave.adr_i.read() - PN_CFG_VIDEO_BASE_ADDRESS - VIDEO_REG_FRAMEBUFFER) >> 2;
            if(c_wb_write_en.read() == 1)
            {
                fb_ctl_write_en_out = 1;
                fb_ctl_data_in_out = write_with_byte_select(0).range(FB_RAM_DATA_WIDTH - 1, 0);
            }
        }
    }
}

sc_uint<PN_CFG_WB_DATA_WIDTH> m_video_wb::write_with_byte_select(sc_uint<PN_CFG_WB_DATA_WIDTH> original_word)
{
    sc_uint<PN_CFG_WB_DATA_WIDTH> mask = 0;
    sc_uint<PN_CFG_WB_DATA_WIDTH / 8> wb_sel_i_var = wb_slave.sel_i.read().range(32 / 8 - 1, 0);

    // Selective byte masking
    if(wb_sel_i_var & 0b0001)
    {
        mask |= 0x000000FF; // Byte 0
    }
    if(wb_sel_i_var & 0b0010)
    {
        mask |= 0x0000FF00; // Byte 1
    }
    if(wb_sel_i_var & 0b0100)
    {
        mask |= 0x00FF0000; // Byte 2
    }
    if(wb_sel_i_var & 0b1000)
    {
        mask |= 0xFF000000; // Byte 3
    }

    return (original_word & ~mask) | (wb_slave.dat_i.read().range(31, 0) & mask);
}

sc_uint<PN_CFG_WB_DATA_WIDTH> m_video_wb::read_with_byte_select(sc_uint<PN_CFG_WB_DATA_WIDTH> original_word)
{
    sc_uint<PN_CFG_WB_DATA_WIDTH> mask = 0;
    sc_uint<PN_CFG_WB_DATA_WIDTH / 8> wb_sel_i_var = wb_slave.sel_i.read().range(32 / 8 - 1, 0);

    // Selective byte masking
    if(wb_sel_i_var & 0b0001)
    {
        mask |= 0x000000FF; // Byte 0
    }
    if(wb_sel_i_var & 0b0010)
    {
        mask |= 0x0000FF00; // Byte 1
    }
    if(wb_sel_i_var & 0b0100)
    {
        mask |= 0x00FF0000; // Byte 2
    }
    if(wb_sel_i_var & 0b1000)
    {
        mask |= 0xFF000000; // Byte 3
    }

    return (original_word & mask);
}
