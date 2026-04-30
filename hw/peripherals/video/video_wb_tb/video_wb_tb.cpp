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

#include "../video_wb.h"

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);
sc_signal<bool> PN_NAME(c_enable);

// signals for wishbone bus
sc_signal<pn_wb_adr_t> PN_NAME(wb_adr_i);
sc_signal<pn_wb_dat_t> PN_NAME(wb_dat_o);
sc_signal<pn_wb_dat_t> PN_NAME(wb_dat_i);
sc_signal<pn_wb_sel_t> PN_NAME(wb_sel_i);
sc_signal<bool> PN_NAME(wb_we_i);
sc_signal<bool> PN_NAME(wb_stb_i);
sc_signal<bool> PN_NAME(wb_ack_o);
sc_signal<bool> PN_NAME(wb_cyc_i);

sc_signal<bool> PN_NAME(wb_rty_o);
sc_signal<bool> PN_NAME(wb_err_o);

sc_signal<sc_uint<H_COUNTER_WIDTH>> PN_NAME(vid_line);

sc_signal<sc_uint<FB_RAM_DATA_WIDTH>> PN_NAME(fb_ctl_data_out_in);
sc_signal<sc_uint<FB_RAM_ADDR_WIDTH>> PN_NAME(fb_ctl_addr_out);
sc_signal<sc_uint<FB_RAM_DATA_WIDTH>> PN_NAME(fb_ctl_data_in_out);
sc_signal<bool> PN_NAME(fb_ctl_write_en_out);

sc_signal<sc_uint<5>> PN_NAME(resolution_mode_out);
sc_signal<sc_uint<5>> PN_NAME(color_mode_out);

sc_signal<bool> PN_NAME(enable_video_out);
sc_signal<bool> PN_NAME(enable_interrupt_out);

void run_cycle(int cycles = 1)
{
    for(int i = 0; i < cycles; i++)
    {
        clk = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

void wishbone_write(uint32_t adr, uint32_t data, uint8_t sel = 0xF)
{

    int count = 0;

    // set the signals for wb write setup
    wb_stb_i = 1;
    wb_cyc_i = 1;
    wb_we_i = 1;
    wb_sel_i = sel;
    wb_adr_i = adr;
    wb_dat_i = data;

    // wait for acknowledgement of the write
    while(wb_ack_o.read() == 0)
    {
        run_cycle();
        count++;

        if(count > 100)
        {
            PN_ERROR("WB Write Timeout");
        }
    }

    run_cycle();

    // clear the signals at the wb
    wb_stb_i = 0;
    wb_cyc_i = 0;
    wb_we_i = 0;
    wb_sel_i = 0;
    wb_adr_i = 0;
    wb_dat_i = 0;

    run_cycle();
}

uint32_t wishbone_read(uint32_t adr, uint8_t sel = 0xF)
{

    int cycles = 0;

    // set the signals for wb read setup
    wb_stb_i = 1;
    wb_cyc_i = 1;
    wb_we_i = 0;
    wb_sel_i = sel;
    wb_adr_i = adr;
    wb_dat_i = 0;

    // wait for acknowledgement of the read
    while(wb_ack_o.read() == 0)
    {
        run_cycle();
        cycles++;

        if(cycles > 100)
        {
            PN_ERROR("WB Read Timeout");
        }
    }

    // getting the data from the wb
    uint32_t data = (uint32_t)wb_dat_o.read();

    // clear the signals at the wb
    wb_stb_i = 0;
    wb_cyc_i = 0;
    wb_we_i = 0;
    wb_sel_i = 0;
    wb_adr_i = 0;

    run_cycle();

    return data;
}

int sc_main(int argc, char** argv)
{
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("video_wb");

    m_video_wb i_dut{"i_dut", PN_CFG_VIDEO_BASE_ADDRESS};
    i_dut.clk(clk);
    i_dut.reset(reset);
    i_dut.wb_slave.adr_i(wb_adr_i);
    i_dut.wb_slave.dat_o(wb_dat_o);
    i_dut.wb_slave.dat_i(wb_dat_i);
    i_dut.wb_slave.sel_i(wb_sel_i);
    i_dut.wb_slave.we_i(wb_we_i);
    i_dut.wb_slave.stb_i(wb_stb_i);
    i_dut.wb_slave.ack_o(wb_ack_o);
    i_dut.wb_slave.cyc_i(wb_cyc_i);
    i_dut.wb_slave.rty_o(wb_rty_o);
    i_dut.wb_slave.err_o(wb_err_o);

    i_dut.fb_ctl_data_out_in(fb_ctl_data_out_in);
    i_dut.vid_line_in(vid_line);
    i_dut.fb_ctl_addr_out(fb_ctl_addr_out);
    i_dut.fb_ctl_data_in_out(fb_ctl_data_in_out);
    i_dut.fb_ctl_write_en_out(fb_ctl_write_en_out);

    i_dut.resolution_mode_out(resolution_mode_out);
    i_dut.color_mode_out(color_mode_out);

    i_dut.enable_video_out(enable_video_out);
    i_dut.enable_interrupt_out(enable_interrupt_out);

    i_dut.pn_trace(tf, pn_cfg_vcd_level);

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Reset
    PN_INFO("Resetting");
    reset = 1;
    run_cycle(1);

    reset = 0;
    run_cycle(1);

    // --- Tests for Wishbone register access ---
    pn_wb_adr_t base = PN_CFG_VIDEO_BASE_ADDRESS;

    // Check default registers
    uint32_t val_ctrl = wishbone_read(base + VIDEO_REG_CONTROL);
    PN_ASSERTM(val_ctrl == 1, "Default control register not zero");

    uint32_t val_res = wishbone_read(base + VIDEO_REG_RESOLUTION_MODE);
    PN_ASSERTM(val_res == RESOLUTION_MODE_640x480, "Default resolution mode mismatch");

    uint32_t val_col = wishbone_read(base + VIDEO_REG_COLOR_MODE);
    PN_ASSERTM(val_col == COLOR_MODE_2_GRAY, "Default color mode mismatch");

    // Full-word write/read to control register
    PN_INFO("Testing full-word register write/read");
    wishbone_write(base + VIDEO_REG_CONTROL, 0x12345678, 0xF);
    val_ctrl = wishbone_read(base + VIDEO_REG_CONTROL);
    PN_ASSERTM(val_ctrl == 0x12345678, "Control register full write/read failed");

    // Byte-select write (only byte 0) to control register
    PN_INFO("Testing byte-select register write");
    wishbone_write(base + VIDEO_REG_CONTROL, 0x000000AA, 0x1);
    val_ctrl = wishbone_read(base + VIDEO_REG_CONTROL);
    PN_ASSERTM(val_ctrl == 0x123456AA, "Control register byte-select write failed");

    // Framebuffer write: write byte 0 only and check fb control outputs
    PN_INFO("Testing framebuffer write and byte-select behavior");
    // Target framebuffer word address (first word)
    pn_wb_adr_t fb_addr = base + VIDEO_REG_FRAMEBUFFER + 0x200;
    // write a 32-bit pattern but only select byte0 using low-level WB toggles
    wb_stb_i = 1;
    wb_cyc_i = 1;
    wb_we_i = 1;
    wb_sel_i = 0x1;
    wb_adr_i = fb_addr;
    wb_dat_i = 0x11223344;

    // run one cycle to enter WB_WRITE1 and drive c_wb_write_en -> framebuffer outputs
    run_cycle(1);

    PN_ASSERTM(fb_ctl_write_en_out.read() == 0, "Framebuffer write enable not asserted");

    run_cycle(1);

    PN_ASSERTM(fb_ctl_write_en_out.read() == 1, "Framebuffer write enable not asserted");
    PN_ASSERTM(fb_ctl_addr_out.read() == 0x80, "Framebuffer address output incorrect");
    PN_ASSERTM((uint32_t)fb_ctl_data_in_out.read() == 0x44, "Framebuffer data LSB incorrect for byte-select write");

    // clear signals (but keep DUT progressing so it can ACK)
    wb_stb_i = 0;
    wb_cyc_i = 0;
    wb_we_i = 0;
    wb_sel_i = 0;
    wb_adr_i = 0;
    wb_dat_i = 0;
    run_cycle(1);

    PN_ASSERTM(fb_ctl_write_en_out.read() == 0, "Framebuffer write enable not asserted");

    // Test: low-level write with byte-select selecting other byte (byte1)
    wb_stb_i = 1;
    wb_cyc_i = 1;
    wb_we_i = 1;
    wb_sel_i = 0x2;
    wb_adr_i = fb_addr;
    wb_dat_i = 0xAABBCCDD;

    run_cycle(1);

    run_cycle(1);

    PN_ASSERTM(fb_ctl_write_en_out.read() == 1, "Framebuffer write enable not asserted on byte1 write");
    PN_ASSERTM((uint32_t)fb_ctl_data_in_out.read() == 0x00, "Framebuffer data LSB should be zero when writing non-LSB byte (as implemented)");

    // clear signals
    wb_stb_i = 0;
    wb_cyc_i = 0;
    wb_we_i = 0;
    wb_sel_i = 0;
    wb_adr_i = 0;
    wb_dat_i = 0;
    run_cycle(1);

    // Framebuffer read
    PN_INFO("Testing framebuffer read");
    // Target framebuffer word address (first word)
    fb_addr = base + VIDEO_REG_FRAMEBUFFER + 0x200;
    // write a 32-bit pattern but only select byte0 using low-level WB toggles
    wb_stb_i = 1;
    wb_cyc_i = 1;
    wb_we_i = 0;
    wb_sel_i = 0x1;
    wb_adr_i = fb_addr;

    // run one cycle to enter WB_WRITE1 and drive c_wb_write_en -> framebuffer outputs
    run_cycle(1);
    fb_ctl_data_out_in = 0xDEADBEEF; // set data coming from framebuffer

    run_cycle(1);

    PN_ASSERTM(fb_ctl_write_en_out.read() == 0, "Framebuffer write enable is asserted on read");
    PN_ASSERTM(fb_ctl_addr_out.read() == 0x80, "Framebuffer address output incorrect");
    PN_ASSERTM(wb_dat_o.read() == 0x000000EF, "Wishbone read data incorrect for byte-select read");

    // clear signals (but keep DUT progressing so it can ACK)
    wb_stb_i = 0;
    wb_cyc_i = 0;
    wb_we_i = 0;
    wb_sel_i = 0;
    wb_adr_i = 0;
    wb_dat_i = 0;
    run_cycle(1);

    PN_INFO("Testing framebuffer read with byte-select in non-existing byte");
    // Target framebuffer word address (first word)
    fb_addr = base + VIDEO_REG_FRAMEBUFFER + 0x200;
    // write a 32-bit pattern but only select byte0 using low-level WB toggles
    wb_stb_i = 1;
    wb_cyc_i = 1;
    wb_we_i = 0;
    wb_sel_i = 0x2;
    wb_adr_i = fb_addr;
    fb_ctl_data_out_in = 0xDEADBEEF; // set data coming from framebuffer

    run_cycle(1);

    PN_ASSERTM(fb_ctl_write_en_out.read() == 0, "Framebuffer write enable is asserted on read");
    PN_ASSERTM(fb_ctl_addr_out.read() == 0x80, "Framebuffer address output incorrect");
    PN_ASSERTM(wb_dat_o.read() == 0x0, "Wishbone read data incorrect for byte-select read");

    // clear signals (but keep DUT progressing so it can ACK)
    wb_stb_i = 0;
    wb_cyc_i = 0;
    wb_we_i = 0;
    wb_sel_i = 0;
    wb_adr_i = 0;
    wb_dat_i = 0;
    run_cycle(1);

    PN_INFO("Testing framebuffer read with whole word byte-select");
    // Target framebuffer word address (first word)
    fb_addr = base + VIDEO_REG_FRAMEBUFFER + 0x200;
    // write a 32-bit pattern but only select byte0 using low-level WB toggles
    wb_stb_i = 1;
    wb_cyc_i = 1;
    wb_we_i = 0;
    wb_sel_i = 0b1111;
    wb_adr_i = fb_addr;
    fb_ctl_data_out_in = 0xDEADBEEF; // set data coming from framebuffer

    run_cycle(1);

    PN_ASSERTM(fb_ctl_write_en_out.read() == 0, "Framebuffer write enable is asserted on read");
    PN_ASSERTM(fb_ctl_addr_out.read() == 0x80, "Framebuffer address output incorrect");
    PN_ASSERTM(wb_dat_o.read() == 0xEF, "Wishbone read data incorrect for byte-select read");

    // clear signals (but keep DUT progressing so it can ACK)
    wb_stb_i = 0;
    wb_cyc_i = 0;
    wb_we_i = 0;
    wb_sel_i = 0;
    wb_adr_i = 0;
    wb_dat_i = 0;
    run_cycle(1);

    // --- Tests for Enable Signals ---
    PN_INFO("Testing enable_video_out signal");
    wishbone_write(base + VIDEO_REG_CONTROL, 0x1, 0xF); // Enable video output
    run_cycle(1);
    PN_ASSERTM(enable_video_out.read() == 1, "enable_video_out signal not set correctly");

    PN_INFO("Testing enable_interrupt_out signal");
    wishbone_write(base + VIDEO_REG_CONTROL, 0x2, 0xF); // Enable interrupt
    run_cycle(1);
    PN_ASSERTM(enable_interrupt_out.read() == 1, "enable_interrupt_out signal not set correctly");

    PN_INFO("Testing both enable signals");
    wishbone_write(base + VIDEO_REG_CONTROL, 0x3, 0xF); // Enable both signals
    run_cycle(1);
    PN_ASSERTM(enable_video_out.read() == 1, "enable_video_out signal not set correctly when both enabled");
    PN_ASSERTM(enable_interrupt_out.read() == 1, "enable_interrupt_out signal not set correctly when both enabled");

    PN_INFO("Testing full-word color_map read");
    wishbone_write(base + VIDEO_REG_COLOR_MODE, COLOR_MODE_16_RGB565, 0xF);
    val_ctrl = wishbone_read(base + VIDEO_REG_COLOR_MAP);
    PN_ASSERTM(val_ctrl == 0x0, "Default color map register not zero");
    wishbone_write(base + VIDEO_REG_COLOR_MODE, COLOR_MODE_3_MAP, 0xF);
    val_ctrl = wishbone_read(base + VIDEO_REG_COLOR_MAP + 4);
    PN_ASSERTM(val_ctrl == 0xab5675, "Color map register wrong");

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}
