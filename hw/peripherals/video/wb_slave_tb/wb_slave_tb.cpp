/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Claus Janicher <claus.janicher@tha.de>
                     2025 Johannes Hofmann <johannes.hofmann1@tha.de>
                     2025 Lukas Bauer <lukas.bauer1@tha.de>
                     2025 Tristan Kundrat <tristan.kundrat@tha.de>
                     2025 Beaurel Ngaleu <beaurel.ingride.ngaleu@tha.de>
                     2025 Martin Erichsen <martin.erichsen@tha.de>
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

#include "wb_slave_config.h"
#include "wb_slave.h"
#include <systemc.h>
#include <piconut.h>

#define PERIOD_NS 10

// Control Signals
sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

// Wishbone Signals
sc_signal<sc_uint<WB_ADR_WIDTH>> PN_NAME(wb_adr_i);
sc_signal<sc_uint<WB_DAT_WIDTH>> PN_NAME(wb_dat_o);
sc_signal<sc_uint<WB_DAT_WIDTH>> PN_NAME(wb_dat_i);
sc_signal<sc_uint<4>> PN_NAME(wb_sel_i);
sc_signal<bool> PN_NAME(wb_we_i);
sc_signal<bool> PN_NAME(wb_stb_i);
sc_signal<bool> PN_NAME(wb_cyc_i);
sc_signal<bool> PN_NAME(wb_ack_o);
sc_signal<bool> PN_NAME(wb_err_o);
sc_signal<bool> PN_NAME(wb_rty_o);

// Register Signals
sc_signal<sc_uint<32>> PN_NAME(reg_control);
sc_signal<sc_uint<32>> PN_NAME(reg_status);
sc_signal<sc_uint<32>> PN_NAME(reg_line);
sc_signal<sc_uint<5>> PN_NAME(reg_color_mode);
sc_signal<sc_uint<32>> PN_NAME(reg_color_mode_support);
sc_signal<sc_uint<5>> PN_NAME(reg_resolution_mode);
sc_signal<sc_uint<32>> PN_NAME(reg_resolution_mode_support);

// Framebuffer Control Signals
sc_signal<sc_uint<FB_RAM_ADDR_WIDTH>> PN_NAME(fb_address);
sc_signal<bool> PN_NAME(fb_write_enable);
sc_signal<sc_uint<FB_RAM_DATA_WIDTH>> PN_NAME(fb_data_write);
sc_signal<sc_uint<FB_RAM_DATA_WIDTH>> PN_NAME(fb_data_read);

// Functions
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

void do_reset()
{
    run_cycle();
    reset = 1;
    run_cycle();
    reset = 0;
    run_cycle();
}

void wishbone_write(uint32_t adr, uint32_t data, uint8_t sel = 0xF)
{
    int cycles = 0;

    // Set the signals for wb write setup
    wb_stb_i = 1;
    wb_cyc_i = 1;
    wb_we_i = 1;
    wb_sel_i = sel;
    wb_adr_i = adr;
    wb_dat_i = data;

    // Wait for acknowledgement of the write
    while(wb_ack_o.read() == 0)
    {
        run_cycle();
        cycles++;

        if(cycles >= 100)
        {
            PN_ASSERTF(false, ("Timed out: no ack after %d cycles", cycles));
            break;
        }
    }

    // Clear the signals at the wb
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

    // Set the signals for wb read setup
    wb_stb_i = 1;
    wb_cyc_i = 1;
    wb_we_i = 0;
    wb_sel_i = sel;
    wb_adr_i = adr;
    wb_dat_i = 0;

    // Wait for acknowledgement of the read
    while(wb_ack_o.read() == 0)
    {
        run_cycle();
        cycles++;

        if(cycles >= 100)
        {
            PN_ASSERTF(false, ("Timed out: no ack after %d cycles", cycles));
            break;
        }
    }

    // Getting the data from the wb
    uint32_t data = (uint32_t)wb_dat_o.read();

    // Clear the signals at the wb
    wb_stb_i = 0;
    wb_cyc_i = 0;
    wb_we_i = 0;
    wb_sel_i = 0;
    wb_adr_i = 0;

    run_cycle();

    return data;
}

/////////////
// Test Cases

void test_control_register()
{
    const uint32_t address = CFG_WB_SLAVE_ADDRESS + m_wishbone_t::ADR_CONTROL;

    PN_INFO("Testing CONTROL register");

    uint32_t data_read;
    uint32_t data = 0x0000000;

    // Write value to register
    PN_INFO("  case: writing via wishbone");
    wishbone_write(address, data);
    run_cycle();  // gib der Logik Zeit
    run_cycle();

    // Read value of module output
    PN_INFO("  case: reading reg_control");
    data_read = reg_control.read();
    PN_ASSERTF(data == data_read, ("reg_control returned different value: expected 0x%08X, got 0x%08X", data, data_read));

    // Readback register via wishbone interface
    PN_INFO("  case: reading via wishbone");
    uint32_t data_read_wb = wishbone_read(address);
    PN_ASSERTF(data == data_read, ("Readback via wishbone returned different value: expected 0x%08X, got 0x%08X", data, data_read));
}

void test_status_register()
{
    const uint32_t address = CFG_WB_SLAVE_ADDRESS + m_wishbone_t::ADR_STATUS;

    PN_INFO("Testing STATUS register");

    uint32_t data_read;
    uint32_t data = 0x12345678;

    // Set status value
    reg_status = data;

    // Read status via wishbone interface
    PN_INFO("  case: reading via wishbone");
    data_read = wishbone_read(address);
    PN_ASSERTF(data == data_read, ("Read via wishbone returned different value: expected 0x%08X, got 0x%08X", data, data_read));
}

void test_scanline_register()
{
    const uint32_t address = CFG_WB_SLAVE_ADDRESS + m_wishbone_t::ADR_LINE;

    PN_INFO("Testing SCANLINE register");

    uint32_t data_read;
    uint32_t data = 0x12345678;

    // Set scanline value
    reg_line = data;

    // Read scanline via wishbone interface
    PN_INFO("  case: reading via wishbone");
    data_read = wishbone_read(address);
    PN_ASSERTF(data == data_read, ("Read via wishbone returned different value: expected 0x%08X, got 0x%08X", data, data_read));
}

void test_resolution_mode_register()
{
    const uint32_t address = CFG_WB_SLAVE_ADDRESS + m_wishbone_t::ADR_RESOLUTION_MODE;

    PN_INFO("Testing RESOLUTION_MODE register");

    uint32_t data_read;
    uint32_t data = 0x12345678;

    // Write value to register
    PN_INFO("  case: writing via wishbone");
    wishbone_write(address, data);

    // Read value of module output
    PN_INFO("  case: reading reg_resolution_mode");
    data_read = reg_resolution_mode.read();
    PN_ASSERTF(data == data_read, ("reg_resolution_mode returned different value: expected 0x%08X, got 0x%08X", data, data_read));

    // Readback register via wishbone interface
    PN_INFO("  case: reading via wishbone");
    data_read = wishbone_read(address);
    PN_ASSERTF(data == data_read, ("Readback via wishbone returned different value: expected 0x%08X, got 0x%08X", data, data_read));
}

void test_resolution_mode_support_register()
{
    const uint32_t address = CFG_WB_SLAVE_ADDRESS + m_wishbone_t::ADR_RESOLUTION_MODE_SUPPORT;

    PN_INFO("Testing RESOLUTION_MODE_SUPPORT register");

    uint32_t data_read;
    uint32_t data = 0x12345678;

    // Set value of port
    reg_resolution_mode_support = data;

    // Read register via wishbone interface
    PN_INFO("  case: reading via wishbone");
    data_read = wishbone_read(address);
    PN_ASSERTF(data == data_read, ("Read via wishbone returned different value: expected 0x%08X, got 0x%08X", data, data_read));
}

void test_color_mode_register()
{
    const uint32_t address = CFG_WB_SLAVE_ADDRESS + m_wishbone_t::ADR_COLOR_MODE;

    PN_INFO("Testing COLOR_MODE register");

    uint32_t data_read;
    uint32_t data = 0x12345678;

    // Write value to register
    PN_INFO("  case: writing via wishbone");
    wishbone_write(address, data);

    // Read value of module output
    PN_INFO("  case: reading reg_color_mode");
    data_read = reg_color_mode.read();
    PN_ASSERTF(data == data_read, ("reg_color_mode returned different value: expected 0x%08X, got 0x%08X", data, data_read));

    // Readback register via wishbone interface
    PN_INFO("  case: reading via wishbone");
    data_read = wishbone_read(address);
    PN_ASSERTF(data == data_read, ("Readback via wishbone returned different value: expected 0x%08X, got 0x%08X", data, data_read));
}

void test_color_mode_support_register()
{
    const uint32_t address = CFG_WB_SLAVE_ADDRESS + m_wishbone_t::ADR_COLOR_MODE_SUPPORT;

    PN_INFO("Testing COLOR_MODE_SUPPORT register");

    uint32_t data_read;
    uint32_t data = 0x12345678;

    // Set value of port
    reg_color_mode_support = data;

    // Read register via wishbone interface
    PN_INFO("  case: reading via wishbone");
    data_read = wishbone_read(address);
    PN_ASSERTF(data == data_read, ("Read via wishbone returned different value: expected 0x%08X, got 0x%08X", data, data_read));
}

void test_fb_address_register()
{
    const uint32_t address = CFG_WB_SLAVE_ADDRESS + m_wishbone_t::INTERN_FB_ADDRESS;

    PN_INFO("Testing FB_ADDRESS register");

    uint32_t data_read;
    uint32_t data = 0x12345678;

    // Write value to register
    PN_INFO("  case: writing via wishbone");
    wishbone_write(address, data);

    // Read value of module output
    PN_INFO("  case: reading fb_address");
    data_read = fb_address.read();
    PN_ASSERTF(data == data_read, ("fb_address returned different value: expected 0x%08X, got 0x%08X", data, data_read));

    // Readback register via wishbone interface
    PN_INFO("  case: reading via wishbone");
    data_read = wishbone_read(address);
    PN_ASSERTF(data == data_read, ("Readback via wishbone returned different value: expected 0x%08X, got 0x%08X", data, data_read));
}

void test_fb_data_register()
{
    const uint32_t address = CFG_WB_SLAVE_ADDRESS + m_wishbone_t::INTERN_FB_DATA;

    PN_INFO("Testing FB_DATA register");

    uint32_t data_read;
    uint32_t data = 0x87654321;

    // Set read value
    fb_data_read = data;

    // Read register via wishbone interface
    PN_INFO("  case: reading via wishbone");
    data_read = wishbone_read(address);
    PN_ASSERTF(data == data_read, ("Read via wishbone returned different value: expected 0x%08X, got 0x%08X", data, data_read));

    data = 0xDEADBEEF;

    // Checking if wishbone write is correct
    PN_INFO("  case: writing via wishbone");
    int cycles = 0;

    // Set the signals for wb write setup
    wb_stb_i = 1;
    wb_cyc_i = 1;
    wb_we_i = 1;
    wb_sel_i = 0xF;
    wb_adr_i = address;
    wb_dat_i = data;

    // Wait for framebuffer write enable
    for(; cycles < 50; cycles++)
    {
        run_cycle();

        if(fb_write_enable.read() == 1)
            break;

        PN_ASSERTM(wb_ack_o.read() == 0, "ack before fb_write_enable got asserted");
    }

    PN_ASSERTF(fb_write_enable.read() == 1, ("Timed out: fb_write_enable not asserted after %d cycles", cycles));

    // Wait for acknowledgement of the write
    for(; cycles < 50; cycles++)
    {
        run_cycle();
        if(wb_ack_o.read() == 1)
            break;
    }

    PN_ASSERTF(wb_ack_o.read() == 1, ("Timed out: no ack after %d cycles", cycles));

    // Clear the signals at the wb
    wb_stb_i = 0;
    wb_cyc_i = 0;
    wb_we_i = 0;
    wb_sel_i = 0;
    wb_adr_i = 0;
    wb_dat_i = 0;

    run_cycle();
}

int sc_main(int argc, char** argv)
{
    // Uncomment to keep running after failed assertion
    // pn_disable_assert_abort = true;

    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("wb_graphics_tb");

    m_wishbone_t dut_inst{"dut_inst"};

    // Connect signals
    dut_inst.clk(clk);
    dut_inst.reset(reset);

    dut_inst.wb_adr_i(wb_adr_i);
    dut_inst.wb_dat_i(wb_dat_i);
    dut_inst.wb_dat_o(wb_dat_o);
    dut_inst.wb_stb_i(wb_stb_i);
    dut_inst.wb_ack_o(wb_ack_o);
    dut_inst.wb_cyc_i(wb_cyc_i);
    dut_inst.wb_sel_i(wb_sel_i);
    dut_inst.wb_err_o(wb_err_o);
    dut_inst.wb_rty_o(wb_rty_o);
    dut_inst.wb_we_i(wb_we_i);

    dut_inst.reg_control(reg_control);
    dut_inst.reg_status(reg_status);
    dut_inst.reg_line(reg_line);
    dut_inst.reg_color_mode(reg_color_mode);
    dut_inst.reg_color_mode_support(reg_color_mode_support);
    dut_inst.reg_resolution_mode(reg_resolution_mode);
    dut_inst.reg_resolution_mode_support(reg_resolution_mode_support);

    dut_inst.fb_addr(fb_address);
    dut_inst.fb_write_en(fb_write_enable);
    dut_inst.fb_data_write(fb_data_write);
    dut_inst.fb_data_read(fb_data_read);

    // Trace
    dut_inst.pn_trace(tf, pn_cfg_vcd_level);

    sc_start(SC_ZERO_TIME);
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Testbench sequence
    do_reset();

    test_control_register();
    test_status_register();
    /*test_scanline_register();
    test_resolution_mode_register();
    test_color_mode_register();
    test_resolution_mode_support_register();
    test_color_mode_support_register();
    test_fb_address_register();
    test_fb_data_register();*/

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}
