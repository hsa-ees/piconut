/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Claus Janicher <claus.janicher@tha.de>
                     2025 Johannes Hofmann <johannes.hofmann1@tha.de>
                     2025 Lukas Bauer <lukas.bauer1@tha.de>
                     2025 Tristan Kundrat <tristan.kundrat@tha.de>
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

// Design template
#include "wishbone_template.h"
#include <stdint.h>
#include <systemc.h>

// defines for testscenario
#define PERIOD_NS 10.0

// initialize TB signals
sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

// signals for wishbone bus
sc_signal<sc_uint<WB_ADR_WIDTH>> PN_NAME(wb_adr_i);
sc_signal<sc_uint<WB_DAT_WIDTH>> PN_NAME(wb_dat_o);
sc_signal<sc_uint<WB_DAT_WIDTH>> PN_NAME(wb_dat_i);
sc_signal<sc_uint<4>> PN_NAME(wb_sel_i);
sc_signal<bool> PN_NAME(wb_we_i);
sc_signal<bool> PN_NAME(wb_stb_i);
sc_signal<bool> PN_NAME(wb_ack_o);
sc_signal<bool> PN_NAME(wb_cyc_i);

sc_signal<bool> PN_NAME(wb_rty_o);
sc_signal<bool> PN_NAME(wb_err_o);

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

    // wait for acknoledgement of the write
    while(wb_ack_o.read() == 0)
    {
        run_cycle();
        count++;

        if(count > 100)
        {
            PN_ERROR("WB Write Timeout");
        }
    }

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

    // wait for acknoledgement of the read
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

void trigger_reset()
{
    run_cycle();
    reset = 1;
    run_cycle();
    reset = 0;
    run_cycle();
}

void test_write_read()
{
    trigger_reset();

    PN_INFO("m_wishbone_template_tb: run test test_write_read() ...");
    uint32_t expected = 0xdeadbeef;

    wishbone_write(
        CFG_WB_SLAVE_TEMPLATE_ADDRESS + m_wishbone_t::ADR_REG_0,
        expected);

    uint32_t actual = wishbone_read(
        CFG_WB_SLAVE_TEMPLATE_ADDRESS + m_wishbone_t::ADR_REG_0);

    PN_ASSERTM(actual == expected, "Write or read failed");
    PN_INFO("wishbone_template_tb: test passed");
}

void test_byte_select_write()
{
    trigger_reset();

    PN_INFO("m_wishbone_template_tb: run test test_byte_select_write() ...");

    uint32_t inital = 0xdeadbeef;
    for(int i = 0; i < (1 << 4); ++i)
    {
        trigger_reset();

        uint32_t expected = 0;
        wishbone_write(
            CFG_WB_SLAVE_TEMPLATE_ADDRESS + m_wishbone_t::ADR_REG_0,
            inital,
            i);

        for(int k = 0; k < 4; k++)
        {
            if(((i >> k) & 1) == 1)
            {
                expected |= inital & (0xffU << (k * 8));
            }
        }

        uint32_t actual = wishbone_read(
            CFG_WB_SLAVE_TEMPLATE_ADDRESS + m_wishbone_t::ADR_REG_0);

        PN_ASSERTF(actual == expected,
            ("Byte select write failed with byte select: 0x%x.\n"
             "Expected: 0x%08x, Actual: 0x%08x",
                i,
                expected,
                actual));
    }

    PN_INFO("wishbone_template_tb: test passed");
}

void test_byte_select_read()
{
    trigger_reset();

    PN_INFO("m_wishbone_template_tb: run test test_byte_select_read() ...");

    uint32_t inital = 0xdeadbeef;
    wishbone_write(
        CFG_WB_SLAVE_TEMPLATE_ADDRESS + m_wishbone_t::ADR_REG_0,
        inital);

    for(int i = 0; i < (1 << 4); ++i)
    {
        uint32_t expected = 0;

        for(int k = 0; k < 4; k++)
        {
            if(((i >> k) & 1) == 1)
            {
                expected |= inital & (0xffU << (k * 8));
            }
        }

        uint32_t actual = wishbone_read(
            CFG_WB_SLAVE_TEMPLATE_ADDRESS + m_wishbone_t::ADR_REG_0,
            i);

        PN_ASSERTF(actual == expected,
            ("Byte select read failed with byte select: 0x%x.\n"
             "Expected: 0x%08x, Actual: 0x%08x",
                i,
                expected,
                actual));
    }

    PN_INFO("wishbone_template_tb: test passed");
}

int sc_main(int argc, char** argv)
{

    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("wishbone_template_tb");

    // Initialize the Design under Testing (DUT)
    m_wishbone_t dut_inst{"dut_inst"};

    // connect the wishbone signals

    dut_inst.clk(clk);
    dut_inst.reset(reset);
    dut_inst.wb_adr_i(wb_adr_i);
    dut_inst.wb_dat_i(wb_dat_i);
    dut_inst.wb_dat_o(wb_dat_o);
    dut_inst.wb_we_i(wb_we_i);
    dut_inst.wb_stb_i(wb_stb_i);
    dut_inst.wb_ack_o(wb_ack_o);
    dut_inst.wb_cyc_i(wb_cyc_i);
    dut_inst.wb_sel_i(wb_sel_i);
    dut_inst.wb_err_o(wb_err_o);
    dut_inst.wb_rty_o(wb_rty_o);

    // Traces of signals here

    dut_inst.pn_trace(tf, pn_cfg_vcd_level); // trace signals of

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Testbench code here
    trigger_reset();

    test_write_read();
    test_byte_select_write();
    test_byte_select_read();

    run_cycle(); // end with a wait

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}