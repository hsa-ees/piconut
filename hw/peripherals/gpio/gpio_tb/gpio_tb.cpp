/*******************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2026 Daniel Sommerfeldt <daniel.sommerfeldt1@tha.de>
                     Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    Testbench for the GPIO Module.

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

 ******************************************************************************/

#include <systemc.h>
#include <piconut.h>

#include "../gpio.h"
#include "../gpio_defs.h"

#define PERIOD_NS 40.0 // 25 MHz

// ================== Global Signals ==================
sc_signal<bool> clk;
sc_signal<bool> reset;

// Wishbone signals
sc_signal<bool> wb_stb_i;
sc_signal<bool> wb_cyc_i;
sc_signal<bool> wb_we_i;
sc_signal<pn_wb_adr_t> wb_adr_i;
sc_signal<pn_wb_dat_t> wb_dat_i;
sc_signal<pn_wb_dat_t> wb_dat_o;
sc_signal<pn_wb_sel_t> wb_sel_i;
sc_signal<bool> wb_ack_o;
sc_signal<bool> wb_err_o;
sc_signal<bool> wb_rty_o;

// GPIO Pins
sc_signal<sc_uint<GPIO_MAX_PINS>> gpio_input;
sc_signal<sc_uint<GPIO_MAX_PINS>> gpio_output;

// ================== Helper Functions ==================

// Toggle clock for 'n' cycles
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

// Perform a hardware reset
void trigger_reset()
{
    PN_INFO("Triggering reset ...");
    reset = 1;
    run_cycle(5);
    reset = 0;
    run_cycle(5);
}

// Wishbone Write Transaction
void wishbone_write(uint32_t adr, uint32_t dat, uint8_t sel = 0xF)
{
    wb_stb_i = 1;
    wb_cyc_i = 1;
    wb_we_i = 1;
    wb_sel_i = sel;
    wb_adr_i = adr;
    wb_dat_i = dat;

    int cycles = 0;
    while(wb_ack_o.read() == 0)
    {
        run_cycle();
        cycles++;

        if(cycles > 100)
        {
            PN_INFOF(("WB Write Timeout at address 0x%x", adr));
            PN_ERROR("WB Write Timeout");
            break;
        }
    }

    wb_stb_i = 0;
    wb_cyc_i = 0;
    wb_we_i = 0;
    wb_sel_i = 0;
    wb_adr_i = 0;
    wb_dat_i = 0;

    run_cycle();
}

// Wishbone Read Transaction
uint32_t wishbone_read(uint32_t adr, uint8_t sel = 0xF)
{
    wb_stb_i = 1;
    wb_cyc_i = 1;
    wb_we_i = 0; // Read
    wb_sel_i = sel;
    wb_adr_i = adr;
    wb_dat_i = 0;

    int cycles = 0;
    while(wb_ack_o.read() == 0)
    {
        run_cycle();
        cycles++;
        if(cycles > 100)
        {
            PN_INFOF(("WB Read Timeout at address 0x%x", adr));
            PN_ERROR("WB Read Timeout");
            break;
        }
    }

    uint32_t dat = (uint32_t)wb_dat_o.read();

    wb_stb_i = 0;
    wb_cyc_i = 0;
    wb_we_i = 0;
    wb_adr_i = 0;

    run_cycle();
    return dat;
}

// ================== Main Testbench ==================

int sc_main(int argc, char** argv)
{
    // Initialize PicoNut parsing/tracing tools
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("gpio_tb");

    // Instantiate DUT with the Base Address defined in gpio_defs.h
    m_gpio i_dut("i_dut", PN_CFG_GPIO_BASE_ADDRESS, 32, 32);

    // Connect Signals
    i_dut.clk(clk);
    i_dut.reset(reset);

    // Connect Wishbone
    i_dut.wb_slave.stb_i(wb_stb_i);
    i_dut.wb_slave.cyc_i(wb_cyc_i);
    i_dut.wb_slave.we_i(wb_we_i);
    i_dut.wb_slave.adr_i(wb_adr_i);
    i_dut.wb_slave.dat_i(wb_dat_i);
    i_dut.wb_slave.sel_i(wb_sel_i);
    i_dut.wb_slave.dat_o(wb_dat_o);
    i_dut.wb_slave.ack_o(wb_ack_o);
    i_dut.wb_slave.err_o(wb_err_o);
    i_dut.wb_slave.rty_o(wb_rty_o);

    // Connect GPIO pins
    i_dut.input(gpio_input);
    i_dut.output(gpio_output);

    // Setup Trace
    i_dut.pn_trace(tf, pn_cfg_vcd_level);

    // Initialize signals
    wb_stb_i = 0;
    wb_cyc_i = 0;
    wb_we_i = 0;
    gpio_input = 0;

    // Start Simulation
    sc_start(SC_ZERO_TIME);
    PN_INFO("\t\t***** GPIO testbench started *****");

    trigger_reset();

    // --- Test Case 1: Check input ---
    PN_INFO("[TEST 1] Testing input ...");

    // 1a. Verify val default is 0
    PN_ASSERT(0 == wishbone_read(PN_CFG_GPIO_BASE_ADDRESS + GPIO_REG_INPUT_VAL));

    // 1b. Verify val stays 0 when en is 0
    uint32_t input_val = 0xFFFFFFFF;
    gpio_input = input_val;
    PN_ASSERT(0 == wishbone_read(PN_CFG_GPIO_BASE_ADDRESS + GPIO_REG_INPUT_VAL));

    // 1c. Verify val is only 1 if en is 1
    uint32_t input_en = 0xDEADBEEF;
    wishbone_write(PN_CFG_GPIO_BASE_ADDRESS + GPIO_REG_INPUT_EN, input_en);
    gpio_input = input_val;
    PN_ASSERT(0xDEADBEEF == wishbone_read(PN_CFG_GPIO_BASE_ADDRESS + GPIO_REG_INPUT_VAL));

    gpio_input = 0;

    trigger_reset();

    // --- Test Case 2: Check output ---
    PN_INFO("[TEST 2] Testing output ...");

    // 2a. Verify val default is 0
    PN_ASSERT(0 == wishbone_read(PN_CFG_GPIO_BASE_ADDRESS + GPIO_REG_OUTPUT_VAL));

    // 2b. Verify val stays 0 when en is 0
    uint32_t output_val = 0xFFFFFFFF;
    wishbone_write(PN_CFG_GPIO_BASE_ADDRESS + GPIO_REG_OUTPUT_EN, output_val);
    PN_ASSERT(0 == gpio_output.read());

    // 2c. Verify val is only 1 if en is 1
    uint32_t output_en = 0xDEADBEEF;
    wishbone_write(PN_CFG_GPIO_BASE_ADDRESS + GPIO_REG_OUTPUT_VAL, output_en);
    PN_ASSERT(0xDEADBEEF == gpio_output.read());

    PN_END_TRACE();
    PN_INFOF(("\t\t***** GPIO testbench finished *****"));
    return 0;
}
