/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)     2025 Eikya Lagisetti <eikya.lagisetti1@tha.de>
                    2025 Sebastian Ebenhöh <sebastian.moritz.ebenhoeh@tha.de>
                    2025 Johannes Fleiner <Johannes.Fleiner1@tha.de>

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

#include "../i2c.h"
#include "../i2c_defs.h"
#include <systemc.h>
#include <stdint.h>

#include "../i2c_wishbone/i2c_wishbone.h"
#include "../i2c_controller/i2c_controller.h"
#include "../i2c_clk_control/i2c_clk_control.h"
#include "../i2c_data_control/i2c_data_control.h"

#define PERIOD_NS 40.0

// ================== Global Testbench Signals ==================
sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

sc_signal<bool> PN_NAME(sda_in_sig);
sc_signal<bool> PN_NAME(scl_in_sig);
sc_signal<bool> PN_NAME(sda_out_sig);
sc_signal<bool> PN_NAME(scl_out_sig);
sc_signal<bool> PN_NAME(sda_oe_sig);
sc_signal<bool> PN_NAME(scl_oe_sig);

sc_signal<bool> PN_NAME(slave_stretch_sig);

// Wishbone
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

static const uint32_t I2C_BASE = PN_CFG_I2C_BASE_ADDRESS;
static const uint32_t REG_CR1 = I2C_BASE + I2C_REG_CR1;
static const uint32_t REG_SR1 = I2C_BASE + I2C_REG_SR1;
static const uint32_t REG_SR2 = I2C_BASE + I2C_REG_SR2;
static const uint32_t REG_DR = I2C_BASE + I2C_REG_DR;
static const uint32_t REG_CCR = I2C_BASE + I2C_REG_CCR;
static const uint32_t REG_TRISE = I2C_BASE + I2C_REG_TRISE;

// ================== Helper Functions ==================

SC_MODULE(BusModel)
{
    sc_in<bool> scl_out;
    sc_in<bool> slave_stretch;
    sc_out<bool> scl_in;
    sc_out<bool> sda_in;

    void bus_logic()
    {
        // Wenn DUT oder Slave zieht (LOW), ist der Bus LOW. Sonst HIGH (Pull-Up).
        if(scl_out.read() == false || slave_stretch.read() == true)
        {
            scl_in.write(false);
        }
        else
        {
            scl_in.write(true);
        }

        sda_in.write(false);
    }

    SC_CTOR(BusModel)
    {
        SC_METHOD(bus_logic);
        sensitive << scl_out << slave_stretch;
    }
};

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

void trigger_reset()
{
    run_cycle();
    reset = 1;
    run_cycle();
    reset = 0;
    run_cycle();
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

    run_cycle();

    // clear the signals at the wb
    wb_stb_i = 0;
    wb_cyc_i = 0;
    wb_we_i = 0;
    wb_sel_i = 0;
    wb_adr_i = 0;

    run_cycle();

    return data;
}

// Read-Modify-Write
void wishbone_rmw(uint32_t adr, uint32_t set_mask, uint32_t clr_mask, uint8_t sel = 0xF)
{
    uint32_t v = wishbone_read(adr, sel);
    v = (v & ~clr_mask) | set_mask;
    wishbone_write(adr, v, sel);
}

void i2c_wait_sr1(uint32_t sr1_addr, uint32_t mask, bool set = true, int max_polls = 10000)
{
    for(int i = 0; i < max_polls; ++i)
    {
        uint32_t sr1 = wishbone_read(sr1_addr);
        if(set)
        {
            if(sr1 & mask)
                return;
        }
        else
        {
            if((sr1 & mask) == 0)
                return;
        }
    }
    PN_ERROR("I2C SR1 wait timeout");
}

int sc_main(int argc, char** argv)
{
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("i2c_tb");

    m_i2c i_dut{"i_dut", PN_CFG_I2C_BASE_ADDRESS};

    // Connect Wishbon
    i_dut.clk(clk);
    i_dut.reset(reset);
    i_dut.wb_slave.adr_i(wb_adr_i);
    i_dut.wb_slave.dat_i(wb_dat_i);
    i_dut.wb_slave.dat_o(wb_dat_o);
    i_dut.wb_slave.we_i(wb_we_i);
    i_dut.wb_slave.stb_i(wb_stb_i);
    i_dut.wb_slave.ack_o(wb_ack_o);
    i_dut.wb_slave.cyc_i(wb_cyc_i);
    i_dut.wb_slave.sel_i(wb_sel_i);
    i_dut.wb_slave.err_o(wb_err_o);
    i_dut.wb_slave.rty_o(wb_rty_o);

    i_dut.sda_i(sda_in_sig);
    i_dut.scl_i(scl_in_sig);

    i_dut.sda_o(sda_out_sig);
    i_dut.scl_o(scl_out_sig);

    i_dut.sda_oe(sda_oe_sig);
    i_dut.scl_oe(scl_oe_sig);

    // --- Bus-Simulation ---
    BusModel i_bus("i_bus");
    i_bus.scl_out(scl_out_sig);
    i_bus.slave_stretch(slave_stretch_sig);
    i_bus.scl_in(scl_in_sig);
    i_bus.sda_in(sda_in_sig);

    i_dut.pn_trace(tf, pn_cfg_vcd_level);

    sc_start(SC_ZERO_TIME);
    cout << "\n\t\t*****I2C Simulation started*****" << endl;

    trigger_reset();

    cout << "\n===== i2c_init() =====" << endl;
    // PCLK = 25 MHz, I2C = 100 kHz
    uint32_t ccr_value = 25000000 / (2 * 100000);
    uint32_t trise_value = 25 + 1;
    wishbone_write(REG_CCR, ccr_value);
    wishbone_write(REG_TRISE, trise_value);
    wishbone_rmw(REG_CR1, I2C_CR1_PE, 0);

    run_cycle(1000);

    cout << "\n===== i2c_start() =====" << endl;
    wishbone_rmw(REG_CR1, I2C_CR1_START, 0);
    i2c_wait_sr1(REG_SR1, I2C_SR1_SB, true);
    cout << "[" << sc_time_stamp() << "] START condition detected (SB=1)" << endl;
    wishbone_read(REG_SR1);

    run_cycle(1000);

    cout << "\n===== i2c_send_addr() for write =====" << endl;
    uint8_t addr_byte = 0x3C << 1;
    wishbone_write(REG_DR, addr_byte);
    i2c_wait_sr1(REG_SR1, I2C_SR1_ADDR, true);
    cout << "[" << sc_time_stamp() << "] ADDR bit set" << endl;
    wishbone_read(REG_SR1);
    wishbone_read(REG_SR2);

    run_cycle(10000);

    cout << "\n===== i2c_send_data() =====" << endl;
    wishbone_write(REG_DR, 0x55);
    i2c_wait_sr1(REG_SR1, I2C_SR1_TXE, true);
    cout << "[" << sc_time_stamp() << "] TXE set, data transmission finished" << endl;
    // temp = wishbone_read(REG_SR1);

    run_cycle(10000);

    cout << "\n===== i2c_stop() =====" << endl;
    wishbone_rmw(REG_CR1, I2C_CR1_STOP, 0);

    run_cycle(10000);

    cout << "\n===== i2c_start() =====" << endl;
    wishbone_rmw(REG_CR1, I2C_CR1_START, 0);
    i2c_wait_sr1(REG_SR1, I2C_SR1_SB, true);
    cout << "[" << sc_time_stamp() << "] START condition detected (SB=1)" << endl;
    wishbone_read(REG_SR1);

    run_cycle(1000);

    cout << "\n===== i2c_send_addr() for read =====" << endl;
    addr_byte = 0x3C << 1 | 0x01;
    wishbone_write(REG_DR, addr_byte);
    i2c_wait_sr1(REG_SR1, I2C_SR1_ADDR, true);
    cout << "[" << sc_time_stamp() << "] ADDR bit set" << endl;
    wishbone_read(REG_SR1);
    wishbone_read(REG_SR2);

    run_cycle(10000);

    cout << "\n===== i2c_recv_data() =====" << endl;
    wishbone_write(REG_CR1, ~I2C_CR1_ACK);
    i2c_wait_sr1(REG_SR1, I2C_SR1_RXNE, true);
    wishbone_read(REG_DR);
    cout << "[" << sc_time_stamp() << "] RXNE set, data receive finished" << endl;

    run_cycle(10000);

    cout << "\n===== i2c_stop() =====" << endl;
    wishbone_rmw(REG_CR1, I2C_CR1_STOP, 0);

    run_cycle(1000);

    cout << "\n===== i2c_de_init() =====" << endl;
    wishbone_rmw(REG_CR1, 0, I2C_CR1_PE);

    run_cycle(5000);

    PN_END_TRACE();
    cout << "\n\t\t*****I2C Simulation complete*****" << endl;
    return 0;
}
