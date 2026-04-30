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

// Design template
#include "../i2c_wishbone.h"
#include <stdint.h>
#include <systemc.h>

// defines for testscenario
#define PERIOD_NS 40

// initialize TB signals
sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

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

// internal dummy signals for i2c_wishbone
sc_signal<bool> PN_NAME(clear_regs_sig);

// CR1 related
sc_signal<bool> PN_NAME(cr1_enable_sig);
sc_signal<bool> PN_NAME(cr1_start_sig);
sc_signal<bool> PN_NAME(cr1_stop_sig);
sc_signal<bool> PN_NAME(cr1_ack_sig);
sc_signal<bool> PN_NAME(clear_cr1_start_sig);
sc_signal<bool> PN_NAME(clear_cr1_stop_sig);

// SR1 related
sc_signal<bool> PN_NAME(sr1_start_bit_sig);
sc_signal<bool> PN_NAME(sr1_addr_bit_sig);
sc_signal<bool> PN_NAME(sr1_txe_sig);
sc_signal<bool> PN_NAME(sr1_btf_sig);
sc_signal<bool> PN_NAME(sr1_af_sig);
sc_signal<bool> PN_NAME(sr1_rxne_sig);

// SR2 related
sc_signal<bool> PN_NAME(sr2_msl_sig);
sc_signal<bool> PN_NAME(sr2_busy_sig);
sc_signal<bool> PN_NAME(sr2_tra_sig);

// DR / data path
sc_signal<sc_uint<8>> PN_NAME(dr_sig);
sc_signal<sc_uint<8>> PN_NAME(tx_data_sig);
sc_signal<sc_uint<8>> PN_NAME(shift_data_sig);
sc_signal<bool> PN_NAME(enable_dr_write_sig);
sc_signal<bool> PN_NAME(CPU_write_DR_sig);
sc_signal<bool> PN_NAME(CPU_read_DR_sig);

// CPU read flags
sc_signal<bool> PN_NAME(CPU_read_SR1_sig);
sc_signal<bool> PN_NAME(CPU_read_SR2_sig);

// Timing outputs
sc_signal<sc_uint<16>> PN_NAME(ccr_sig);
sc_signal<sc_uint<6>> PN_NAME(trise_sig);

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

    // clear the signals at the wb
    wb_stb_i = 0;
    wb_cyc_i = 0;
    wb_we_i = 0;
    wb_sel_i = 0;
    wb_adr_i = 0;

    run_cycle();

    return data;
}

// Read-Modify-Write (Bits setzen/löschen per Masken)
void wishbone_rmw(uint32_t adr, uint32_t set_mask, uint32_t clr_mask, uint8_t sel = 0xF)
{
    uint32_t v = wishbone_read(adr, sel);
    v = (v & ~clr_mask) | set_mask;
    wishbone_write(adr, v, sel);
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

    PN_INFO("m_wb_tb: run test test_write_read() ...");
    uint32_t expected = 0xdeadbeef;

    wishbone_write(
        PN_CFG_I2C_BASE_ADDRESS + I2C_REG_CR1,
        expected);

    uint32_t actual = wishbone_read(
        PN_CFG_I2C_BASE_ADDRESS + I2C_REG_CR1);

    PN_ASSERTM(actual == expected, "Write or read failed");
    PN_INFO("m_wb_tb: test passed");
}

void test_byte_select_write()
{
    trigger_reset();

    PN_INFO("m_wb_tb: run test test_byte_select_write() ...");

    uint32_t inital = 0xdeadbeef;
    for(int i = 0; i < (1 << 4); ++i)
    {
        trigger_reset();

        uint32_t expected = 0;
        wishbone_write(
            PN_CFG_I2C_BASE_ADDRESS + I2C_REG_CR1,
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
            PN_CFG_I2C_BASE_ADDRESS + I2C_REG_CR1);

        PN_ASSERTF(actual == expected,
            ("Byte select write failed with byte select: 0x%x.\n"
             "Expected: 0x%08x, Actual: 0x%08x",
                i,
                expected,
                actual));
    }

    PN_INFO("m_wb_tb: test passed");
}

void test_byte_select_read()
{
    trigger_reset();

    PN_INFO("m_i2c_wishbone_tb: run test test_byte_select_read() ...");

    uint32_t inital = 0xdeadbeef;
    wishbone_write(
        PN_CFG_I2C_BASE_ADDRESS + I2C_REG_CR1,
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
            PN_CFG_I2C_BASE_ADDRESS + I2C_REG_CR1,
            i);

        PN_ASSERTF(actual == expected,
            ("Byte select read failed with byte select: 0x%x.\n"
             "Expected: 0x%08x, Actual: 0x%08x",
                i,
                expected,
                actual));
    }

    PN_INFO("m_i2c_wishbone_tb: test passed");
}

void test_reset_defaults()
{
    trigger_reset();
    PN_INFO("m_i2c_wishbone_tb: run test_reset_defaults() -- check defaults...");

    uint32_t base = PN_CFG_I2C_BASE_ADDRESS;
    PN_ASSERTM(wishbone_read(base + I2C_REG_CR1) == 0, "CR1 != 0 after reset");
    PN_ASSERTM(wishbone_read(base + I2C_REG_CR2) == 0, "CR2 != 0 after reset");
    PN_ASSERTM(wishbone_read(base + I2C_REG_OAR1) == 0, "OAR1 != 0 after reset");
    PN_ASSERTM(wishbone_read(base + I2C_REG_OAR2) == 0, "OAR2 != 0 after reset");
    PN_ASSERTM(wishbone_read(base + I2C_REG_DR) == 0, "DR != 0 after reset");
    PN_ASSERTM(wishbone_read(base + I2C_REG_SR1) == 0, "SR1 != 0 after reset");
    PN_ASSERTM(wishbone_read(base + I2C_REG_SR2) == 0, "SR2 != 0 after reset");
    PN_ASSERTM(wishbone_read(base + I2C_REG_CCR) == 0, "CCR != 0 after reset");
    PN_ASSERTM(wishbone_read(base + I2C_REG_TRISE) == 0x02, "TRISE != 0x02 after reset");
    PN_ASSERTM(wishbone_read(base + I2C_REG_FLTR) == 0, "FLTR != 0 after reset");

    PN_INFO("m_i2c_wishbone_tb: test_reset_defaults (check defaults) passed");
}

void test_sr2_read_only()
{
    trigger_reset();
    PN_INFO("m_i2c_wishbone_tb: run test_sr2_read_only() -- SR2 write has to be ignored ...");

    uint32_t base = PN_CFG_I2C_BASE_ADDRESS;
    uint32_t addr = base + I2C_REG_SR2;

    uint32_t before = wishbone_read(addr);
    wishbone_write(addr, 0xFFFFFFFFu);
    uint32_t after = wishbone_read(addr);

    PN_ASSERTM(before == after, "SR2 write should be ignored (RO)");
    PN_INFO("m_i2c_wishbone_tb: test_sr2_read_only (SR2 write has to be ignored) passed");
}

void test_dr_lsb_only()
{
    trigger_reset();
    PN_INFO("m_i2c_wishbone_tb: run test_dr_lsb_only() -- DR LSB write/read ...");

    uint32_t base = PN_CFG_I2C_BASE_ADDRESS;
    uint32_t addr = base + I2C_REG_DR;

    wishbone_write(addr, 0xA5A5A5A5u, 0xF);
    uint32_t rd = wishbone_read(addr);

    PN_ASSERTF(rd == 0x000000A5u,
        ("DR LSB-only failed. Expected 0x000000A5, Actual 0x%08x", rd));
    PN_INFO("m_i2c_wishbone_tb: test_dr_lsb_only (DR LSB write/read) passed");
}

// CR2[FREQ(5:0)]
void test_cr2_freq_field()
{
    trigger_reset();
    PN_INFO("m_i2c_wishbone_tb: run test_cr2_freq_field() -- CR2[FREQ(5:0)] write/read ...");

    uint32_t base = PN_CFG_I2C_BASE_ADDRESS;
    uint32_t addr = base + I2C_REG_CR2;
    const uint32_t FREQ_MASK = I2C_CR2_FREQ_MASK; // 0x3F

    for(uint32_t mhz : {8u, 16u, 36u, 42u})
    {
        // RMW: Feld löschen, neuen Wert setzen
        wishbone_rmw(addr, (mhz & FREQ_MASK), FREQ_MASK);
        uint32_t rd = wishbone_read(addr);

        PN_ASSERTF((rd & FREQ_MASK) == (mhz & FREQ_MASK),
            ("CR2[FREQ] write/read failed. Wrote %u, Read 0x%08x", mhz, rd));
    }

    PN_INFO("m_i2c_wishbone_tb: test_cr2_freq_field (CR2[FREQ(5:0)] write/read) passed");
}

// CR1 Bit-Set/Clear: START/STOP/ACK via RMW
void test_cr1_bit_ops()
{
    trigger_reset();
    PN_INFO("m_i2c_wishbone_tb: run test_cr1_bit_ops() -- CR1 Bit-Set/Clear ...");

    uint32_t base = PN_CFG_I2C_BASE_ADDRESS;
    uint32_t addr = base + I2C_REG_CR1;

    wishbone_rmw(addr, I2C_CR1_START, 0);
    PN_ASSERTM((wishbone_read(addr) & I2C_CR1_START) != 0, "CR1.START not set");

    wishbone_rmw(addr, 0, I2C_CR1_ACK);
    PN_ASSERTM((wishbone_read(addr) & I2C_CR1_ACK) == 0, "CR1.ACK not cleared");

    wishbone_rmw(addr, I2C_CR1_STOP, 0);
    PN_ASSERTM((wishbone_read(addr) & I2C_CR1_STOP) != 0, "CR1.STOP not set");

    wishbone_rmw(addr, 0, I2C_CR1_STOP);
    PN_ASSERTM((wishbone_read(addr) & I2C_CR1_STOP) == 0, "CR1.STOP not cleared");

    PN_INFO("m_i2c_wishbone_tb: test_cr1_bit_ops (CR1 Bit-Set/Clear) passed");
}

void test_clearing_cr1_start_from_controller()
{
    trigger_reset();
    PN_INFO("m_i2c_wishbone_tb: run test_clearing_cr1_start_from_controller() -- CR1.START Clear from controller ...");

    uint32_t base = PN_CFG_I2C_BASE_ADDRESS;
    uint32_t addr = base + I2C_REG_CR1;

    wishbone_rmw(addr, I2C_CR1_START, 0);
    clear_cr1_start_sig.write(true);
    run_cycle();

    PN_ASSERTM((wishbone_read(addr) & I2C_CR1_START) == 0, "CR1.START not cleared");

    PN_INFO("m_i2c_wishbone_tb: test_clearing_cr1_start_from_controller() passed");
}

void test_cpu_read_sequences()
{

    trigger_reset();
    PN_INFO("m_i2c_wishbone_tb: run test_cpu_read_sequences() -- sequential SR1->SR2 and SR1->DR tests ...");

    uint32_t base = PN_CFG_I2C_BASE_ADDRESS;

    // --- 1. Test: SR1 -> SR2 ---
    wishbone_read(base + I2C_REG_SR1);
    wishbone_read(base + I2C_REG_SR2);

    run_cycle();

    // --- 2. Test: SR1 -> DR ---
    trigger_reset();
    wishbone_read(base + I2C_REG_SR1);
    wishbone_write(base + I2C_REG_DR, 0xAA);

    PN_INFO("m_i2c_wishbone_tb: run test_cpu_read_sequences() finished");
}

int sc_main(int argc, char** argv)
{

    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("m_i2c_wishbone_tb");

    // Initialize the Design under Testing (DUT)
    m_i2c_wishbone i_dut{"i_dut", PN_CFG_I2C_BASE_ADDRESS};

    // connect the wishbone signals
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

    // control / status connections
    i_dut.clear_regs_in(clear_regs_sig);

    i_dut.cr1_enable_out(cr1_enable_sig);
    i_dut.cr1_start_out(cr1_start_sig);
    i_dut.cr1_stop_out(cr1_stop_sig);
    i_dut.cr1_ack_out(cr1_ack_sig);
    i_dut.clear_cr1_start_in(clear_cr1_start_sig);
    i_dut.clear_cr1_stop_in(clear_cr1_stop_sig);

    i_dut.sr1_start_bit_in(sr1_start_bit_sig);
    i_dut.sr1_addr_bit_in(sr1_addr_bit_sig);
    i_dut.sr1_btf_in(sr1_btf_sig);
    i_dut.sr1_txe_in(sr1_txe_sig);
    i_dut.sr1_af_in(sr1_af_sig);
    i_dut.sr1_rxne_in(sr1_rxne_sig);

    i_dut.sr2_msl_in(sr2_msl_sig);
    i_dut.sr2_busy_in(sr2_busy_sig);
    i_dut.sr2_tra_in(sr2_tra_sig);

    i_dut.CPU_read_SR1_out(CPU_read_SR1_sig);
    i_dut.CPU_read_SR2_out(CPU_read_SR2_sig);
    i_dut.CPU_read_DR_out(CPU_read_DR_sig);
    i_dut.CPU_write_DR_out(CPU_write_DR_sig);

    i_dut.shift_data_in(shift_data_sig);
    i_dut.enable_dr_write_in(enable_dr_write_sig);
    i_dut.dr_out(dr_sig);
    i_dut.tx_data_out(tx_data_sig);

    i_dut.ccr_out(ccr_sig);
    i_dut.trise_out(trise_sig);

    // Traces of signals here
    i_dut.pn_trace(tf, pn_cfg_vcd_level); // trace signals of DUT

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    trigger_reset();
    test_reset_defaults();
    test_write_read();
    test_byte_select_write();
    test_byte_select_read();

    test_sr2_read_only();
    test_dr_lsb_only();
    test_cr2_freq_field();
    test_cr1_bit_ops();

    // Test if clear_cr1_start_in writes into CR1 register
    test_clearing_cr1_start_from_controller();
    // Test CPU read and write sequences
    test_cpu_read_sequences();

    run_cycle(); // end with a wait

    PN_END_TRACE();
    cout << "\n\t\t*****wb_tb Simulation complete*****" << endl;

    return 0;
}
