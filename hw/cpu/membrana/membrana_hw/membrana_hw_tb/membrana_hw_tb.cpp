/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Claus Janicher <claus.janicher@tha.de>
                     2025 Niklas Sirch <niklas.sirch1@tha.de>
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

/* TBD+: Eliminate the super-class 'c_hw_membrana_test' to enable a unified
 *       testbench for simulation and synthesis. In general, a testbench must
 *       validate the _unmodified_ design under test!
 */

#ifdef __SYNTHESIS__

// Mininmal main program just for synthesis ...
int sc_main(int argc, char** argv)
{

    m_membrana_hw i_dut{"i_dut"}; // design name needed by 'svc_tool'

    sc_signal<bool> PN_NAME(clk);
    sc_signal<bool> PN_NAME(reset);

    sc_signal<bool> PN_NAME(ip_stb);          // strobe
    sc_signal<sc_uint<4>> PN_NAME(ip_bsel);   // byte select bsel[n]? 4 == 32bit, 8 == 64bit, 16 == 128bit
    sc_signal<sc_uint<32>> PN_NAME(ip_adr);   // address <=30
    sc_signal<bool> PN_NAME(ip_ack);          // acknowledge
    sc_signal<sc_uint<32>> PN_NAME(ip_rdata); // hands data to the core 32/64/128 bit width

    sc_signal<bool> PN_NAME(dp_stb);          // strobe
    sc_signal<bool> PN_NAME(dp_we);           // write enable DPort only?
    sc_signal<sc_uint<4>> PN_NAME(dp_bsel);   // byte select bsel[n]? 4 == 32bit, 8 == 64bit, 16 == 128bit
    sc_signal<sc_uint<32>> PN_NAME(dp_adr);   // address <=30
    sc_signal<sc_uint<32>> PN_NAME(dp_wdata); // write data from core to membrana 32/64/128 bit width DPort only?
    sc_signal<bool> PN_NAME(dp_ack);          // acknowledge
    sc_signal<sc_uint<32>> PN_NAME(dp_rdata); // hands data to the core 32/64/128 bit width
    sc_signal<bool> PN_NAME(dp_lr_sc);        // lr/sc reservation for cores
    sc_signal<bool> PN_NAME(dp_amo);          // lr/sc reservation for cores

    sc_signal<pn_wb_adr_t> PN_NAME(wb_adr_o);
    sc_signal<pn_wb_dat_t> PN_NAME(wb_dat_o);
    sc_signal<pn_wb_dat_t> PN_NAME(wb_dat_i);
    sc_signal<bool> PN_NAME(wb_we_o);
    sc_signal<bool> PN_NAME(wb_stb_o);
    sc_signal<bool> PN_NAME(wb_ack_i);
    sc_signal<bool> PN_NAME(wb_cyc_o);
    sc_signal<bool> PN_NAME(wb_rty_i);
    sc_signal<bool> PN_NAME(wb_err_i);
    sc_signal<pn_wb_sel_t> PN_NAME(wb_sel_o);

    i_dut.clk(clk);
    i_dut.reset(reset);
    i_dut.wb_master.adr_o(wb_adr_o);
    i_dut.wb_master.dat_i(wb_dat_i);
    i_dut.wb_master.dat_o(wb_dat_o);
    i_dut.wb_master.we_o(wb_we_o);
    i_dut.wb_master.stb_o(wb_stb_o);
    i_dut.wb_master.ack_i(wb_ack_i);
    i_dut.wb_master.cyc_o(wb_cyc_o);
    i_dut.wb_master.sel_o(wb_sel_o);
    i_dut.wb_master.rty_i(wb_rty_i);
    i_dut.wb_master.err_i(wb_err_i);
    i_dut.ip_stb[0](ip_stb);
    i_dut.ip_bsel[0](ip_bsel);
    i_dut.ip_adr[0](ip_adr);
    i_dut.ip_ack[0](ip_ack);
    i_dut.ip_rdata[0](ip_rdata);
    i_dut.dp_stb[0](dp_stb);
    i_dut.dp_we[0](dp_we);
    i_dut.dp_bsel[0](dp_bsel);
    i_dut.dp_adr[0](dp_adr);
    i_dut.dp_wdata[0](dp_wdata);
    i_dut.dp_ack[0](dp_ack);
    i_dut.dp_rdata[0](dp_rdata);
    i_dut.dp_lr_sc[0](dp_lr_sc);
    i_dut.dp_amo[0](dp_amo);

    sc_start(SC_ZERO_TIME); // start simulation
    return 0;
}

#else // __SYNTHESIS__

#include <stdint.h>
#include <systemc.h>
#include <piconut.h>

#include "../membrana_hw.h"
#include <vector>
#include <functional>

#define PERIOD_NS 10.0

// defines for full mem test
// insert memory size correctly to avoid core dumps
#define full_mem_test 1
#define blockram_mem_test 1
#define mem_size 0x000001FF

/**
 * test class with public access to all members of wb_membrana
 */
class c_hw_membrana_test : public m_membrana_hw
{
public:
    c_hw_membrana_test(const sc_module_name& nm)
        : m_membrana_hw(nm)
    {
    }

    // expose protected members for testing blockram interaction
    using m_membrana_hw::wea;
    using m_membrana_hw::web;
    using m_membrana_hw::ena;
    using m_membrana_hw::enb;
    using m_membrana_hw::addra;
    using m_membrana_hw::addrb;
    using m_membrana_hw::dia;
    using m_membrana_hw::dib;
    using m_membrana_hw::doa;
    using m_membrana_hw::dob;
};

c_hw_membrana_test* i_dut;

// initialize TB signals

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

sc_signal<bool> PN_NAME(ip_stb);          // strobe
sc_signal<sc_uint<4>> PN_NAME(ip_bsel);   // byte select bsel[n]? 4 == 32bit, 8 == 64bit, 16 == 128bit
sc_signal<sc_uint<32>> PN_NAME(ip_adr);   // address <=30
sc_signal<bool> PN_NAME(ip_ack);          // acknowledge
sc_signal<sc_uint<32>> PN_NAME(ip_rdata); // hands data to the core 32/64/128 bit width
// signals DataPort
sc_signal<bool> PN_NAME(dp_stb);          // strobe
sc_signal<bool> PN_NAME(dp_we);           // write enable DPort only?
sc_signal<sc_uint<4>> PN_NAME(dp_bsel);   // byte select bsel[n]? 4 == 32bit, 8 == 64bit, 16 == 128bit
sc_signal<sc_uint<32>> PN_NAME(dp_adr);   // address <=30
sc_signal<sc_uint<32>> PN_NAME(dp_wdata); // write data from core to membrana 32/64/128 bit width DPort only?
sc_signal<bool> PN_NAME(dp_ack);          // acknowledge
sc_signal<sc_uint<32>> PN_NAME(dp_rdata); // hands data to the core 32/64/128 bit width
sc_signal<bool> PN_NAME(dp_lr_sc);        // lr/sc reservation for cores
sc_signal<bool> PN_NAME(dp_amo);          // lr/sc reservation for cores

// signals for wishbone bus
sc_signal<pn_wb_dat_t> PN_NAME(wb_adr_o);
sc_signal<pn_wb_dat_t> PN_NAME(wb_dat_o);
sc_signal<pn_wb_dat_t> PN_NAME(wb_dat_i);
sc_signal<bool> PN_NAME(wb_we_o);
sc_signal<bool> PN_NAME(wb_stb_o);
sc_signal<bool> PN_NAME(wb_ack_i);
sc_signal<bool> PN_NAME(wb_cyc_o);
sc_signal<pn_wb_sel_t> PN_NAME(wb_sel_o);
sc_signal<bool> PN_NAME(wb_rty_i);
sc_signal<bool> PN_NAME(wb_err_i);

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

void assert_idle()
{
    PN_ASSERT(i_dut->wea.read() == 0);
    PN_ASSERT(i_dut->web.read() == 0);
    PN_ASSERT(i_dut->ena.read() == 0);
    PN_ASSERT(i_dut->enb.read() == 0);
    PN_ASSERT(i_dut->addra.read() == 0);
    PN_ASSERT(i_dut->addrb.read() == 0);
    PN_ASSERT(i_dut->dia.read() == 0);
    PN_ASSERT(i_dut->dib.read() == 0);
    PN_ASSERT(wb_adr_o.read() == 0);
    PN_ASSERT(wb_cyc_o.read() == 0);
    PN_ASSERT(wb_dat_o.read() == 0);
    PN_ASSERT(wb_stb_o.read() == 0);
    PN_ASSERT(dp_ack.read() == 0);
    PN_ASSERT(dp_rdata.read() == 0);
    PN_ASSERT(ip_ack.read() == 0);
    PN_ASSERT(ip_rdata.read() == 0);
}

void test_reset()
{
    PN_INFO("Test: Performing reset sequence");
    reset = 1;
    PN_INFO("Reset was set");
    run_cycle(1);
    reset = 0;
    PN_INFO("Reset no longer set");
    run_cycle(1);
    assert_idle();
}

void test_dport_write(uint32_t addr = 0x10020000)
{
    PN_INFO("Test: DPort Write");
    dp_stb = 1;
    dp_we = 1;
    dp_adr = addr;
    dp_wdata = 0xCAFECAFE;
    dp_bsel = 0xF;
    run_cycle(1);
    // write check
    PN_INFO("Test for check");
    run_cycle(1);
    PN_INFO("Test for WRITE1");
    PN_ASSERT(i_dut->addra.read() == (addr >> 2));
    PN_ASSERT(i_dut->wea.read() == 0xF);
    PN_ASSERT(i_dut->dia.read() == 0xCAFECAFE);
    PN_ASSERT(i_dut->ena.read() == 0x1);
    PN_ASSERT(wb_cyc_o.read() == 0);
    run_cycle(1);
    PN_ASSERT(dp_ack.read() == 1);
    PN_ASSERT(i_dut->ena.read() == 0);
    run_cycle(1);
    PN_ASSERT(dp_ack.read() == 0);
    dp_stb = 0;
    dp_we = 0;
    dp_adr = 0;
    dp_wdata = 0;
    dp_bsel = 0;
    run_cycle(1);
}

void test_dport_read(uint32_t addr = 0x10020000)
{
    PN_INFO("Test: DPort Read");
    dp_stb = 1;
    dp_adr = addr;
    dp_bsel = 0xF;
    run_cycle(1);
    // read check
    PN_ASSERT(i_dut->addra.read() == (addr >> 2));
    PN_ASSERT(i_dut->ena.read() == 0x1);
    run_cycle(1);
    PN_ASSERT(i_dut->addra.read() == (addr >> 2));
    PN_ASSERT(i_dut->ena.read() == 0x1);
    PN_ASSERT(dp_ack.read() == 1);
    PN_ASSERT(dp_rdata.read() == 0xCAFECAFE);
    PN_ASSERT(wb_cyc_o.read() == 0);
    run_cycle(1);
    PN_ASSERT(dp_ack.read() == 0);
    dp_stb = 0;
    dp_adr = 0;
    dp_bsel = 0;
}

void test_lrsc()
{
    PN_INFO("Test: LR/SC (Load Reserved / Store Conditional)");
    // Set up address and data
    sc_uint<32> lrsc_addr = 0x10001234;
    sc_uint<32> lrsc_data = 0xABCD1234;
    // 1. Perform LR (load reserved)
    dp_adr = lrsc_addr;
    dp_stb = 1;
    dp_we = 0;
    dp_lr_sc = 1;
    run_cycle(1);
    dp_stb = 0;
    run_cycle(3);
    // 2. Perform SC (store conditional) - should succeed
    dp_adr = lrsc_addr;
    dp_wdata = lrsc_data;
    dp_stb = 1;
    dp_we = 1;
    dp_lr_sc = 1;
    dp_bsel = 0xF;
    run_cycle(1);
    dp_stb = 0;
    dp_we = 0;
    run_cycle(2);
    PN_ASSERT(dp_rdata.read() == 0x0);
    run_cycle(1);

    run_cycle(5);
    lrsc_addr = 0x1000FFFF;
    lrsc_data = 0x12345678;

    // 1. Perform LR (load reserved)
    dp_adr = lrsc_addr;
    dp_stb = 1;
    dp_we = 0;
    dp_lr_sc = 1;
    run_cycle(1);
    dp_stb = 0;
    run_cycle(3);
    // 2. Perform normal store
    dp_adr = lrsc_addr;
    dp_wdata = lrsc_data;
    dp_stb = 1;
    dp_we = 1;
    dp_lr_sc = 0; // Not a SC operation
    dp_bsel = 0xF;
    run_cycle(1);
    dp_stb = 0;
    dp_we = 0;
    run_cycle(3);
    // 3. Perform SC - should fail
    dp_adr = lrsc_addr;
    dp_wdata = 0xDEADBEEF;
    dp_stb = 1;
    dp_we = 1;
    dp_lr_sc = 1;
    dp_bsel = 0xF;
    run_cycle(1);
    dp_stb = 0;
    dp_we = 0;
    run_cycle(1);
    PN_ASSERT(dp_rdata.read() == 0x1);
    run_cycle(1);
    // Reset signals
    dp_stb = 0;
    dp_we = 0;
    dp_adr = 0;
    dp_wdata = 0;
    dp_bsel = 0;
    dp_lr_sc = 0;
    dp_amo = 0;
    run_cycle(1);
}

void test_wishbone_write(uint32_t addr = 0x30000000, uint32_t data = 0xDEADBEEF)
{
    PN_INFO("Test: Wishbone Write");
    dp_stb = 1;
    dp_we = 1;
    dp_adr = addr;
    dp_wdata = data;
    dp_bsel = 0xF;
    run_cycle(1);
    // write check
    run_cycle(1);
    // wb_write_1
    PN_ASSERT(wb_cyc_o.read() == 1);
    PN_ASSERT(wb_we_o.read() == 1);
    PN_ASSERT(wb_stb_o.read() == 1);
    PN_ASSERT(wb_sel_o.read() == 0xF);
    PN_ASSERT(wb_adr_o.read() == addr);
    PN_ASSERT(wb_dat_o.read() == data);
    run_cycle(2);
    PN_ASSERT(wb_cyc_o.read() == 1);
    PN_ASSERT(wb_we_o.read() == 1);
    PN_ASSERT(wb_stb_o.read() == 1);
    PN_ASSERT(wb_sel_o.read() == 0xF);
    PN_ASSERT(wb_adr_o.read() == addr);
    PN_ASSERT(wb_dat_o.read() == data);
    run_cycle(1);
    // Simulate wishbone ack
    wb_ack_i = 1;
    run_cycle(1);
    PN_ASSERT(dp_ack.read() == 1);
    wb_ack_i = 0;
    run_cycle(1);
    PN_ASSERT(dp_ack.read() == 0);
    dp_stb = 0;
    dp_we = 0;
    dp_adr = 0;
    dp_wdata = 0;
    dp_bsel = 0;
    run_cycle(1);
}

void test_wishbone_read(uint32_t addr = 0x30000000, uint32_t wb_data = 0xBEEFCAFE)
{
    PN_INFO("Test: Wishbone Read");
    dp_stb = 1;
    dp_we = 0;
    dp_adr = addr;
    dp_bsel = 0xF;
    // Simulate wishbone data
    wb_dat_i = wb_data;
    run_cycle(1);
    // read check
    run_cycle(1);
    PN_ASSERT(wb_cyc_o.read() == 1);
    PN_ASSERT(wb_stb_o.read() == 1);
    PN_ASSERT(wb_adr_o.read() == addr);
    // Simulate wishbone ack
    wb_ack_i = 1;
    run_cycle(1);
    PN_ASSERT(dp_ack.read() == 1);
    PN_ASSERT(dp_rdata.read() == wb_data);
    wb_ack_i = 0;
    run_cycle(1);
    PN_ASSERT(dp_ack.read() == 0);
    dp_stb = 0;
    dp_adr = 0;
    dp_bsel = 0;
    run_cycle(1);
}

void test_wishbone_lrsc(uint32_t addr = 0x30000000, uint32_t wb_data = 0xBEEFCAFE)
{
    PN_INFO("Test: Wishbone LR/SC");
    PN_INFO("Test for successful LR/SC");
    // 1. Perform LR (load reserved) via wishbone
    dp_stb = 1;
    dp_we = 0;
    dp_adr = addr;
    dp_bsel = 0xF;
    dp_lr_sc = 1;
    wb_dat_i = wb_data;
    run_cycle(1);
    // read check
    run_cycle(1);
    // wb_read_lr
    run_cycle(1);
    PN_ASSERT(wb_cyc_o.read() == 1);
    PN_ASSERT(wb_stb_o.read() == 1);
    PN_ASSERT(wb_adr_o.read() == addr);
    // Simulate wishbone ack
    wb_ack_i = 1;
    run_cycle(1);
    PN_ASSERT(dp_ack.read() == 1);
    PN_ASSERT(dp_rdata.read() == wb_data);
    wb_ack_i = 0;
    run_cycle(1);
    PN_ASSERT(dp_ack.read() == 0);
    dp_stb = 0;
    dp_lr_sc = 0;
    run_cycle(2);

    // 2. Perform SC (store conditional) via wishbone - should succeed
    uint32_t lrsc_data = 0x12345678;
    dp_stb = 1;
    dp_we = 1;
    dp_adr = addr;
    dp_wdata = lrsc_data;
    dp_bsel = 0xF;
    dp_lr_sc = 1;
    run_cycle(1);
    // write check
    run_cycle(1);
    // write_sc_success
    run_cycle(1);
    PN_ASSERT(wb_cyc_o.read() == 1);
    PN_ASSERT(wb_we_o.read() == 1);
    PN_ASSERT(wb_stb_o.read() == 1);
    PN_ASSERT(wb_sel_o.read() == 0xF);
    PN_ASSERT(wb_adr_o.read() == addr);
    PN_ASSERT(wb_dat_o.read() == lrsc_data);
    run_cycle(2);
    wb_ack_i = 1;
    run_cycle(1);
    PN_ASSERT(dp_ack.read() == 1);
    PN_ASSERT(dp_rdata.read() == 0x0); // SC success
    wb_ack_i = 0;
    run_cycle(1);
    PN_ASSERT(dp_ack.read() == 0);
    dp_stb = 0;
    dp_we = 0;
    dp_lr_sc = 0;
    run_cycle(2);

    PN_INFO("Test for failing LR/SC");
    // 1. LR
    dp_stb = 1;
    dp_we = 0;
    dp_adr = addr;
    dp_bsel = 0xF;
    dp_lr_sc = 1;
    wb_dat_i = wb_data;
    run_cycle(1);
    // read check
    run_cycle(1);
    // wb_read_lr
    run_cycle(1);
    PN_ASSERT(wb_cyc_o.read() == 1);
    PN_ASSERT(wb_stb_o.read() == 1);
    PN_ASSERT(wb_adr_o.read() == addr);
    // Simulate wishbone ack
    wb_ack_i = 1;
    run_cycle(1);
    PN_ASSERT(dp_ack.read() == 1);
    PN_ASSERT(dp_rdata.read() == wb_data);
    wb_ack_i = 0;
    run_cycle(1);
    PN_ASSERT(dp_ack.read() == 0);
    dp_stb = 0;
    dp_lr_sc = 0;
    run_cycle(2);

    // 2. Normal Store
    dp_stb = 1;
    dp_we = 1;
    dp_adr = addr;
    dp_wdata = 0xDEADBEEF;
    dp_bsel = 0xF;
    dp_lr_sc = 0;
    run_cycle(1); // check
    run_cycle(1); // 1
    run_cycle(1); // 2
    wb_ack_i = 1;
    run_cycle(1);
    wb_ack_i = 0;
    dp_stb = 0;
    dp_we = 0;
    run_cycle(2);

    // 3. SC should fail
    dp_stb = 1;
    dp_we = 1;
    dp_adr = addr;
    dp_wdata = 0xCAFEBABE;
    dp_bsel = 0xF;
    dp_lr_sc = 1;
    run_cycle(1);
    run_cycle(1);
    PN_ASSERT(dp_rdata.read() == 0x1); // SC fail
    wb_ack_i = 0;
    dp_stb = 0;
    dp_we = 0;
    dp_lr_sc = 0;
    run_cycle(1);
}

int sc_main(int argc, char** argv)
{
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("membrana_hw_tb");

    i_dut = new c_hw_membrana_test("i_dut");
    i_dut->clk(clk);
    i_dut->reset(reset);
    i_dut->wb_master.adr_o(wb_adr_o);
    i_dut->wb_master.dat_i(wb_dat_i);
    i_dut->wb_master.dat_o(wb_dat_o);
    i_dut->wb_master.we_o(wb_we_o);
    i_dut->wb_master.stb_o(wb_stb_o);
    i_dut->wb_master.ack_i(wb_ack_i);
    i_dut->wb_master.cyc_o(wb_cyc_o);
    i_dut->wb_master.sel_o(wb_sel_o);
    i_dut->wb_master.rty_i(wb_rty_i);
    i_dut->wb_master.err_i(wb_err_i);
    i_dut->ip_stb[0](ip_stb);
    i_dut->ip_bsel[0](ip_bsel);
    i_dut->ip_adr[0](ip_adr);
    i_dut->ip_ack[0](ip_ack);
    i_dut->ip_rdata[0](ip_rdata);
    i_dut->dp_stb[0](dp_stb);
    i_dut->dp_we[0](dp_we);
    i_dut->dp_bsel[0](dp_bsel);
    i_dut->dp_adr[0](dp_adr);
    i_dut->dp_wdata[0](dp_wdata);
    i_dut->dp_ack[0](dp_ack);
    i_dut->dp_rdata[0](dp_rdata);
    i_dut->dp_lr_sc[0](dp_lr_sc);
    i_dut->dp_amo[0](dp_amo);

    i_dut->pn_trace(tf, pn_cfg_vcd_level);

    const std::vector<std::function<void()>> tests = {
        [] { test_dport_write(); },
        [] { test_dport_read(); },
        [] { test_reset(); },
        [] { test_lrsc(); },
        [] { test_wishbone_write(); },
        [] { test_wishbone_read(); },
        [] { test_wishbone_lrsc(); },
    };

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    run_cycle(1);

    assert_idle();

    for(const auto& test : tests)
    {
        test();
        assert_idle();
        run_cycle(5);
    }

    cout << "\n\t\t*****Simulation complete*****" << endl;
    return 0;
}

#endif // __SYNTHESIS__
