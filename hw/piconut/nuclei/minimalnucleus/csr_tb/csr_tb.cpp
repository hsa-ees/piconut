/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Johannes Hofmann <johannes.hofmann1@tha.de>
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

#include "csr.h"

#include <base.h>
#include <piconut-config.h>

#include <systemc.h>

#include <cstdint>

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

// csr bus
sc_signal<bool> PN_NAME(csr_bus_read_en);
sc_signal<bool> PN_NAME(csr_bus_write_en);
sc_signal<sc_uint<CFG_CSR_BUS_ADR_WIDTH>> PN_NAME(csr_bus_adr);
sc_signal<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(csr_bus_wdata);
sc_signal<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(csr_bus_rdata);

// status signals
sc_signal<sc_uint<32>> PN_NAME(pc);
sc_signal<bool> PN_NAME(debug_level_enter_ebreak);
sc_signal<bool> PN_NAME(debug_level_enter_haltrequest);
sc_signal<bool> PN_NAME(debug_level_enter_step);
sc_signal<bool> PN_NAME(debug_level_leave);

// control signals
sc_signal<sc_uint<32>> PN_NAME(dpc);
sc_signal<bool> PN_NAME(debug_level_enter);
sc_signal<bool> PN_NAME(debug_step);

///////////////// Helpers /////////////////
void run_cycle(int cycles = 1);
void run_reset();

sc_uint<CFG_CSR_BUS_DATA_WIDTH> csr_read(sc_uint<CFG_CSR_BUS_ADR_WIDTH> _adr);
void csr_write(sc_uint<CFG_CSR_BUS_ADR_WIDTH> _adr, sc_uint<CFG_CSR_BUS_DATA_WIDTH> _data);

///////////////// Tests /////////////////
// mstatus
void test_mstatus_initilized();
void test_mstatus_access_readwrite();

// misa
void test_misa_initialized();
void test_misa_access_readonly();

// dcsr
void test_dcsr_initialized();
void test_dcsr_debug_level_entering_changes_cause_field();
void test_debug_step_signal_when_step_is_set();

// dpc
void test_dpc_initialized();
void test_dpc_saves_pc_only_when_entering_debug_level();

// dscratch0
void test_dscratch0_initilized();
void test_dscratch0_access_readwrite();

// dscratch1
void test_dscratch1_initilized();
void test_dscratch1_access_readwrite();

// general
void test_read_write_simultanous();

int sc_main(int argc, char** argv)
{
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("csr_tb");

    m_csr dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.clk(clk);
    dut_inst.reset(reset);

    // Csr bus
    dut_inst.csr_bus_read_en_in(csr_bus_read_en);
    dut_inst.csr_bus_write_en_in(csr_bus_write_en);
    dut_inst.csr_bus_adr_in(csr_bus_adr);
    dut_inst.csr_bus_wdata_in(csr_bus_wdata);
    dut_inst.csr_bus_rdata_out(csr_bus_rdata);

    // Status signals
    dut_inst.pc_in(pc);
    dut_inst.debug_level_enter_ebreak_in(debug_level_enter_ebreak);
    dut_inst.debug_level_enter_haltrequest_in(debug_level_enter_haltrequest);
    dut_inst.debug_level_enter_step_in(debug_level_enter_step);
    dut_inst.debug_level_leave_in(debug_level_leave);

    // Control signals
    dut_inst.dpc_out(dpc);
    dut_inst.debug_level_enter_out(debug_level_enter);
    dut_inst.debug_step_out(debug_step);

    dut_inst.Trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT

    sc_start(SC_ZERO_TIME); // start simulation

    cout << "\n\t\t*****Simulation started*****" << endl;

    run_reset();

    test_mstatus_initilized();
    test_mstatus_access_readwrite();
    test_misa_initialized();
    test_misa_access_readonly();

    test_dcsr_initialized();
    test_dcsr_debug_level_entering_changes_cause_field();
    test_debug_step_signal_when_step_is_set();

    test_dpc_initialized();
    test_dpc_saves_pc_only_when_entering_debug_level();
    test_dscratch0_initilized();
    test_dscratch0_access_readwrite();
    test_dscratch1_initilized();
    test_dscratch1_access_readwrite();
    test_read_write_simultanous();

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}

///////////////// Helpers /////////////////
void run_cycle(int cycles)
{
    for(int i = 0; i < cycles; i++)
    {
        clk = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

void run_reset()
{
    run_cycle();
    reset = 1;
    run_cycle();
    reset = 0;
    run_cycle();
}

sc_uint<CFG_CSR_BUS_DATA_WIDTH> csr_read(sc_uint<CFG_CSR_BUS_ADR_WIDTH> _adr)
{
    csr_bus_read_en = 1;
    csr_bus_write_en = 0;
    csr_bus_adr = _adr;

    run_cycle();

    csr_bus_read_en = 0;
    csr_bus_write_en = 0;
    csr_bus_adr = 0x0;

    return csr_bus_rdata.read();
}

void csr_write(
    sc_uint<CFG_CSR_BUS_ADR_WIDTH> _adr,
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> _data)
{
    csr_bus_read_en = 0;
    csr_bus_write_en = 1;
    csr_bus_adr = _adr;
    csr_bus_wdata = _data;

    run_cycle();

    csr_bus_read_en = 0;
    csr_bus_write_en = 0;
    csr_bus_adr = 0x0;
    csr_bus_wdata = 0x0;
}

///////////////// Tests /////////////////
void test_mstatus_initilized()
{
    run_reset();
    PN_INFO("csr_tb: run test test_mstatus_initilized() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = (1 << 17);
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_mstatus);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_mstatus_access_readwrite()
{
    run_reset();
    PN_INFO("csr_tb: run test test_mstatus_access_readwrite() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = (1 << 17);
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_mstatus);

    PN_ASSERT(expected == actual);

    expected = 0xaffeaffe;
    csr_write(e_csr_address::csr_adr_mstatus, expected);
    actual = csr_read(e_csr_address::csr_adr_mstatus);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_misa_initialized()
{
    run_reset();
    PN_INFO("csr_tb: run test test_misa_initilized() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x40000100;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_misa);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_misa_access_readonly()
{
    run_reset();
    PN_INFO("csr_tb: run test test_misa_access_readonly() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x40000100;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> dummy_value = 0xaffeaffe;

    csr_write(e_csr_address::csr_adr_misa, dummy_value);
    actual = csr_read(e_csr_address::csr_adr_misa);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_dcsr_initialized()
{
    run_reset();
    PN_INFO("csr_tb: run test test_dcsr_initilized() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x4000b003;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_dcsr);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_dcsr_debug_level_entering_changes_cause_field()
{
    run_reset();
    PN_INFO("csr_tb: run test test_dcsr_debug_level_entering_changes_cause_field() ...");

    auto reset_debug_level_enter = []() {
        debug_level_enter_ebreak = 0;
        debug_level_enter_haltrequest = 0;
        debug_level_enter_step = 0;
    };

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = 0;

    debug_level_enter_ebreak = 1;
    run_cycle();
    reset_debug_level_enter();
    actual = csr_read(e_csr_address::csr_adr_dcsr);
    PN_ASSERT(actual.range(8, 6) == 1);

    debug_level_enter_haltrequest = 1;
    run_cycle();
    reset_debug_level_enter();
    actual = csr_read(e_csr_address::csr_adr_dcsr);
    PN_ASSERT(actual.range(8, 6) == 3);

    debug_level_enter_step = 1;
    run_cycle();
    reset_debug_level_enter();
    actual = csr_read(e_csr_address::csr_adr_dcsr);
    PN_ASSERT(actual.range(8, 6) == 4);

    debug_level_enter_ebreak = 1;
    debug_level_enter_haltrequest = 1;
    debug_level_enter_step = 1;
    run_cycle();
    reset_debug_level_enter();
    actual = csr_read(e_csr_address::csr_adr_dcsr);
    PN_ASSERT(actual.range(8, 6) == 1);

    PN_INFO("csr_tb: passed.");
}

void test_debug_step_signal_when_step_is_set()
{
    run_reset();
    PN_INFO("csr_tb: test_debug_step_signal_when_step_is_set() ...");

    debug_level_enter_haltrequest = 1;
    run_cycle();
    debug_level_enter_haltrequest = 0;

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> dcsr_step_enabled = 0x4000b007;
    csr_write(e_csr_address::csr_adr_dcsr, dcsr_step_enabled);

    debug_level_leave = 1;
    PN_ASSERT(debug_step.read() == 0);
    run_cycle();

    debug_level_leave = 0;
    PN_ASSERT(debug_step.read() == 1);
    run_cycle();

    PN_INFO("csr_tb: passed.");
}

void test_dpc_initialized()
{
    run_reset();
    PN_INFO("csr_tb: run test test_dpc_initialized() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_dpc);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_dpc_saves_pc_only_when_entering_debug_level()
{
    run_reset();
    PN_INFO("csr_tb: run test test_dpc_saves_pc_only_when_entering_debug_level() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual;

    pc = 0xaffeaffe;
    debug_level_enter_ebreak = 1;
    run_cycle();
    actual = csr_read(e_csr_address::csr_adr_dpc);
    PN_ASSERT(actual == 0xaffeaffe);
    PN_ASSERT(dpc.read() == 0xaffeaffe);

    debug_level_enter_ebreak = 0;
    run_cycle();
    actual = csr_read(e_csr_address::csr_adr_dpc);
    PN_ASSERT(actual == 0xaffeaffe);

    pc = 0xdeadbeef;
    debug_level_enter_ebreak = 1;
    run_cycle();
    actual = csr_read(e_csr_address::csr_adr_dpc);
    PN_ASSERT(actual == 0xaffeaffe);

    debug_level_enter_ebreak = 0;
    debug_level_leave = 1;
    run_cycle();

    debug_level_leave = 0;
    run_cycle();

    PN_INFO("csr_tb: passed.");
}

void test_dscratch0_initilized()
{
    run_reset();
    PN_INFO("csr_tb: run test test_dscratch0_initilized() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_dscratch0);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_dscratch0_access_readwrite()
{
    run_reset();
    PN_INFO("csr_tb: run test test_dscratch0_access_readwrite() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_dscratch0);

    PN_ASSERT(expected == actual);

    expected = 0xaffeaffe;
    csr_write(e_csr_address::csr_adr_dscratch0, expected);
    actual = csr_read(e_csr_address::csr_adr_dscratch0);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_dscratch1_initilized()
{
    run_reset();
    PN_INFO("csr_tb: run test test_dscratch1_initilized() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_dscratch1);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_dscratch1_access_readwrite()
{
    run_reset();
    PN_INFO("csr_tb: run test test_dscratch1_access_readwrite() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_dscratch1);

    PN_ASSERT(expected == actual);

    expected = 0xaffeaffe;
    csr_write(e_csr_address::csr_adr_dscratch1, expected);
    actual = csr_read(e_csr_address::csr_adr_dscratch1);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_read_write_simultanous()
{
    run_reset();
    PN_INFO("csr_tb: run test test_read_write_simultanous() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_dscratch0);

    PN_ASSERT(expected == actual);

    csr_write(e_csr_address::csr_adr_dscratch0, 0xdeadbeef);

    expected = 0xaffeaffe;

    csr_bus_read_en = 1;
    csr_bus_write_en = 1;
    csr_bus_adr = e_csr_address::csr_adr_dscratch0;
    csr_bus_wdata = expected;
    run_cycle();
    PN_ASSERT(0xdeadbeef == csr_bus_rdata.read());

    PN_ASSERT(expected == csr_read(e_csr_address::csr_adr_dscratch0));

    csr_bus_read_en = 0;
    csr_bus_write_en = 0;
    csr_bus_adr = 0x0;
    csr_bus_wdata = 0x0;

    PN_INFO("csr_tb: passed.");
}