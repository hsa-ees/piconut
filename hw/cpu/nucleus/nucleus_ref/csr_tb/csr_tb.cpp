/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Johannes Hofmann <johannes.hofmann1@tha.de>
                     2025 Alexander Beck <alexander.beck1@tha.de>
                     2025 Christian Zellinger <Christian.Zellinger1@tha.de>
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
  (INCLUDING NEGLIGENCE OR OTHERWISE ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *************************************************************************/

#include "csr.h"

#include <piconut.h>
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

// interrupt outputs
sc_signal<bool> PN_NAME(mip_msip_out);
sc_signal<bool> PN_NAME(mie_msie_out);
sc_signal<bool> PN_NAME(mstatus_mie_out);
sc_signal<sc_uint<32>> PN_NAME(mtvec_trap_address_out);
sc_signal<bool> PN_NAME(mip_mtip_out);
sc_signal<bool> PN_NAME(mip_meip_out);
sc_signal<bool> PN_NAME(mie_mtie_out);
sc_signal<bool> PN_NAME(mie_meie_out);
sc_signal<bool> PN_NAME(interrupt_pending_out);

// write mode signal
sc_signal<sc_uint<2>> PN_NAME(write_mode_in);

// interrupt input signal
sc_signal<bool> PN_NAME(interrupt_in);

// mret signal
sc_signal<bool> PN_NAME(mret_in);

// mepc signal
sc_signal<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(mepc_out);

// Machine External Interrupt Pin input
sc_signal<bool> PN_NAME(meip_in);

// Machine Timer Interrupt Pin input
sc_signal<bool> PN_NAME(mtip_in);

// Machine Software Interrupt Pin input
sc_signal<bool> PN_NAME(msip_in);

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

// mie (Machine Interrupt Enable)
void test_mie_initialized();
void test_mie_access_readwrite();
void test_mie_outputs();

// mtvec (Machine Trap Vector)
void test_mtvec_initialized();
void test_mtvec_access_readwrite();
void test_mtvec_address_output();

// mepc (Machine Exception Program Counter)
void test_mepc_initialized();
void test_mepc_access_readwrite();

// mcause (Machine Cause Register)
void test_mcause_initialized();
void test_mcause_access_readwrite();

// mtval (Machine Trap Value)
void test_mtval_initialized();
void test_mtval_access_readwrite();

// mip (Machine Interrupt Pending)
void test_mip_initialized();
void test_mip_access_readwrite();
void test_mip_outputs();

// Interrupt functionality
void test_interrupt_pending_detection();

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

    // Connect interrupt outputs
    dut_inst.mip_msip_out(mip_msip_out);
    dut_inst.mie_msie_out(mie_msie_out);
    dut_inst.mstatus_mie_out(mstatus_mie_out);
    dut_inst.mtvec_trap_address_out(mtvec_trap_address_out);
    dut_inst.mip_mtip_out(mip_mtip_out);
    dut_inst.mip_meip_out(mip_meip_out);
    dut_inst.mie_mtie_out(mie_mtie_out);
    dut_inst.mie_meie_out(mie_meie_out);
    dut_inst.interrupt_pending_out(interrupt_pending_out);

    // Connect write mode signal
    dut_inst.write_mode_in(write_mode_in);

    // Connect interrupt input signal
    dut_inst.interrupt_in(interrupt_in);

    // Connect mret signal
    dut_inst.mret_in(mret_in);

    // Connect mepc_out signal
    dut_inst.mepc_out(mepc_out);

    // Connect Machine External Interrupt Pin input
    dut_inst.meip_in(meip_in);

    // Connect Machine Timer Interrupt Pin input
    dut_inst.mtip_in(mtip_in); // Bind the mtip_in signal to the DUT

    // Connect Machine Software Interrupt Pin input
    dut_inst.msip_in(msip_in);

    dut_inst.pn_trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT

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

    test_mie_initialized();
    test_mie_access_readwrite();
    test_mie_outputs();

    test_mtvec_initialized();
    test_mtvec_access_readwrite();
    test_mtvec_address_output();

    test_mepc_initialized();
    test_mepc_access_readwrite();

    test_mcause_initialized();
    test_mcause_access_readwrite();

    test_mtval_initialized();
    test_mtval_access_readwrite();

    test_mip_initialized();
    test_mip_access_readwrite();
    test_mip_outputs(); // TODO: fix

    test_interrupt_pending_detection();

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

    // 1. Read initial value of mstatus
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> initial = csr_read(e_csr_address::csr_adr_mstatus);

    // 2. Prepare a value that inverts all bits of the initial value
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> inverted = ~initial;

    // 3. Write the inverted value to mstatus
    csr_write(e_csr_address::csr_adr_mstatus, inverted);

    // 4. Read back after write
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> after_write = csr_read(e_csr_address::csr_adr_mstatus);

    // 5. Only the writable bits should have changed, all others must remain as before
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> writable_mask = 0x0000088E; // Only these bits are writable
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = (initial & ~writable_mask) | (inverted & writable_mask);

    PN_ASSERTM(
        after_write == expected,
        "Only the writable bits of mstatus must be changed by a write operation!");

    PN_INFO("csr_tb: passed.");
}

void test_misa_initialized()
{
    run_reset();
    PN_INFO("csr_tb: run test test_misa_initilized() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x40000101;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_misa);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_misa_access_readonly()
{
    run_reset();
    PN_INFO("csr_tb: run test test_misa_access_readonly() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x40000101;
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

void test_mie_initialized()
{
    run_reset();
    PN_INFO("csr_tb: run test test_mie_initialized() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_mie);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_mie_access_readwrite()
{
    run_reset();
    PN_INFO("csr_tb: run test test_mie_access_readwrite() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_mie);

    PN_ASSERT(expected == actual);

    // Set specific interrupt enable bits
    // Bit 3 = MSIE (Machine Software Interrupt Enable)
    // Bit 7 = MTIE (Machine Timer Interrupt Enable)
    // Bit 11 = MEIE (Machine External Interrupt Enable)
    expected = 0x888;
    csr_write(e_csr_address::csr_adr_mie, expected);
    actual = csr_read(e_csr_address::csr_adr_mie);

    PN_ASSERT(expected == actual);

    // // Check if output signals are correct
    // run_cycle();
    // PN_ASSERT(mie_msie_out.read() == 1);
    // PN_ASSERT(mie_mtie_out.read() == 1);
    // PN_ASSERT(mie_meie_out.read() == 1);

    // Test each bit separately
    expected = 0x008; // Just MSIE
    csr_write(e_csr_address::csr_adr_mie, expected);
    run_cycle(0);
    PN_ASSERT(mie_msie_out.read() == 1);

    expected = 0x080; // Just MTIE
    csr_write(e_csr_address::csr_adr_mie, expected);
    run_cycle(0);
    PN_ASSERT(mie_mtie_out.read() == 1);

    expected = 0x800; // Just MEIE
    csr_write(e_csr_address::csr_adr_mie, expected);
    run_cycle(0);
    PN_ASSERT(mie_meie_out.read() == 1);

    PN_INFO("csr_tb: passed.");
}

void test_mie_outputs()
{
    run_reset();
    PN_INFO("csr_tb: run test test_mie_outputs() ...");

    // Test MSIP output
    csr_write(e_csr_address::csr_adr_mie, 0x8); // Set MSIE bit
    run_cycle();
    PN_ASSERT(mie_msie_out.read() == 1);

    // Test MTIP output
    csr_write(e_csr_address::csr_adr_mie, 0x80); // Set MTIE bit
    run_cycle();
    PN_ASSERT(mie_mtie_out.read() == 1);
    PN_ASSERT(mie_msie_out.read() == 0);

    // Test MEIP output
    csr_write(e_csr_address::csr_adr_mie, 0x800); // Set MEIE bit
    run_cycle();
    PN_ASSERT(mie_meie_out.read() == 1);
    PN_ASSERT(mie_mtie_out.read() == 0);

    // Test all bits off
    csr_write(e_csr_address::csr_adr_mie, 0x0);
    run_cycle();
    PN_ASSERT(mie_msie_out.read() == 0);
    PN_ASSERT(mie_mtie_out.read() == 0);
    PN_ASSERT(mie_meie_out.read() == 0);

    PN_INFO("csr_tb: passed.");
}

void test_mtvec_initialized()
{
    run_reset();
    PN_INFO("csr_tb: run test test_mtvec_initialized() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_mtvec);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_mtvec_access_readwrite()
{
    run_reset();
    PN_INFO("csr_tb: run test test_mtvec_access_readwrite() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_mtvec);

    PN_ASSERT(expected == actual);

    // Set trap address with mode=0 (direct)
    expected = 0x80000000; // trap handler address aligned to 4 bytes
    csr_write(e_csr_address::csr_adr_mtvec, expected);
    actual = csr_read(e_csr_address::csr_adr_mtvec);

    PN_ASSERT(expected == actual);

    // Set trap address with mode=1 (vectored)
    expected = 0x80000001; // trap handler address with vectored mode
    csr_write(e_csr_address::csr_adr_mtvec, expected);
    actual = csr_read(e_csr_address::csr_adr_mtvec);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_mtvec_address_output()
{
    run_reset();
    PN_INFO("csr_tb: run test test_mtvec_address_output() ...");

    // Set trap address and check output
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> trap_addr = 0x80000000;
    csr_write(e_csr_address::csr_adr_mtvec, trap_addr);
    run_cycle();

    PN_ASSERT(mtvec_trap_address_out.read() == 0x80000000);

    // Set a different address
    trap_addr = 0x40000000;
    csr_write(e_csr_address::csr_adr_mtvec, trap_addr);
    run_cycle();

    PN_ASSERT(mtvec_trap_address_out.read() == 0x40000000);

    PN_INFO("csr_tb: passed.");
}

void test_mepc_initialized()
{
    run_reset();
    PN_INFO("csr_tb: run test test_mepc_initialized() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_mepc);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_mepc_access_readwrite()
{
    run_reset();
    PN_INFO("csr_tb: run test test_mepc_access_readwrite() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_mepc);

    PN_ASSERT(expected == actual);

    // Write and read back a value
    expected = 0xfeedbeef;
    csr_write(e_csr_address::csr_adr_mepc, expected);
    actual = csr_read(e_csr_address::csr_adr_mepc);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_mcause_initialized()
{
    run_reset();
    PN_INFO("csr_tb: run test test_mcause_initialized() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_mcause);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_mcause_access_readwrite()
{
    run_reset();
    PN_INFO("csr_tb: run test test_mcause_access_readwrite() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_mcause);

    PN_ASSERT(expected == actual);

    // Write an exception code (non-interrupt)
    expected = 0x2; // Illegal instruction
    csr_write(e_csr_address::csr_adr_mcause, expected);
    actual = csr_read(e_csr_address::csr_adr_mcause);

    PN_ASSERT(expected == actual);

    // Write an interrupt code (MSB set)
    expected = 0x80000007; // Machine timer interrupt
    csr_write(e_csr_address::csr_adr_mcause, expected);
    actual = csr_read(e_csr_address::csr_adr_mcause);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_mtval_initialized()
{
    run_reset();
    PN_INFO("csr_tb: run test test_mtval_initialized() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_mtval);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_mtval_access_readwrite()
{
    run_reset();
    PN_INFO("csr_tb: run test test_mtval_access_readwrite() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_mtval);

    PN_ASSERT(expected == actual);

    // Write and read back a value (e.g., fault address or instruction)
    expected = 0xdeadbeef;
    csr_write(e_csr_address::csr_adr_mtval, expected);
    actual = csr_read(e_csr_address::csr_adr_mtval);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_mip_initialized()
{
    run_reset();
    PN_INFO("csr_tb: run test test_mip_initialized() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_mip);

    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_mip_access_readwrite()
{
    run_reset();
    PN_INFO("csr_tb: run test test_mip_access_readwrite() ...");

    sc_uint<CFG_CSR_BUS_DATA_WIDTH> expected = 0x0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> actual = csr_read(e_csr_address::csr_adr_mip);

    PN_ASSERT(expected == actual);

    // // Set specific interrupt pending bits               //TODO simple test enough?
    // // Bit 3 = MSIP (Machine Software Interrupt Pending)
    // // Bit 7 = MTIP (Machine Timer Interrupt Pending)
    // // Bit 11 = MEIP (Machine External Interrupt Pending)
    // expected = 0x888;
    // csr_write(e_csr_address::csr_adr_mip, expected);
    // actual = csr_read(e_csr_address::csr_adr_mip);

    // PN_ASSERT(expected == actual);

    // Only set MSIP bit (bit 3)
    expected = 0x8;
    csr_write(e_csr_address::csr_adr_mip, expected);
    actual = csr_read(e_csr_address::csr_adr_mip);
    PN_ASSERT(expected == actual);

    PN_INFO("csr_tb: passed.");
}

void test_mip_outputs()
{
    run_reset();
    PN_INFO("csr_tb: run test test_mip_outputs() ...");

    // Test 1: MSIP output - set through hardware signal
    msip_in.write(1); // Set software interrupt via hardware signal
    run_cycle();
    PN_ASSERT(mip_msip_out.read() == 1);

    // Test 2: MTIP output - use hardware signal
    msip_in.write(0); // Clear software interrupt
    mtip_in.write(1); // Set timer interrupt via hardware signal
    run_cycle();
    PN_ASSERT(mip_mtip_out.read() == 1);
    PN_ASSERT(mip_msip_out.read() == 0);

    // Test 3: MEIP output - use hardware signal
    mtip_in.write(0); // Clear timer interrupt
    meip_in.write(1); // Set external interrupt via hardware signal
    run_cycle();
    PN_ASSERT(mip_meip_out.read() == 1);
    PN_ASSERT(mip_mtip_out.read() == 0);

    // Test 4: All bits off
    meip_in.write(0); // Clear external interrupt
    run_cycle();
    PN_ASSERT(mip_msip_out.read() == 0);
    PN_ASSERT(mip_mtip_out.read() == 0);
    PN_ASSERT(mip_meip_out.read() == 0);

    // Test 5: Multiple interrupts active simultaneously
    msip_in.write(1); // Set software interrupt
    mtip_in.write(1); // Set timer interrupt
    meip_in.write(1); // Set external interrupt
    run_cycle();
    PN_ASSERT(mip_msip_out.read() == 1);
    PN_ASSERT(mip_mtip_out.read() == 1);
    PN_ASSERT(mip_meip_out.read() == 1);

    // Clean up
    msip_in.write(0);
    mtip_in.write(0);
    meip_in.write(0);
    run_cycle();

    PN_INFO("csr_tb: passed.");
}

void test_interrupt_pending_detection()
{
    run_reset();
    PN_INFO("csr_tb: run test test_interrupt_pending_detection() ...");

    // Enable the global interrupt enable bit in mstatus register (MIE bit)
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> mstatus_val = (1 << 3); // MIE bit position
    csr_write(e_csr_address::csr_adr_mstatus, mstatus_val);

    // Test 1: No interrupts pending and no interrupts enabled
    csr_write(e_csr_address::csr_adr_mie, 0x0);
    csr_write(e_csr_address::csr_adr_mip, 0x0);
    run_cycle();
    PN_ASSERT(interrupt_pending_out.read() == 0);

    // Test 2: Interrupts pending but not enabled
    csr_write(e_csr_address::csr_adr_mip, 0x888); // Set MSIP, MTIP, MEIP
    csr_write(e_csr_address::csr_adr_mie, 0x0);   // No interrupts enabled
    run_cycle();
    PN_ASSERT(interrupt_pending_out.read() == 0);

    // Test 3: Interrupts enabled but not pending
    csr_write(e_csr_address::csr_adr_mip, 0x0);   // No interrupts pending
    csr_write(e_csr_address::csr_adr_mie, 0x888); // Enable MSIE, MTIE, MEIE
    run_cycle();
    PN_ASSERT(interrupt_pending_out.read() == 0);

    // Test 4: Software interrupt pending
    msip_in = 1;
    run_cycle();

    // Directly set the MSIP bit in the mip register
    csr_write(e_csr_address::csr_adr_mip, 0x8); // Set MSIP bit (bit 3)

    csr_write(e_csr_address::csr_adr_mie, 0x8); // MSIE enabled
    run_cycle();
    // PN_INFOF(("DEBUG mip=0x%x, mie=0x%x, mip_mtip=%d, mie_mtie=%d, interrupt_pending=%d",
    //           csr_read(e_csr_address::csr_adr_mip),
    //           csr_read(e_csr_address::csr_adr_mie),
    //           mip_mtip_out.read(), mie_mtie_out.read(),
    //           interrupt_pending_out.read()));

    // PN_INFOF(("DEBUG mip_msip=%d, mie_msie=%d, mstatus_mie=%d, interrupt_pending=%d",
    //             mip_msip_out.read(), mie_msie_out.read(),
    //             mstatus_mie_out.read(), interrupt_pending_out.read()));

    PN_ASSERT(interrupt_pending_out.read() == 1);

    // Test 5: Timer interrupt pending
    mtip_in = 1;
    run_cycle();
    csr_write(e_csr_address::csr_adr_mie, 0x80); // MTIE enabled
    run_cycle();
    PN_ASSERT(interrupt_pending_out.read() == 1);

    // Before test 6, verify global interrupt enable is still active
    PN_ASSERTM(mstatus_mie_out.read() == 1, "Global interrupts should be enabled");

    // Test 6: External interrupt pending and enabled
    meip_in = 1; // Set external interrupt pin
    run_cycle();
    csr_write(e_csr_address::csr_adr_mie, 0x800); // MEIE enabled
    run_cycle();
    PN_ASSERT(interrupt_pending_out.read() == 1);
    meip_in = 0; // Clear afterwards

    // Test 7: Multiple interrupts pending
    msip_in = 1;
    mtip_in = 1;
    meip_in = 1;
    run_cycle();
    csr_write(e_csr_address::csr_adr_mie, 0x888); // All interrupts enabled
    run_cycle();
    PN_ASSERT(interrupt_pending_out.read() == 1);

    // Test 8: Disable global interrupts via mstatus.MIE
    csr_write(e_csr_address::csr_adr_mstatus, 0x0); // Clear MIE bit
    run_cycle();
    PN_ASSERT(mstatus_mie_out.read() == 0);
    PN_ASSERT(interrupt_pending_out.read() == 0); // Should be disabled by global MIE

    PN_INFO("csr_tb: passed.");
}