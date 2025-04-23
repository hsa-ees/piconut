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

#include "csr_master.h"
#include "typedef.h"

#include <base.h>
#include <piconut-config.h>

#include <systemc.h>

#include <cstdint>

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

// csr bus
sc_signal<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(csr_bus_wdata);
sc_signal<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(csr_bus_rdata);

sc_signal<sc_uint<32>> PN_NAME(source_register);
sc_signal<bool> PN_NAME(imm_en);
sc_signal<sc_uint<5>> PN_NAME(imm);
sc_signal<sc_uint<2>> PN_NAME(write_mode);

///////////////// Helpers /////////////////
void delay(int delay = 1);

///////////////// Tests /////////////////
void test_write_mode_write();
void test_write_mode_set();
void test_write_mode_clear();
void test_write_mode_write_immediate();
void test_write_mode_set_immediate();
void test_write_mode_clear_immediate();

int sc_main(int argc, char** argv)
{
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("csr_master_tb");

    m_csr_master dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.csr_bus_rdata_in(csr_bus_rdata);
    dut_inst.source_reg_in(source_register);
    dut_inst.imm_en_in(imm_en);
    dut_inst.imm_in(imm);
    dut_inst.write_mode_in(write_mode);
    dut_inst.csr_bus_wdata_out(csr_bus_wdata);

    dut_inst.Trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT

    sc_start(SC_ZERO_TIME); // start simulation

    cout << "\n\t\t*****Simulation started*****" << endl;

    test_write_mode_write();
    test_write_mode_set();
    test_write_mode_clear();
    test_write_mode_write_immediate();
    test_write_mode_set_immediate();
    test_write_mode_clear_immediate();

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}

///////////////// Helpers /////////////////
void delay(int delay)
{
    sc_start(1, SC_NS);
}

///////////////// Tests /////////////////
void test_write_mode_write()
{
    PN_INFO("csr_master_tb: run test test_write_mode_write() ...");

    source_register = 0xFFFFFFFF;
    write_mode = e_csr_write_mode::CSR_WRITE_ALL;
    delay();

    PN_ASSERT(csr_bus_wdata.read() == 0xFFFFFFFF);

    source_register = 0x0;
    write_mode = 0x0;

    PN_INFO("csr_master_tb: passed.");
}

void test_write_mode_set()
{
    PN_INFO("csr_master_tb: run test test_write_mode_set() ...");

    csr_bus_rdata = 0xFFFFFFFF;
    source_register = 0x0;
    write_mode = e_csr_write_mode::CSR_WRITE_SET;
    delay();

    PN_ASSERT(csr_bus_wdata.read() == 0xFFFFFFFF);

    csr_bus_rdata = 0x0;
    source_register = 0x0;
    write_mode = 0x0;

    PN_INFO("csr_master_tb: passed.");
}

void test_write_mode_clear()
{
    PN_INFO("csr_master_tb: run test test_write_mode_clear() ...");

    csr_bus_rdata = 0xFFFFFFFF;
    source_register = 0x55555555;
    write_mode = e_csr_write_mode::CSR_WRITE_CLEAR;
    delay();

    PN_ASSERT(csr_bus_wdata.read() == 0x55555555);

    csr_bus_rdata = 0x0;
    source_register = 0x0;
    write_mode = 0x0;

    PN_INFO("csr_master_tb: passed.");
}

void test_write_mode_write_immediate()
{
    PN_INFO("csr_master_tb: run test test_write_mode_write_immediate() ...");

    imm_en = 1;
    imm = 0x1F;
    write_mode = e_csr_write_mode::CSR_WRITE_ALL;
    delay();

    PN_ASSERT(csr_bus_wdata.read() == 0x1F);

    imm_en = 0;
    imm = 0x0;
    write_mode = 0x0;

    PN_INFO("csr_master_tb: passed.");
}

void test_write_mode_set_immediate()
{
    PN_INFO("csr_master_tb: run test test_write_mode_set_immediate() ...");

    csr_bus_rdata = 0x0;
    imm_en = 1;
    imm = 0x1F;
    write_mode = e_csr_write_mode::CSR_WRITE_SET;
    delay();

    PN_ASSERT(csr_bus_wdata.read() == 0x1F);

    csr_bus_rdata = 0x0;
    imm_en = 0;
    imm = 0x0;
    write_mode = 0x0;

    PN_INFO("csr_master_tb: passed.");
}

void test_write_mode_clear_immediate()
{
    PN_INFO("csr_master_tb: run test test_write_mode_clear_immediate() ...");

    csr_bus_rdata = 0xFFFFFFFF;
    imm_en = 1;
    imm = 0x1F;
    write_mode = e_csr_write_mode::CSR_WRITE_CLEAR;
    delay();

    PN_ASSERT(csr_bus_wdata.read() == 0x1F);

    csr_bus_rdata = 0x0;
    imm_en = 0;
    imm = 0x0;
    write_mode = 0x0;

    PN_INFO("csr_master_tb: passed.");
}
