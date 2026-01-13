/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, University of Applied Sciences

  Description: This file contains the implementation of the c_soft_graphics for simulation ONLY

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

#include "../pn_interconnect.h"

#include <systemc.h>

SC_MODULE(m_mock_master), pn_module_if
{
public:
    pn_wishbone_master_t wb_master;
    SC_CTOR(m_mock_master)
        : wb_master{
              .alen = 32,
              .dlen = 32}
    {
        pn_add_wishbone_master(&wb_master);
    }
};

SC_MODULE(m_mock_slave), pn_module_if
{
public:
    pn_wishbone_slave_t wb_slave_1;
    pn_wishbone_slave_t wb_slave_2;
    SC_CTOR(m_mock_slave)
        : wb_slave_1{
              .alen = 32,
              .dlen = 32,
              .base_address = 0x10000000,
              .size = 0x0000ffff}
        , wb_slave_2{//
              .alen = 32,
              .dlen = 32,
              .base_address = 0x10010000,
              .size = 0x0002ffff}
    {
        pn_add_wishbone_slave(&wb_slave_1);
        pn_add_wishbone_slave(&wb_slave_2);
    }
};

m_mock_master PN_NAME(mock_master);
m_mock_slave PN_NAME(mock_slave);

void test_wishbone_connected();

int sc_main(int argc, char** argv)
{
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("interconnect_tb");

    m_pn_interconnect PN_NAME(i_dut);
    i_dut.add_module(&mock_master);
    i_dut.add_module(&mock_slave);
    i_dut.elaborate();

    i_dut.pn_trace(tf, pn_cfg_vcd_level);

    sc_start(SC_ZERO_TIME);
    cout << "\n\t\t*****Simulation started*****" << endl;

    test_wishbone_connected();

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}

void test_wishbone_connected()
{
    PN_ASSERT(mock_slave.wb_slave_1.adr_i.read() == 0);
    PN_ASSERT(mock_slave.wb_slave_1.dat_i.read() == 0);
    PN_ASSERT(mock_slave.wb_slave_1.sel_i.read() == 0);
    PN_ASSERT(mock_slave.wb_slave_1.stb_i.read() == 0);
    PN_ASSERT(mock_slave.wb_slave_1.cyc_i.read() == 0);
    PN_ASSERT(mock_slave.wb_slave_1.we_i.read() == 0);

    PN_ASSERT(mock_slave.wb_slave_2.adr_i.read() == 0);
    PN_ASSERT(mock_slave.wb_slave_2.dat_i.read() == 0);
    PN_ASSERT(mock_slave.wb_slave_2.sel_i.read() == 0);
    PN_ASSERT(mock_slave.wb_slave_2.stb_i.read() == 0);
    PN_ASSERT(mock_slave.wb_slave_2.cyc_i.read() == 0);
    PN_ASSERT(mock_slave.wb_slave_2.we_i.read() == 0);

    mock_master.wb_master.adr_o = 0xdeadbeef;
    mock_master.wb_master.dat_o = 0xaffeaffe;
    mock_master.wb_master.sel_o = 0xf;
    mock_master.wb_master.stb_o = 1;
    mock_master.wb_master.cyc_o = 1;
    mock_master.wb_master.we_o = 1;

    sc_start();

    PN_ASSERT(mock_slave.wb_slave_1.adr_i.read() == 0xdeadbeef);
    PN_ASSERT(mock_slave.wb_slave_1.dat_i.read() == 0xaffeaffe);
    PN_ASSERT(mock_slave.wb_slave_1.sel_i.read() == 0xf);
    PN_ASSERT(mock_slave.wb_slave_1.stb_i.read() == 1);
    PN_ASSERT(mock_slave.wb_slave_1.cyc_i.read() == 1);
    PN_ASSERT(mock_slave.wb_slave_1.we_i.read() == 1);

    mock_master.wb_master.adr_o = 0x10000000;
    mock_master.wb_master.dat_o = 0;
    mock_master.wb_master.sel_o = 0;
    mock_master.wb_master.stb_o = 0;
    mock_master.wb_master.cyc_o = 0;
    mock_master.wb_master.we_o = 0;

    mock_slave.wb_slave_1.dat_o = 0xaffeaffe;
    mock_slave.wb_slave_1.ack_o = 1;
    mock_slave.wb_slave_1.rty_o = 1;
    mock_slave.wb_slave_1.err_o = 1;

    sc_start();

    PN_ASSERT(mock_master.wb_master.dat_i.read() == 0xaffeaffe);
    PN_ASSERT(mock_master.wb_master.ack_i.read() == 1);
    PN_ASSERT(mock_master.wb_master.rty_i.read() == 1);
    PN_ASSERT(mock_master.wb_master.err_i.read() == 1);

    mock_master.wb_master.adr_o = 0;

    sc_start();

    PN_ASSERT(mock_master.wb_master.dat_i.read() == 0);
    PN_ASSERT(mock_master.wb_master.ack_i.read() == 0);
    PN_ASSERT(mock_master.wb_master.rty_i.read() == 0);
    PN_ASSERT(mock_master.wb_master.err_i.read() == 0);

    mock_master.wb_master.adr_o = 0x10010000;

    mock_slave.wb_slave_2.dat_o = 0xf00dbeef;
    mock_slave.wb_slave_2.ack_o = 1;
    mock_slave.wb_slave_2.rty_o = 1;
    mock_slave.wb_slave_2.err_o = 1;

    sc_start();

    PN_ASSERT(mock_master.wb_master.dat_i.read() == 0xf00dbeef);
    PN_ASSERT(mock_master.wb_master.ack_i.read() == 1);
    PN_ASSERT(mock_master.wb_master.rty_i.read() == 1);
    PN_ASSERT(mock_master.wb_master.err_i.read() == 1);

    mock_master.wb_master.adr_o = 0;
    mock_slave.wb_slave_1.dat_o = 0;
    mock_slave.wb_slave_1.ack_o = 0;
    mock_slave.wb_slave_1.rty_o = 0;
    mock_slave.wb_slave_1.err_o = 0;
    mock_slave.wb_slave_2.dat_o = 0;
    mock_slave.wb_slave_2.ack_o = 0;
    mock_slave.wb_slave_2.rty_o = 0;
    mock_slave.wb_slave_2.err_o = 0;
}
