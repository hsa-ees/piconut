
/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Johannes Hofmann <johannes.hofmann1@tha.de>
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

#include "top.h"

void m_demo_debugger::pn_trace(sc_trace_file* tf, int level)
{
    if(level >= 2)
    {
        cpu->pn_trace(tf, level);
        uart->pn_trace(tf, level);
        debugger->pn_trace(tf, level);
    }
}

void m_demo_debugger::init_submodules()
{

    // ----------- Create submodules -----------
    // ----------- CPU -----------
    cpu = sc_new<m_cpu>("i_cpu");

    cpu->reset(reset);
    cpu->clk(clk);

    cpu->wb_master.ack_i(wb_ack_master);
    cpu->wb_master.dat_i(wb_dat_i_master);
    cpu->wb_master.dat_o(wb_dat_o);
    cpu->wb_master.adr_o(wb_adr);
    cpu->wb_master.we_o(wb_we);
    cpu->wb_master.stb_o(wb_stb);
    cpu->wb_master.cyc_o(wb_cyc);
    cpu->wb_master.sel_o(wb_sel);
    cpu->wb_master.rty_i(wb_rty_master);
    cpu->wb_master.err_i(wb_err_master);

    cpu->mtip_in(dummy_low);
    cpu->msip_in(dummy_low);
    cpu->meip_in(dummy_low);

    cpu->debug_haltrequest_in(debug_haltrequest);
    cpu->debug_haltrequest_ack_out(debug_haltrequest_ack);

    // ----------- UART -----------
    uart = sc_new<m_uart>("i_uart", PN_CFG_UART_BASE_ADDRESS);

    uart->reset(reset);
    uart->clk(clk);

    uart->rx(rx_i);
    uart->tx(tx_o);

    uart->wb_slave.ack_o(wb_ack_uart);
    uart->wb_slave.dat_i(wb_dat_o);
    uart->wb_slave.dat_o(wb_dat_i_uart);
    uart->wb_slave.adr_i(wb_adr);
    uart->wb_slave.we_i(wb_we);
    uart->wb_slave.stb_i(wb_stb);
    uart->wb_slave.cyc_i(wb_cyc);
    uart->wb_slave.sel_i(wb_sel);
    uart->wb_slave.rty_o(wb_rty_uart);
    uart->wb_slave.err_o(wb_err_uart);

    // ----------- DEBUGGER -----------
    debugger = sc_new<m_debugger>("i_debugger");
    debugger->clk(clk);
    debugger->reset(reset);
    debugger->tck_i(tck_i);
    debugger->tms_i(tms_i);
    debugger->tdi_i(tdi_i);
    debugger->trst_n_i(dummy_high);
    debugger->tdo_o(tdo_o);

    debugger->wb_ack_o(wb_ack_debugger);
    debugger->wb_dat_i(wb_dat_o_debugger);
    debugger->wb_dat_o(wb_dat_i_debugger);
    debugger->wb_adr_i(wb_adr_debugger);
    debugger->wb_we_i(wb_we);
    debugger->wb_bte_i(wb_bte);
    debugger->wb_cti_i(wb_cti);
    debugger->wb_stb_i(wb_stb);
    debugger->wb_cyc_i(wb_cyc);
    debugger->wb_sel_i(wb_sel_debugger);
    debugger->wb_rty_o(wb_rty_debugger);
    debugger->wb_err_o(wb_err_debugger);

    debugger->debug_haltrequest_o(debug_haltrequest);
    debugger->debug_haltrequest_ack_i(debug_haltrequest_ack);
}

void m_demo_debugger::proc_cmb()
{
    dummy_low.write(0);
    dummy_high.write(1);
}

void m_demo_debugger::proc_cmb_wb()
{
    wb_adr_debugger = wb_adr.read().range(31, 0);
    wb_sel_debugger = wb_sel.read().range(3, 0);
    wb_dat_o_debugger = wb_dat_o.read().range(31, 0);

    if(wb_adr.read() >= PN_CFG_UART_BASE_ADDRESS &&
        wb_adr.read() < (PN_CFG_UART_BASE_ADDRESS + 0x1C))
    {
        wb_dat_i_master = wb_dat_i_uart.read();
        wb_ack_master = wb_ack_uart.read();
        wb_rty_master = wb_rty_uart.read();
        wb_err_master = wb_err_uart.read();
    }
    else
    {
        wb_dat_i_master = (sc_uint<32>(0), wb_dat_i_debugger.read());
        wb_ack_master = wb_ack_debugger.read();
        wb_rty_master = wb_rty_debugger.read();
        wb_err_master = wb_err_debugger.read();
    }
}
