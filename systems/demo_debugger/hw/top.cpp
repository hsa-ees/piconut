
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
        pn_interconnect->pn_trace(tf, level);
        cpu->pn_trace(tf, level);
        uart->pn_trace(tf, level);
        debugger->pn_trace(tf, level);
    }
}

void m_demo_debugger::init_submodules()
{

    // ----------- Create submodules -----------
    pn_interconnect = sc_new<m_pn_interconnect>("i_pn_interconnect");

    // ----------- CPU -----------
    cpu = sc_new<m_cpu>("i_cpu");
    cpu->reset(reset);
    cpu->clk(clk);

    cpu->mtip_in(dummy_low);
    cpu->msip_in(dummy_low);
    cpu->meip_in(dummy_low);

    pn_interconnect->add_module(cpu);

    // ----------- UART -----------
    uart = sc_new<m_uart>("i_uart", PN_CFG_UART_BASE_ADDRESS);
    uart->reset(reset);
    uart->clk(clk);

    uart->rx(rx_i);
    uart->tx(tx_o);

    pn_interconnect->add_module(uart);

    // ----------- DEBUGGER -----------
    debugger = sc_new<m_debugger>("i_debugger");
    debugger->clk(clk);
    debugger->reset(reset);

    debugger->tck_i(tck_i);
    debugger->tms_i(tms_i);
    debugger->tdi_i(tdi_i);
    debugger->trst_n_i(dummy_high);
    debugger->tdo_o(tdo_o);

    pn_interconnect->add_module(debugger);

    pn_interconnect->elaborate();
}

void m_demo_debugger::proc_cmb()
{
    dummy_low.write(0);
    dummy_high.write(1);
}
