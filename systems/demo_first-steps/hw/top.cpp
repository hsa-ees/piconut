/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2026 Johannes Hofmann <johannes.hofmann1@tha.de>
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

void m_demo_first_steps::pn_trace(sc_trace_file* tf, int level)
{

    // calling trace of submodules
    if(level >= 2)
    {
        pn_interconnect->pn_trace(tf, level);
        cpu->pn_trace(tf, level);
        gpio->pn_trace(tf, level);
        debugger->pn_trace(tf, level);
#ifdef __SYNTHESIS__
        uart->pn_trace(tf, level);
#endif
    }
    // Internal traces
}

void m_demo_first_steps::init_submodules()
{

    // ----------- Create submodules -----------
    pn_interconnect = sc_new<m_pn_interconnect>("i_pn_interconnect");

    // ----------- CPU -----------
    cpu = sc_new<m_cpu>("i_cpu");
    cpu->clk(clk);

    cpu->reset(reset);

    cpu->mtip_in(dummy_low);
    cpu->msip_in(dummy_low);
    cpu->meip_in(dummy_low);

    pn_interconnect->add_module(cpu);

    // ----------- GPIO -----------
    gpio = sc_new<m_gpio>("i_gpio", PN_CFG_GPIO_BASE_ADDRESS, 4, 8);
    gpio->clk(clk);
    gpio->reset(reset);

    gpio->input(gpio_input);
    gpio->output(gpio_output);

    pn_interconnect->add_module(gpio);

#ifdef __SYNTHESIS__
    // ----------- UART -----------
    uart = sc_new<m_uart>("i_uart", PN_CFG_UART_BASE_ADDRESS);
    uart->clk(clk);
    uart->reset(reset);

    uart->rx(uart_rx);
    uart->tx(uart_tx);

    pn_interconnect->add_module(uart);
#else
    // ----------- UART Soft -----------
    uart = std::make_unique<c_soft_uart>(0x22, PN_CFG_UART_BASE_ADDRESS);
    cpu->membrana->add_peripheral(PN_CFG_UART_BASE_ADDRESS, std::move(uart));
#endif

    // ----------- Debugger -----------
    debugger = sc_new<m_debugger>("i_debugger");
    debugger->clk(clk);
    debugger->reset(reset);

    debugger->tck_i(jtag_tck_i);
    debugger->tms_i(jtag_tms_i);
    debugger->tdi_i(jtag_tdi_i);
    debugger->trst_n_i(dummy_high);
    debugger->tdo_o(jtag_tdo_o);

    pn_interconnect->add_module(debugger);

    pn_interconnect->elaborate();
}

void m_demo_first_steps::proc_cmb()
{
    dummy_low.write(0);
    dummy_high.write(1);
}
