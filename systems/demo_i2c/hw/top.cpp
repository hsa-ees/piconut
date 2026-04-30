
/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Claus Janicher <claus.janicher@tha.de>
                     2025 Tristan Kundrat <tristan.kundrat@tha.de>
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

void m_demo_i2c::pn_trace(sc_trace_file* tf, int level)
{
    cpu->pn_trace(tf, level);
    i2c->pn_trace(tf, level);
}

void m_demo_i2c::init_submodules()
{
    pn_interconnect = sc_new<m_pn_interconnect>("i_pn_interconnect");

    // ----------- Create submodules -----------
    // ----------- PicoNut -----------
    cpu = sc_new<m_cpu>("i_cpu");
    cpu->clk(clk);
    cpu->reset(reset);

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

    // ----------- I2C -----------
    i2c = sc_new<m_i2c>("i_i2c", PN_CFG_I2C_BASE_ADDRESS);

    i2c->reset(reset);
    i2c->clk(clk);

    pn_interconnect->add_module(i2c);

    i2c->sda_i(sda_i);
    i2c->sda_o(sda_o);

    i2c->scl_i(scl_i);
    i2c->scl_o(scl_o);

    i2c->sda_oe(sda_oe);
    i2c->scl_oe(scl_oe);

    pn_interconnect->elaborate();
}

void m_demo_i2c::proc_comb()
{
    dummy_low.write(0);
}
