/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Marco Milenkovic <Marco.Milenkovic@hs-augsburg.de>
                2025 Gundolf Kiefer <gundolf.kiefer@tha.de>

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

// TBD(jh): remove clock division for orangecrab board.
#include <string_view>

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
constexpr std::string_view current_board = STR(PN_CFG_BOARD);

void m_refdesign::pn_trace(sc_trace_file* tf, int level)
{

    // calling trace of submodules
    if(level >= 2)
    {
        pn_interconnect->pn_trace(tf, level);
        cpu->pn_trace(tf, level);
#ifdef __SYNTHESIS__
        uart->pn_trace(tf, level);
#endif
    }
    // Internal traces
}

void m_refdesign::init_submodules()
{

    // ----------- Create submodules -----------
    pn_interconnect = sc_new<m_pn_interconnect>("i_pn_interconnect");

    // ----------- CPU -----------
    cpu = sc_new<m_cpu>("i_cpu");
    if constexpr(current_board == "orangecrab")
    {
        cpu->clk(clk_half);
    }
    else
    {
        cpu->clk(clk);
    }

    cpu->reset(reset);

    cpu->mtip_in(dummy_low);
    cpu->msip_in(dummy_low);
    cpu->meip_in(dummy_low);

    pn_interconnect->add_module(cpu);

    // ----------- UART -----------
#ifdef __SYNTHESIS__
    uart = sc_new<m_uart>("i_uart", PN_CFG_UART_BASE_ADDRESS);

    if constexpr(current_board == "orangecrab")
    {
        uart->clk(clk_half);
    }
    else
    {
        uart->clk(clk);
    }
    uart->reset(reset);

    uart->rx(uart_rx);
    uart->tx(uart_tx);

    pn_interconnect->add_module(uart);
#endif

    pn_interconnect->elaborate();
}

void m_refdesign::proc_cmb()
{
    dummy_low.write(0);
}

void m_refdesign::proc_clk()
{
    while(true)
    {
        wait();

        clk_half = !clk_half.read();
    }
}
