/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2026 Hermann Zoha <hermann.zoha@tha.de>

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

void m_demo_sdram::pn_trace(sc_trace_file* tf, int level)
{

    // calling trace of submodules
    if(level >= 2)
    {
        pn_interconnect->pn_trace(tf, level);
        cpu->pn_trace(tf, level);
        uart->pn_trace(tf, level);
        sdram_controller->pn_trace(tf, level);
    }
    // Internal traces
}

void m_demo_sdram::init_submodules()
{
    pn_interconnect = sc_new<m_pn_interconnect>("i_pn_interconnect");

    // ----------- Create submodules -----------
    // ----------- CPU -----------
    cpu = sc_new<m_cpu>("i_cpu");
    cpu->clk(clk);
    cpu->reset(reset);

    cpu->mtip_in(dummy_low);
    cpu->msip_in(dummy_low);
    cpu->meip_in(dummy_low);

    pn_interconnect->add_module(cpu);




    // -------------SDRAM_CONTROLLER----------
    sdram_controller = sc_new<m_sdram_controller>("i_sdram_controller", PN_CFG_SDRAM_CONTROLLER_BASE_ADDRESS);

    sdram_controller->clk(clk);
    sdram_controller->reset(reset);
    sdram_controller->sdram_clk(sdram_clk);
    sdram_controller->sdram_clk_en(sdram_clk_en);
    sdram_controller->sdram_cp_sel_n(sdram_cp_sel_n);
    sdram_controller->sdram_wr_en_n(sdram_wr_en_n);
    sdram_controller->sdram_col_ac_sel_n(sdram_col_ac_sel_n);
    sdram_controller->sdram_row_ac_sel_n(sdram_row_ac_sel_n);
    sdram_controller->sdram_addr(sdram_addr);
    sdram_controller->sdram_ba(sdram_ba);
    sdram_controller->c_we_o(c_we_o);
    sdram_controller->sdram_dq_i(sdram_dq_i);
    sdram_controller->sdram_dq_o(sdram_dq_o);
    sdram_controller->sdram_dqm(sdram_dqm);

    pn_interconnect->add_module(sdram_controller);

        // ----------- UART -----------

   uart = sc_new<m_uart>("i_uart", PN_CFG_UART_BASE_ADDRESS);

   uart->reset(reset);
   uart->clk(clk);

   uart->rx(rx_i);
   uart->tx(tx_o);

   pn_interconnect->add_module(uart);


    pn_interconnect->elaborate();
}

void m_demo_sdram::proc_cmb()
{
    dummy_low.write(0);
}
