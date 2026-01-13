/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Claus Janicher <claus.janicher@tha.de>
                     2025 Alexander Beck <alexander.beck1@tha.de>
                     2025 Christian Zellinger <christian.zellinger1@tha.de>
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

void m_top::pn_trace(sc_trace_file* tf, int level)
{

    // calling trace of submodules
    if(level >= 2)
    {
        piconut->pn_trace(tf, level);
        wb_timer->pn_trace(tf, level);
    }
    // Internal traces
}

void m_top::init_submodules()
{

    // ----------- Create submodules -----------
    // ----------- PicoNut -----------
    piconut = sc_new<m_piconut>("piconut");

    piconut->reset(reset);
    piconut->clk(clk);

    piconut->debug_haltrequest_in(dummy_low);
    // Issue: vh_const not working right now. Uncomment the line below
    // after vh_const updated and remove dummy signal.
    // piconut->debug_haltrequest_in(vh_const<bool>(0));
    piconut->debug_haltrequest_ack_out(vh_open);

    piconut->wb_ack_i(wb_ack_pn);
    piconut->wb_dat_i(wb_dat_i_pn);
    piconut->wb_dat_o(wb_dat_o);
    piconut->wb_adr_o(wb_adr);
    piconut->wb_we_o(wb_we);
    piconut->wb_stb_o(wb_stb);
    piconut->wb_cyc_o(wb_cyc);
    piconut->wb_sel_o(wb_sel_o);

    // Connect the timer interrupt signal to the machine timer interrupt input
    piconut->mtip_in(dummy_low); // Timer interrupt connected to machine timer interrupt
    piconut->msip_in(dummy_low); // Not used in this design
    piconut->meip_in(timer_irq_signal);

    // ----------- WB_TIMER -----------
    wb_timer = sc_new<m_wb_timer>("wb_timer");

    wb_timer->reset(reset);
    wb_timer->clk(clk);

    wb_timer->wb_ack_o(wb_ack_timer);
    wb_timer->wb_dat_i(wb_dat_o);
    wb_timer->wb_dat_o(wb_dat_i_timer);
    wb_timer->wb_we_i(wb_we);
    wb_timer->wb_stb_i(wb_stb);
    wb_timer->wb_cyc_i(wb_cyc);
    wb_timer->wb_sel_i(wb_sel_o);
    wb_timer->wb_rty_o(wb_rty_timer);
    wb_timer->wb_err_o(wb_err_timer);
    wb_timer->wb_adr_i(wb_adr);

    wb_timer->timer_irq(timer_irq_signal);
    wb_timer->autoreload_pulse(autoreload_pulse_signal);

    // ----------- WB_UART -----------
    wb_uart = sc_new<m_wb_uart>("wb_uart");

    wb_uart->reset(reset);
    wb_uart->clk(clk);

    wb_uart->rx(rx_i);
    wb_uart->tx(tx_o);

    wb_uart->ack_o(wb_ack_uart);
    wb_uart->dat_i(wb_dat_o);
    wb_uart->dat_o(wb_dat_i_uart);
    wb_uart->addr_i(wb_adr);
    wb_uart->we_i(wb_we);
    wb_uart->stb_i(wb_stb);
    wb_uart->cyc_i(wb_cyc);
    wb_uart->sel_i(wb_sel_o);
    wb_uart->rty_o(wb_rty_uart);
    wb_uart->err_o(wb_err_uart);
}

void m_top::proc_comb_wb()
{
    if(wb_adr.read() >= CFG_TIMER_BASE_ADDRESS &&
        wb_adr.read() < (CFG_TIMER_BASE_ADDRESS + 79))
    {
        wb_dat_i_pn = wb_dat_i_timer.read();
        wb_ack_pn = wb_ack_timer.read();
    }
    else
    {
        wb_dat_i_pn = wb_dat_i_uart.read();
        wb_ack_pn = wb_ack_uart.read();
    }
    dummy_low.write(0);
}
