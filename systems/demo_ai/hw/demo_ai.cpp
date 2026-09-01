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


#include "demo_ai.h"


void m_demo_ai::pn_trace (sc_trace_file* tf, int level) {

  // Local signals ...
  if (level >= 1) {
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, pwroff);
#ifdef __SYNTHESIS__
    PN_TRACE(tf, rx_i);
    PN_TRACE(tf, rx_o);
#endif
  }

  // Submodules ...
  if (level >= 2) {
    cpu->pn_trace (tf, level - 1);
#ifdef __SYNTHESIS__
    wb_uart->pn_trace (tf, level - 1);
#endif
  }
}


void m_demo_ai::init_submodules() {

  // CPU ...
  cpu = sc_new<m_cpu>("i_cpu");
  cpu->clk(clk);
  cpu->reset(reset);
  cpu->pwroff(pwroff);

  // ... Debugging ...
  cpu->debug_slave.haltrequest_i(dummy_low);
    // TBD: vh_const not working right now. Uncomment the line below
    // after vh_const updated and remove dummy signal.
  // cpu->debug_haltrequest_in(vh_const<bool>(0));
  cpu->debug_slave.haltrequest_ack_o(vh_open);

  // ... Interrupts ...
  cpu->mtip_in(dummy_low);
  cpu->msip_in(dummy_low);
  cpu->meip_in(dummy_low);

  // ... Wishbone interface ...
#ifdef __SYNTHESIS__
  cpu->wb_ack_i(wb_ack);
  cpu->wb_dat_i(wb_dat_i);
  cpu->wb_dat_o(wb_dat_o);
  cpu->wb_adr_o(wb_adr);
  cpu->wb_we_o(wb_we);
  cpu->wb_stb_o(wb_stb);
  cpu->wb_cyc_o(wb_cyc);
  cpu->wb_sel_o(wb_sel_o);
#endif

  // UART ...
#ifdef __SYNTHESIS__
  wb_uart = sc_new<m_uart>("wb_uart");

  wb_uart->reset(reset);
  wb_uart->clk(clk);

  wb_uart->rx(rx_i);
  wb_uart->tx(tx_o);

  wb_uart->ack_o(wb_ack);
  wb_uart->dat_i(wb_dat_o);
  wb_uart->dat_o(wb_dat_i);
  wb_uart->addr_i(wb_adr);
  wb_uart->we_i(wb_we);
  wb_uart->stb_i(wb_stb);
  wb_uart->cyc_i(wb_cyc);
  wb_uart->sel_i(wb_sel_o);
  wb_uart->rty_o(wb_rty);
  wb_uart->err_o(wb_err);
#endif
}


void m_demo_ai::proc_cmb_dummy () {
  dummy_low.write(0);
}
