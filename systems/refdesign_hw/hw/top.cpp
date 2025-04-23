
/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Claus Janicher <claus.janicher@tha.de>
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

void m_top::Trace(sc_trace_file *tf, int level)
{

   // calling trace of submodules
   if (level >= 2)
   {
      piconut->Trace(tf, level);
      wb_uart->Trace(tf, level);
   }
   // Internal traces
}

void m_top::init_submodules()
{

   // ----------- Create submodules -----------
   // ----------- PicoNut -----------
   piconut = sc_new<m_piconut>("piconut");

   piconut->reset(reset);
   piconut->clk(clk_25);

   piconut->debug_haltrequest_in(dummy_low);
   // Issue: vh_const not working right now. Uncomment the line below
   // after vh_const updated and remove dummy signal.
   // piconut->debug_haltrequest_in(vh_const<bool>(0));
   piconut->debug_haltrequest_ack_out(vh_open);

   piconut->wb_ack_i(wb_ack);
   piconut->wb_dat_i(wb_dat_i);
   piconut->wb_dat_o(wb_dat_o);
   piconut->wb_adr_o(wb_adr);
   piconut->wb_we_o(wb_we);
   piconut->wb_stb_o(wb_stb);
   piconut->wb_cyc_o(wb_cyc);
   piconut->wb_sel_o(wb_sel_o);

   // ----------- WB_UART -----------
   wb_uart = sc_new<m_wb_uart>("wb_uart");

   wb_uart->reset(reset);
   wb_uart->clk(clk_25);

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

}

void m_top::proc_cmb()
{
   dummy_low.write(0);

   test_o = clk_25.read();
}
