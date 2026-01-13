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

#ifndef __TOP_H__
#define __TOP_H__

#include <piconut.h>

#include "wb_timer.h"
#include "wb_uart.h"



SC_MODULE(m_top)
{
public:
   // Ports

   sc_in_clk PN_NAME(clk);
   sc_in<bool> PN_NAME(reset);

   sc_in<bool> PN_NAME(rx_i);
   sc_out<bool> PN_NAME(tx_o);

   // Constructor/Destructors
   SC_CTOR(m_top)
   {

      SC_METHOD(proc_comb_wb);
      sensitive << wb_dat_i_timer << wb_dat_i_uart << wb_adr << wb_ack_timer << wb_ack_uart;

      init_submodules();

   }

   // Functions
   void pn_trace(sc_trace_file * tf, int level = 1);

   // Processes
   void proc_comb_wb();

   // Submodules
   m_piconut *piconut;
   m_wb_timer *wb_timer;
   m_wb_uart *wb_uart;

protected:
   // Internal Signals
   // ---------- Wishbone intermediate signals ----------
   sc_signal<bool> PN_NAME(wb_ack_pn);
   sc_signal<bool> PN_NAME(wb_ack_timer);
   sc_signal<bool> PN_NAME(wb_ack_uart);
   sc_signal<sc_uint<32>> PN_NAME(wb_dat_i_pn);
   sc_signal<sc_uint<32>> PN_NAME(wb_dat_i_timer);
   sc_signal<sc_uint<32>> PN_NAME(wb_dat_i_uart);
   sc_signal<sc_uint<32>> PN_NAME(wb_dat_o);
   sc_signal<sc_uint<32>> PN_NAME(wb_adr);
   sc_signal<bool> PN_NAME(wb_we);
   sc_signal<bool> PN_NAME(wb_stb);
   sc_signal<bool> PN_NAME(wb_cyc);
   sc_signal<sc_uint<4>> PN_NAME(wb_sel_o);

   sc_signal<bool> PN_NAME(wb_rty_timer);
   sc_signal<bool> PN_NAME(wb_rty_uart);
   sc_signal<bool> PN_NAME(wb_err_timer);
   sc_signal<bool> PN_NAME(wb_err_uart);

   sc_signal<bool> PN_NAME(dummy_low);

   sc_signal<bool> PN_NAME(timer_irq_signal); // Timer interrupt signal
   sc_signal<bool> PN_NAME(autoreload_pulse_signal); // Timer autoreload pulse signal


   // Methods
   void init_submodules();
};

#endif // __TOP_H__
