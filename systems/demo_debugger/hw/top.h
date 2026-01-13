
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

#ifndef __TOP_H__
#define __TOP_H__

#include <piconut.h>

#include <cpu.h>
#include <uart.h>
#include <debugger.h>


SC_MODULE(m_demo_debugger)
{
public:
    // --------------- System ---------------
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    // --------------- Uart ---------------
    sc_in<bool> PN_NAME(rx_i);
    sc_out<bool> PN_NAME(tx_o);

    // --------------- Jtag ---------------
    sc_in<bool> PN_NAME(tck_i);
    sc_in<bool> PN_NAME(tms_i);
    sc_in<bool> PN_NAME(tdi_i);
    // sc_in<bool> PN_NAME(trst_n_i); // optional
    sc_out<bool> PN_NAME(tdo_o);

    // Constructor/Destructors
    SC_CTOR(m_demo_debugger)
    {
        SC_METHOD(proc_cmb);

        SC_METHOD(proc_cmb_wb);
        sensitive << wb_adr
                  << wb_sel
                  << wb_dat_o
                  << wb_dat_i_uart
                  << wb_ack_uart
                  << wb_rty_uart
                  << wb_err_uart
                  << wb_dat_i_debugger
                  << wb_ack_debugger
                  << wb_rty_debugger
                  << wb_err_debugger;

        init_submodules();
    }

    // Functions
    void pn_trace(sc_trace_file * tf, int level = 1);

    // Processes
    void proc_cmb();
    void proc_cmb_wb();

    // Submodules
    m_cpu* cpu;
    m_uart* uart;
    m_debugger* debugger;

protected:
    // Internal Signals
    sc_signal<bool> PN_NAME(dummy_low);
    sc_signal<bool> PN_NAME(dummy_high);

    sc_signal<bool> PN_NAME(debug_haltrequest);
    sc_signal<bool> PN_NAME(debug_haltrequest_ack);

    // ---------- Wishbone intermediate signals ----------
    sc_signal<bool> PN_NAME(wb_ack_master);
    sc_signal<pn_wb_dat_t> PN_NAME(wb_dat_i_master);
    sc_signal<bool> PN_NAME(wb_rty_master);
    sc_signal<bool> PN_NAME(wb_err_master);
    sc_signal<pn_wb_dat_t> PN_NAME(wb_dat_o);
    sc_signal<pn_wb_adr_t> PN_NAME(wb_adr);
    sc_signal<bool> PN_NAME(wb_we);
    sc_signal<sc_uint<3>> PN_NAME(wb_cti);
    sc_signal<sc_uint<2>> PN_NAME(wb_bte);
    sc_signal<bool> PN_NAME(wb_stb);
    sc_signal<bool> PN_NAME(wb_cyc);
    sc_signal<pn_wb_sel_t> PN_NAME(wb_sel);

    sc_signal<bool> PN_NAME(wb_rty_uart);
    sc_signal<bool> PN_NAME(wb_err_uart);
    sc_signal<bool> PN_NAME(wb_ack_uart);
    sc_signal<pn_wb_dat_t> PN_NAME(wb_dat_i_uart);

    sc_signal<bool> PN_NAME(wb_rty_debugger);
    sc_signal<bool> PN_NAME(wb_err_debugger);
    sc_signal<bool> PN_NAME(wb_ack_debugger);
    sc_signal<sc_uint<32>> PN_NAME(wb_dat_i_debugger);
    sc_signal<sc_uint<32>> PN_NAME(wb_dat_o_debugger);
    sc_signal<sc_uint<32>> PN_NAME(wb_adr_debugger);
    sc_signal<sc_uint<4>> PN_NAME(wb_sel_debugger);

    // Methods
    void init_submodules();
};

#endif // __TOP_H__
