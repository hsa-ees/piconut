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

#ifndef __CLINT_TOP_H__
#define __CLINT_TOP_H__

#include <systemc.h>
#include <piconut.h>

// PicoNut modules used ...
#include <cpu.h>

#include <clint.h>
#include <uart.h>

SC_MODULE(m_demo_clint)
{
public:
    // Ports

    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    sc_in<bool> PN_NAME(rx_i);
    sc_out<bool> PN_NAME(tx_o);

    // Constructor/Destructors
    SC_CTOR(m_demo_clint)
    {
        SC_METHOD(proc_cmb);
        sensitive << wb_dat_i_clint << wb_dat_i_uart << wb_adr << wb_ack_clint
                  << wb_ack_uart << wb_rty_clint << wb_err_clint << wb_rty_uart
                  << wb_err_uart << wb_dat_o << wb_sel;

        init_submodules();
    }

    // Functions
    void pn_trace(sc_trace_file * tf, int level = 1);

    // Processes
    void proc_cmb();

    // Submodules
    m_cpu* cpu;

    m_clint* clint;
    m_uart* uart;

protected:
    // Internal Signals
    sc_signal<bool> PN_NAME(dummy_low);

    // ---------- Wishbone intermediate signals ----------
    sc_signal<bool> PN_NAME(wb_ack_master);
    sc_signal<bool> PN_NAME(wb_ack_clint);
    sc_signal<bool> PN_NAME(wb_ack_uart);
    sc_signal<pn_wb_dat_t> PN_NAME(wb_dat_i_master);
    sc_signal<sc_uint<32>> PN_NAME(wb_dat_i_clint);
    sc_signal<pn_wb_dat_t> PN_NAME(wb_dat_i_uart);
    sc_signal<pn_wb_dat_t> PN_NAME(wb_dat_o);
    sc_signal<sc_uint<32>> PN_NAME(wb_dat_o_clint);
    sc_signal<pn_wb_adr_t> PN_NAME(wb_adr);
    sc_signal<sc_uint<32>> PN_NAME(wb_adr_clint);
    sc_signal<bool> PN_NAME(wb_we);
    sc_signal<bool> PN_NAME(wb_stb);
    sc_signal<bool> PN_NAME(wb_cyc);
    sc_signal<pn_wb_sel_t> PN_NAME(wb_sel);
    sc_signal<sc_uint<4>> PN_NAME(wb_sel_clint);

    sc_signal<bool> PN_NAME(wb_rty_master);
    sc_signal<bool> PN_NAME(wb_rty_clint);
    sc_signal<bool> PN_NAME(wb_rty_uart);
    sc_signal<bool> PN_NAME(wb_err_master);
    sc_signal<bool> PN_NAME(wb_err_clint);
    sc_signal<bool> PN_NAME(wb_err_uart);

    sc_signal<bool> PN_NAME(msip_signal);
    sc_signal<bool> PN_NAME(mtip_signal);
    sc_signal<bool> PN_NAME(meip_signal);

    // Methods
    void init_submodules();
};

#endif // __CLINT_TOP_H__
