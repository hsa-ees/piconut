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

#include "audio_top.h"

void m_demo_audio::pn_trace(sc_trace_file* tf, int level)
{

    // calling trace of submodules
    if(level >= 2)
    {
        cpu->pn_trace(tf, level);
        uart->pn_trace(tf, level);
        audio->pn_trace(tf, level);
    }
    // Internal traces
}

void m_demo_audio::init_submodules()
{

    // ----------- Create submodules -----------
    // ----------- PicoNut -----------
    cpu = sc_new<m_cpu>("i_cpu");
    cpu->clk(clk);
    cpu->reset(reset);

    cpu->debug_haltrequest_in(dummy_low);
    // Issue: vh_const not working right now. Uncomment the line below
    // after vh_const updated and remove dummy signal.
    // cpu->debug_haltrequest_in(vh_const<bool>(0));
    cpu->debug_haltrequest_ack_out(vh_open);

#ifdef PN_CFG_MEMBRANA_IS_MEMBRANA_HW // always defined just to make linter happy
    cpu->wb_master.ack_i(wb_ack_master);
    cpu->wb_master.dat_i(wb_dat_i_master);
    cpu->wb_master.dat_o(wb_dat_o);
    cpu->wb_master.adr_o(wb_adr);
    cpu->wb_master.we_o(wb_we);
    cpu->wb_master.stb_o(wb_stb);
    cpu->wb_master.cyc_o(wb_cyc);
    cpu->wb_master.sel_o(wb_sel);
    cpu->wb_master.rty_i(wb_rty_master);
    cpu->wb_master.err_i(wb_err_master);
#else
#error "PicoNut CPU Wishbone interface not defined!"
#endif

    // Connect the interrupt signals to the PicoNut processor
    cpu->mtip_in(mtip_signal);
    cpu->msip_in(msip_signal);
    cpu->meip_in(meip_signal);

    // ----------- UART -----------
    uart = sc_new<m_uart>("i_uart", PN_CFG_UART_BASE_ADDRESS);

    uart->reset(reset);
    uart->clk(clk);

    uart->rx(rx_i);
    uart->tx(tx_o);

    uart->wb_slave.ack_o(wb_ack_uart);
    uart->wb_slave.dat_i(wb_dat_o);
    uart->wb_slave.dat_o(wb_dat_i_uart);
    uart->wb_slave.adr_i(wb_adr);
    uart->wb_slave.we_i(wb_we);
    uart->wb_slave.stb_i(wb_stb);
    uart->wb_slave.cyc_i(wb_cyc);
    uart->wb_slave.sel_i(wb_sel);
    uart->wb_slave.rty_o(wb_rty_uart);
    uart->wb_slave.err_o(wb_err_uart);

    // ----------- CLINT -----------
    clint = sc_new<m_clint>("i_clint");
    clint->reset(reset);
    clint->clk(clk);

    clint->wb_ack_o(wb_ack_clint);
    clint->wb_dat_i(wb_dat_o_clint);
    clint->wb_dat_o(wb_dat_i_clint);
    clint->wb_we_i(wb_we);
    clint->wb_stb_i(wb_stb);
    clint->wb_cyc_i(wb_cyc);
    clint->wb_sel_i(wb_sel_clint);
    clint->wb_rty_o(wb_rty_clint);
    clint->wb_err_o(wb_err_clint);
    clint->wb_adr_i(wb_adr_clint);

    clint->msip_o(msip_signal);
    clint->mtip_o(mtip_signal);

    // ----------- AUDIO -----------
    audio = sc_new<m_audio>("i_audio");

    audio->reset(reset);
    audio->clk(clk);

    audio->wb_ack_o(wb_ack_audio);
    audio->wb_dat_i(wb_dat_o_clint);
    audio->wb_dat_o(wb_dat_i_audio);
    audio->wb_we_i(wb_we);
    audio->wb_stb_i(wb_stb);
    audio->wb_cyc_i(wb_cyc);
    audio->wb_sel_i(wb_sel_clint);
    audio->wb_rty_o(wb_rty_audio);
    audio->wb_err_o(wb_err_audio);
    audio->wb_adr_i(wb_adr_clint);

    audio->audio_r(audio_r_16);
    audio->audio_l(audio_l_16);
}

void m_demo_audio::proc_comb()
{
    dummy_low.write(0);
    meip_signal.write(0); // No external interrupts in this demo

    audio_r = audio_r_16.read() >> (16 - AUDIO_BIT_WIDTH);
    audio_l = audio_l_16.read() >> (16 - AUDIO_BIT_WIDTH);
}

void m_demo_audio::proc_comb_wb()
{
    wb_adr_clint = wb_adr.read().range(31, 0);
    wb_dat_o_clint = wb_dat_o.read().range(31, 0);
    wb_sel_clint = wb_sel.read().range(3, 0);

#define NUM_AUDIO (1U << (PN_CFG_AUDIO_EXP + 1))
    // Check for AUDIO address range
    if(wb_adr.read() >= PN_CFG_AUDIO_BASE_ADDRESS && wb_adr.read() < (PN_CFG_AUDIO_BASE_ADDRESS + (PN_AUDIO_SIZE_BYTE * NUM_AUDIO * 2)))
    {
        wb_dat_i_master = (sc_uint<32>(0), wb_dat_i_audio.read());
        wb_ack_master = wb_ack_audio.read();
        wb_rty_master = wb_rty_audio.read();
        wb_err_master = wb_err_audio.read();
    }
    // Check for CLINT address range
    else if(wb_adr.read() >= PN_CFG_CLINT_BASE_ADDRESS && wb_adr.read() < (PN_CFG_CLINT_BASE_ADDRESS + CLINT_SIZE))
    {
        wb_dat_i_master = (sc_uint<32>(0), wb_dat_i_clint.read());
        wb_ack_master = wb_ack_clint.read();
        wb_rty_master = wb_rty_clint.read();
        wb_err_master = wb_err_clint.read();
    }
    // Default to UART
    else
    {
        wb_dat_i_master = wb_dat_i_uart.read();
        wb_ack_master = wb_ack_uart.read();
        wb_rty_master = wb_rty_uart.read();
        wb_err_master = wb_err_uart.read();
    }
}
