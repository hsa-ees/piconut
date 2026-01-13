/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lukas Bauer <lukas.bauer1@tha.de>
                     2024 Marco Milenkovic <Marco.Milenkovic@tha.de>
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

#include "cpu.h"

void m_cpu::pn_trace(sc_trace_file* tf, int level)
{

    // calling trace of submodules
    if(level >= 2)
    {
        nucleus->pn_trace(tf, level);
        membrana->pn_trace(tf, level);
    }
    // Internal traces
}

void m_cpu::init_submodules()
{
    nucleus->clk(clk);
    nucleus->reset(reset);
    nucleus->debug_haltrequest_in(debug_haltrequest_in);
    nucleus->debug_haltrequest_ack_out(debug_haltrequest_ack_out);

    nucleus->msip_in(msip_in);
    nucleus->mtip_in(mtip_in);
    nucleus->meip_in(meip_in);

    nucleus->iport_stb_out(stb_iport[0]);
    nucleus->iport_adr_out(adr_iport[0]);
    nucleus->iport_bsel_out(bsel_iport[0]);
    nucleus->iport_rdata_in(rdata_iport[0]);
    nucleus->iport_ack_in(ack_iport[0]);

    nucleus->dport_stb_out(stb_dport[0]);
    nucleus->dport_we_out(we_dport[0]);
    nucleus->dport_lrsc_out(lrsc_dport[0]);
    nucleus->dport_amo_out(amo_dport[0]);
    nucleus->dport_adr_out(adr_dport[0]);
    nucleus->dport_wdata_out(wdata_dport[0]);
    nucleus->dport_bsel_out(bsel_dport[0]);
    nucleus->dport_rdata_in(rdata_dport[0]);
    nucleus->dport_ack_in(ack_dport[0]);

// TBD: Unify Membrana interfaces
#ifdef PN_CFG_MEMBRANA_IS_MEMBRANA_HW
    membrana->clk(clk);
    membrana->reset(reset);
    // ------------ IPort Signals ------------
    membrana->ip_stb(stb_iport);
    membrana->ip_adr(adr_iport);
    membrana->ip_bsel(bsel_iport);
    membrana->ip_rdata(rdata_iport);
    membrana->ip_ack(ack_iport);

    // ------------ DPort Signals ------------
    membrana->dp_stb(stb_dport);
    membrana->dp_we(we_dport);
    membrana->dp_adr(adr_dport);
    membrana->dp_wdata(wdata_dport);
    membrana->dp_bsel(bsel_dport);
    membrana->dp_rdata(rdata_dport);
    membrana->dp_ack(ack_dport);
    membrana->dp_lr_sc(lrsc_dport);
    membrana->dp_amo(amo_dport);

    // ------------ Wishbone Interface ------------
    membrana->wb_master.ack_i(wb_master.ack_i);
    membrana->wb_master.dat_i(wb_master.dat_i);
    membrana->wb_master.adr_o(wb_master.adr_o);
    membrana->wb_master.dat_o(wb_master.dat_o);
    membrana->wb_master.we_o(wb_master.we_o);
    membrana->wb_master.stb_o(wb_master.stb_o);
    membrana->wb_master.cyc_o(wb_master.cyc_o);
    membrana->wb_master.sel_o(wb_master.sel_o);
    membrana->wb_master.rty_i(wb_master.rty_i);
    membrana->wb_master.err_i(wb_master.err_i);

#endif

#ifdef PN_CFG_MEMBRANA_IS_MEMBRANA_SOFT
    membrana->clk(clk);
    membrana->reset(reset);

    membrana->stb_iport(stb_iport);
    membrana->adr_iport(adr_iport);
    membrana->bsel_iport(bsel_iport);
    membrana->rdata_iport(rdata_iport);
    membrana->ack_iport(ack_iport);

    membrana->stb_dport(stb_dport);
    membrana->we_dport(we_dport);
    membrana->load_reserve(lrsc_dport);
    membrana->amo_dport(amo_dport);
    membrana->adr_dport(adr_dport);
    membrana->wdata_dport(wdata_dport);
    membrana->bsel_dport(bsel_dport);
    membrana->rdata_dport(rdata_dport);
    membrana->ack_dport(ack_dport);
#endif

#ifdef PN_CFG_MEMBRANA_IS_MEMBRANA_AI
    membrana->clk(clk);
    membrana->reset(reset);

    membrana->iport_stb(stb_iport);
    membrana->iport_adr(adr_iport);
    membrana->iport_bsel(bsel_iport);
    membrana->iport_rdata(rdata_iport);
    membrana->iport_ack(ack_iport);

    membrana->dport_stb(stb_dport);
    membrana->dport_we(we_dport);
    membrana->dport_load_reserve(lrsc_dport);
    membrana->dport_amo(amo_dport);
    membrana->dport_adr(adr_dport);
    membrana->dport_wdata(wdata_dport);
    membrana->dport_bsel(bsel_dport);
    membrana->dport_rdata(rdata_dport);
    membrana->dport_ack(ack_dport);
#endif
}

bool m_cpu::state_is_not_halt()
{
    return nucleus->state_is_not_halt();
}
